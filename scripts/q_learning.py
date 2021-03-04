#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward, QMatrixRow 

import time
import math
import random

class QLearning(object):

    def __init__(self):
        self.initialized = False
        # initialize this node
        rospy.init_node('q_learning', disable_signals=True)

        # ROS publish to robot action so that phantom movement can occur
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action",RobotMoveDBToBlock, queue_size=10 )

        # ROS publish to the Gazebo topic to set the locations of the models
        self.model_states_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)


        # ROS publish to Q Matrix to update Q-Learning Matrix
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        
        #ROS subscribe to Rewards to receive from the environment the reward after each action you take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.processReward)

        # intialize Q matrix, reward, minimum number of iterations to do before 
        # checking convergence, current iteraton, and convergence
        self.q_matrix = QMatrix()
        self.initialize_q_matrix()
        self.reward = None
        self.min_iterations = 150
        self.num_iterations_eps = 0
        self.current_iteration = 1
        self.num_rewards = 0
        self.num_actions_pub = 0
        self.converged = False
        self.past_iteration = -1
        self.num_iterations = 0

        self.final_actions = []

        # intialize current state, next state, and action taken
        self.current_state = 0
        self.next_state = -1
        self.action = -1


        # Create action matrix
        self.action_matrix = [[] for x in range(64)]
        for i in range(64):
            self.action_matrix[i] = [0 for x in range(64)]

        # Initialize acton matrix
        self.initialize_action_matrix()

        self.initialized = True

        

    def initialize_action_matrix(self):
        
        # Populate State Space
        state_space = []
        for b in range(4):
            for g in range(4):
                for r in range(4):
                    state_space.append((r,g,b))
        
        # Iterate through step1 and step2 sequences
        for s1_index, s1 in enumerate(state_space):
            for s2_index, s2 in enumerate(state_space):
                # Check if steps are valid
                if not self.is_valid_step(s1, s2):
                    self.action_matrix[s1_index][s2_index] = -1
                else:
                    self.action_matrix[s1_index][s2_index] = self.get_action(s1,s2)


    def is_valid_step(self,s1, s2):
        # Count moved blocks
        moved_blocks = {0: 0, 1: 0}

        # Check no dumbbells are at the same position
        steps = [s1, s2]
        for index, (r,g,b) in enumerate(steps):
            if r != 0:
                moved_blocks[index] += 1
                if r == g or r == b :
                    return False
            if g != 0:
                moved_blocks[index] += 1
                if b == g:
                    return False 
            if b != 0:
                moved_blocks[index] += 1

        # check if dumbbells move from anyplace other than origin
        for i in range(3):
            if s1[i] != 0 and s1[i] != s2[i]:
                return False
         

        # Check anything other than one block is moved from step1 to step2
        if moved_blocks[1] != moved_blocks[0] + 1:
            return False
        return True

    def get_action(self, s1, s2):
        actions = []
        # red = 1, green = 2, blue = 3
        # 1,2,3 box numbers respectively
        for color in range(1,4):
            for box in range(1,4):
                actions.append((color,box))

        for i in range(3):
          if s1[i] != s2[i]:
              color = i + 1
              box = s2[i]
              return actions.index((color,box))  


    def get_random_action(self, step1):
        # gets random action
        if not self.initialized:
            return 
        possible_actions = self.action_matrix[step1]
        action = random.choice(possible_actions)
        while action == -1:
            action = random.choice(possible_actions)


        # sets action & next state according to random action
        self.action = action
        self.next_state = possible_actions.index(action)

        db, block_num = self.get_action_tuple(self.action)

        rospy.sleep(1)

        # publish action
        pub_action = RobotMoveDBToBlock(robot_db=db, block_id = block_num)
        self.robot_action_pub.publish(pub_action)
        self.action_sent = True
        self.num_actions_pub += 1


    def get_action_tuple(self, action):
        # get action as a tuple (dumbbell, block_id)

        # determine if r/g/b dumbbell
        if 0 <= action < 3:
            db = "red"
        elif 3 <= action < 6:
            db = "green"
        elif 6 <= action < 9:
            db = "blue"
        else:
            print("Invalid action")
        
        # determine block num
        block_num = int((action % 3) + 1)

        return (db, block_num)



    def initialize_q_matrix(self):
        # loop over 64 rows and 9 cols putting 0 in each
        for i in range(64):
            x = QMatrixRow()
            for j in range(9):
                x.q_matrix_row.append(0)
            self.q_matrix.q_matrix.append(x)
        # publish q matrix
        self.q_matrix_pub.publish(self.q_matrix)


    def processReward(self, data):
        # allows reward to be processed by q matrix
        self.reward = data
        self.num_rewards += 1

        # check that there are only 3 rewards returned per action seqn
        # if more returned don't compute any further (messes up queue)
        if self.reward.iteration_num == self.past_iteration:
            self.num_iterations += 1
        else:
            self.num_iterations = 1

        if self.num_iterations > 3:
            return 

        # updates q matrix
        self.update_q_matrix()

        self.past_iteration = self.reward.iteration_num

        # updates state when world is reset
        if self.current_iteration % 3 == 0:
            self.current_state = 0
            rospy.sleep(0.5)

        # checks if matrix converged
        if self.converged is False:

            # update iteration
            self.current_iteration += 1


            # check whether we should update convergence
            self.check_convergence()

            # update next action
            self.get_random_action(self.current_state)
        

    def check_convergence(self):
        # check if iterated minmum amount of times
        if self.current_iteration < self.min_iterations:
            return
        
        #checks if converged & makes sure that world is reset
        if self.num_iterations_eps >= 30 and sum(self.q_matrix.q_matrix[0].q_matrix_row) > 0 \
            and self.current_iteration % 3 == 0:
            self.converged = True
            self.get_final_actions()
            print("The matrix has converged!")



    def update_q_matrix(self):
        alpha = 1
        gamma = 0.5
        eps = 10

        # get value of row from state & action
        current_val = self.q_matrix.q_matrix[self.current_state].q_matrix_row[self.action]
        
        # get max value of all actions for state2
        max_a = max(self.q_matrix.q_matrix[self.next_state].q_matrix_row)

        # update q matrix for state1 & action_t
        self.q_matrix.q_matrix[self.current_state].q_matrix_row[self.action]  += \
            int(alpha * (self.reward.reward + gamma * max_a  - current_val))
        
        post_val = self.q_matrix.q_matrix[self.current_state].q_matrix_row[self.action]

        # update amount of times the the q matrix values don't really change
        if abs(post_val - current_val) < eps:
            self.num_iterations_eps += 1
        else: 
            self.num_iterations_eps = 0


        # publish Q matrix
        self.q_matrix_pub.publish(self.q_matrix)

        # update current state
        self.current_state = self.next_state


    def get_final_actions(self):
        # get the actions that the bot should taken given q matrix
        state = 0
        # loop through and return actions robot should take
        for i in range(2):
            # get row of current state & best next action
            q_mat_row = self.q_matrix.q_matrix[state].q_matrix_row
            best_action = q_mat_row.index(max(q_mat_row))

            # convert action to tuple to use in movement
            action_tuple = self.get_action_tuple(best_action)

            self.final_actions.append(action_tuple)

            # get next state
            state = self.action_matrix[state].index(best_action)
        

        # added this code to allow valid sequence to be returned
        # so that movement could still be integrated
        actions = self.final_actions
        possible_colors = ['red', 'blue', 'green']
        possible_blocks = [1, 2, 3]

        # remove colors & blocks already in seqn
        possible_colors.remove(actions[0][0])
        possible_colors.remove(actions[1][0])
        possible_blocks.remove(actions[0][1])
        possible_blocks.remove(actions[1][1])

        # add tuple to list of actions to take
        self.final_actions.append((possible_colors[0], possible_blocks[0]))

        print("Here are the final actions", self.final_actions)
    
    def send_final_actions(self):
        # sends the final actions to movement algo

        for i in range(3):
            db, block_num = self.final_actions[i]

            # publish action
            pub_action = RobotMoveDBToBlock(robot_db=db, block_id = block_num)
            self.robot_action_pub.publish(pub_action)

            rospy.sleep(0.5)



            

    def run(self):
        # wait for node to initialize fully
        while self.initialized is False:
            continue
        
        #wait fully for initializtion
        rospy.sleep(1)
        
        # begin q learning loop
        self.get_random_action(self.current_state)
        
            


if __name__=="__main__":
    node = QLearning()
    node.run()

    input("Press enter to continue once matrix has converged...")

    node.send_final_actions()
