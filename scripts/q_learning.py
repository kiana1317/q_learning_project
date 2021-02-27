#!/usr/bin/env python3

import rospy
import numpy as np

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward, QMatrixRow 

import time
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
import random

class QLearning(object):

    def __init__(self):
        self.initialized = False
        # initialize this node
        rospy.init_node('q_learning')

        # ROS subscribe to the topic publishing actions for the robot to take
        # Every time you want to execute an action, publish a message to this topic 
        # (this is the same topic you'll be subscribing to in the node you write to have your robot execute the actions).
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.robot_actions)

        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action",RobotMoveDBToBlock, queue_size=10 )

        # ROS subscribe to the Gazebo topic publishing the locations of the models
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_received)

        # ROS publish to the Gazebo topic to set the locations of the models
        self.model_states_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

        # ROS publish to the /cmd_vel topic to update angular and linear velocity
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # ROS subscribe to the Scan topic to monitor items in the robot's va
        rospy.Subscriber("/scan", LaserScan, self.processScan)

        # ROS publish to Q Matrix to update Q-Learning Matrix
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        
        #ROS subscribe to Rewards to receive from the environment the reward after each action you take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.processReward)

        # intialize Q matrix, reward, minimum number of iterations to do before 
        # checking convergence, current iteraton, and convergence
        self.q_matrix = QMatrix()
        self.initialize_q_matrix()
        self.reward = None
        self.min_iterations = 500
        self.current_iteration = 0
        self.converged = False

        # intialize current state, next state, and action taken
        self.current_state = 0
        self.next_state = -1
        self.action = -1

        # Creats a twist object
        self.twist = Twist()

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        # self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        # self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.reached_dumbbell = False
        self.action_matrix = [[] for x in range(64)]
        self.initialize_action_matrix()
        self.initialized = True

        # Position the arm
        self.move_arm()

    def initialize_action_matrix(self):
        
        action_matrix = []
        for b in range(4):
            for g in range(4):
                for r in range(4):
                    action_matrix.append((r,g,b))
        
        starting_state, next_state = action_matrix
        for step1_index, step1 in starting_state.enumerate():
            for step2_index, step2 in next_state.enumerate():
                # Check if steps are valid
                if not self.is_valid_step(step1, step2):
                    self.action_matrix[step1_index][step2_index] = -1
                else:
                    # find action
                    self.action_matrix[step1_index][step2_index] = self.get_action(step1,step2)


    def is_valid_step(self,step1, step2):
        steps = [step1, step2]
        
        # Check if the starting positions of step1 and step2 are valid
        # No dumbbells are at the same position
        num_blocks = {0: 1, 1: 0}
        for index, (r,g,b) in steps.enumerate():
            if r != 0:
                num_block[index] += 1
                if r == g or r == b :
                    return False
            if g != 0:
                num_block[index] += 1
                if b == g:
                    return False 
            if b != 0:
                num_block[index] += 1

        # Check if more than one block are moved from step1 to step2
        # Count moved blocks
        if num_blocks[1] != num_blocks[0] + 1:
            return False
        
        return True

    def get_action(step1, step2):
        actions = []
        # red = 1, green = 2, blue = 3
        for color in range(1,4):
            for box in range(1,4):
                actions.append((color,box))

        for i in range(3):
          if step1[i] != step2[i]:
              color = i + 1
              box = step2[i] - 1
            #   returns action number
              return actions.index((color,box))  


    def get_random_action(self):
        # gets valid random action given current state
        possible_actions = self.action_matrix[self.current_state]
        action = random.choice(possible_actions)
        while action == -1:
            action = random.choice(possible_actions)
        self.action = action
        self.next_state = possible_actions.index(action)
        
    def move_arm(self):
        # arm_joint_goal is a list of 4 radian values, 1 for each joint
        # for instance,
        #           arm_joint_goal = [0.0,0.0,0.0,0.0]
        if not self.initialized:
            print("Initializing...")
            return
        arm_joint_goal = [0,
            math.radians(50.0),
            math.radians(-40),
            math.radians(-20)]
        # wait=True ensures that the movement is synchronous
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

        # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        self.open_grip()

    
    
    def robot_actions(self, data):
        pass

    def model_states_received(self, data):
        pass

    def initialize_q_matrix(self):
        # loop over 64 rows and 9 cols putting 0 in each
        for i in range(64):
            x = QMatrixRow()
            for j in range(9):
                x.q_matrix_row.append(0)
            self.q_matrix.q_matrix.append(x)
        self.q_matrix_pub.publish(self.q_matrix)
        print(self.q_matrix.q_matrix[0].q_matrix_row[0])


    def processReward(self, data):
        # allows reward to be processed by q matrix
        self.reward = data

    def check_convergence(self):

        # check if iterated minmum amount of times
        if self.current_iteration < self.min_iterations:
            return
        
        self.converged = True
        print("The matrix has converged!")

    def update_q_matrix(self):
        alpha = 1
        gamma = 0.5

        # get value of row from state & action
        current_val = self.q_matrix.q_matrix[self.current_state].q_matrix_row[self.action]
        
        # get max value of all actions for state2
        max_a = self.q_matrix[self.next_state].max()

        # update q matrix for state1 & action_t
        self.q_matrix.q_matrix[self.current_state].q_matrix_row[self.action]  += \
            alpha * (self.reward + gamma * max_a  - current_val)

        # publish Q matrix
        self.q_matrix_pub.publish(self.q_matrix)

        # update current state
        self.current_state = self.next_state


    def run(self):
        """
        # while Q matrix is not converged
        while self.converged is False:
            
            # check whether we should update convergence
            self.check_convergence()
            
        """

        rospy.spin()


if __name__=="__main__":
    node = QLearning()
    node.run()
