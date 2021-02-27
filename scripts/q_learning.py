#!/usr/bin/env python3

import rospy


from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward 

import time
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

        # Creats a twist object
        self.twist = Twist()

        # ROS subscribe to the Scan topic to monitor items in the robot's va
        rospy.Subscriber("/scan", LaserScan, self.processScan)

        # ROS publish to Q Matrix to update Q-Learning Matrix
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        
        #ROS subscribe to Rewards to receive from the environment the reward after each action you take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.processReward )

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
        moved_blocks = {0: 1, 1: 0}

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
        if not self.initialized:
            return
        possible_actions = self.action_matrix[step1]
        action = random.choice(possible_actions)
        while action == -1:
            action = random.choice(possible_actions)
        return action

    
    
    def processScan(self, data):
        pass

    def robot_actions(self, data):
        pass

    def model_states_received(self, data):
        pass

    def processReward(self, data):
        pass

    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = QLearning()
    node.run()
