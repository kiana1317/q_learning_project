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
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.processReward )
        # Creats a twist object
        self.twist = Twist()

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.reached_dumbbell = False
        self.action_matrix = [[] for x in range(64)]
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


    def get_random_action(step1):
        possible_actions = self.action_matrix[step1]
        action = random.choice(possible_actions)
        while action == -1:
            action = random.choice(possible_actions)
        return action
        
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

    
    
    def processScan(self, data):
        # pass
        if not self.initialized:
            return

        # Move the robot to the dumbbell
        if data.ranges[0] < 0.27:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            self.reached_dumbbell = True
        elif not self.reached_dumbbell:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        
    def close_grip(self):
        # close grip
        gripper_joint_goal = [.005,-.005]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def open_grip(self):
        # open grip
        gripper_joint_goal = [.01,-.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def lift_dumbbell(self):
         arm_joint_goal = [0,
            math.radians(50.0),
            math.radians(-30),
            math.radians(-20)]
        # wait=True ensures that the movement is synchronous
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

    def place_dumbbell(self):
        # Lower the dumbbel to ground level
        arm_joint_goal = [0,
            math.radians(50.0),
            math.radians(-40.0),
            math.radians(-20.0)]
        # wait=True ensures that the movement is synchronous
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()
        self.open_grip()
        self.twist.linear.x = -0.1
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)

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
