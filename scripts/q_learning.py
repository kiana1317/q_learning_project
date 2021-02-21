#!/usr/bin/env python3
import rospy


from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward 

import time
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

class QLearning(object):

    def __init__(self):
        # once everything is setup initialized will be set to true
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

        self.pose = rospy.Publisher("/pose")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        rospy.Subscriber("/odometry/filtered", Odometry, self.newOdom)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.in_front_of_dumbbells = False
        self.found_dumbell = False
        
        self.initialized = True
  
    def move_arm(self):
        # arm_joint_goal is a list of 4 radian values, 1 for each joint
        # for instance,
        #           arm_joint_goal = [0.0,0.0,0.0,0.0]
        # arm_joint_goal = [0.0,
        #     math.radians(5.0),
        #     math.radians(10.0),
        #     math.radians(-20.0)]
        # # wait=True ensures that the movement is synchronous
        # self.move_group_arm.go(arm_joint_goal, wait=True)
        # # Calling ``stop()`` ensures that there is no residual movement
        # self.move_group_arm.stop()

        # # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        # # for instance,
        # gripper_joint_goal = [0.009,0.0009]
        # #           gripper_joint_goal = [0.0, 0.0]
        # self.move_group_gripper.go(gripper_joint_goal, wait=True)
        # self.move_group_gripper.stop()
        pass

    def find_dumbell(self,dir):
        pass

    def processScan(self, data):
        if data.ranges[0] == float("inf"):
            print("Waiting to scan")
            return
        # print("Made it")
        # max_val = max(data.ranges[0:90])
        # max_index = data.ranges.index(max_val)
        
        # if not self.found_dumbell:
        #     # check where the dumbell
        #     # (1 = left, 2 = center, 3 = right)
        #     # postions = {1:-1,2:0,3:1}
        #     # direc = postions[random.randint(1, 3)]
        #     direc = 1
        #     self.twist.angular.z = .394*direc
        #     self.twist.linear.x = 0.0
        #     self.cmd_vel_pub.publish(self.twist)
        #     rospy.sleep(1)
        #     self.found_dumbell = True
        #     return

        # if data.ranges[0] < 1:
        #     self.twist.linear.x = 0
        #     self.twist.angular.z = 0
        #     self.cmd_vel_pub.publish(self.twist)
        # else:
        #     self.twist.linear.x = 0.1
        #     self.twist.angular.z = 0
        #     self.cmd_vel_pub.publish(self.twist)
        

        # move robot forward till in front of center dumbbell
        # if data.ranges[0] < 0.5 and not self.in_front_of_dumbbells:
        #     self.twist.linear.x = 0
        #     self.twist.angular.z = 0
        #     self.cmd_vel_pub.publish(self.twist)
        #     self.in_front_of_dumbbells = True
        # elif not self.in_front_of_dumbbells:
        #     self.twist.linear.x = 0.1
        #     self.twist.angular.z = 0
        #     self.cmd_vel_pub(self.twist)
        #     return
        
        # # check where the dumbell
        # # (1 = left, 2 = center, 3 = right)
        # # postions = [1,2,3]
        # # direc = postions[math.floor(math.random(3))]
        # direc = 1
        # # Offsets the transition time to meet the velocity
        # offset = .0053 
        # if direc == 1:
        #     # turn left
        #     # Angle difference for 4 seconds is 0.3927
        #     self.twist.angular.z = -.3927 + offset
        #     self.twist.linear.x = 0.0
        #     self.cmd_vel_pub.publish(self.twist)
        #     rospy.sleep(4)

        #     self.twist.angular.z = 0
        #     self.twist.linear.x = 0.1
        #     self.cmd_vel_pub.publish(self.twist)
        #     rospy.sleep(4)

        #     self.twist.angular.z = .3927 + offset
        #     self.twist.linear.x = 0.0
        #     self.cmd_vel_pub.publish(self.twist)
        #     rospy.sleep(4)


        # elif direc == 3:
        #     # turn left
        #     # Angle difference for 4 seconds is 0.3927
        #     self.twist.angular.z = .3927 + offset
        #     self.twist.linear.x = 0.0
        #     self.publisher.publish(self.twist)
        #     rospy.sleep(4)
        

        
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
