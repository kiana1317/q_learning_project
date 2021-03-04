#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import keras_ocr


from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan, Image
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward 

import time
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

class Movement(object):

    def __init__(self):

        self.initialized = False

        # initialize this node
        rospy.init_node('robot_movement')

        # ROS publish to the /cmd_vel topic to update angular and linear velocity
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # ROS subscribe to the Scan topic to monitor items in the robot's va
        rospy.Subscriber("/scan", LaserScan, self.processScan)

        # ROS subscribe to Image topic to get rgb images
        rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)

        # Creats a twist object
        self.twist = Twist()

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # intialize image data to be able to process block/dumbbell image
        self.image = None

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # initialize pipeline for keras_ocr
        self.pipeline = keras_ocr.pipeline.Pipeline()

        # Position the arm
        self.starting_arm_position()

        # Action Sequence based on Convergence (color, box)
        self.action_sequence = [('Blue',2),('Red',3),('Green',1)]

        # Variables to check states
        self.moved_dumbbells = 0
        self.block_found = False
        self.reseting = False
        self.scanDataRanges = []
        self.has_dumbbell = False
        self.placed_dumbbell = False
        self.passed_blocks = set()
        self.shift = True
        
        self.initialized = True

    def complete_action(self):
        if not self.has_dumbbell:
            self.process_dumbbells()
            return
        elif not self.placed_dumbbell:
            self.process_blocks()
            return
        elif self.reseting:
            self.reset()
            return
    

    def processScan(self, data):
        if not self.initialized:
            return
        # Store global scan data
        self.scanDataRanges = data.ranges
        
    
    def reset(self):
        # reinitialize variables
        self.has_dumbbell = False
        self.placed_dumbbell = False
        self.block_found = False
        self.reseting = False
        self.shift = True
        self.passed_blocks = set()

    def starting_arm_position(self):
        arm_joint_goal = [0,.4,.5,-.9]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.open_grip()


    def close_grip(self):
        gripper_joint_goal = [.007,.007]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def open_grip(self):
        gripper_joint_goal = [.01,.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def lift_dumbbell(self):
        arm_joint_goal = [0,
            math.radians(0.0),
            math.radians(-20),
            math.radians(-20)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

    def place_dumbbell(self):
        # Place dumbbell on the groun
        self.open_grip()
        arm_joint_goal = [0,.4,.5,-.9]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        # Reverse away from the dumbbbell
        self.twist.linear.x = -0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)
        # Stop
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)



    def image_callback(self, data):
        # keeps image updated for processing dumbbells/blocks
        self.image = data

    def process_dumbbells(self):
        if not self.initialized:
            return

        if not self.scanDataRanges:
            return
        # Assumes robot is at the original/reset position and returns relative
        # location of dumbbells as a dict
        # 'L', 'C', 'R' stands for left, center, and right respectively

        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        rospy.sleep(0.1)
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        curr_color = self.action_sequence[self.moved_dumbbells][0]

        # define upper & lower bounds for each color
        # found these values at 
        # https://pysource.com/2019/02/15/detecting-colors-hsv-color-space-opencv-with-python/
        # make mask and get moment of each color to find center
        if curr_color == "Red":
            lower_color = np.array([0, 120, 70])
            upper_color = np.array([10, 255, 255])
        elif curr_color == "Blue":
            lower_color = np.array([94, 80, 2])
            upper_color = np.array([126, 255, 255])
        else:
            lower_color = np.array([25, 52, 72])
            upper_color = np.array([102, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        M = cv2.moments(mask)

        # we now erase all pixels
        h, w, d = image.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # if the color is found 
        if M['m00'] > 0:
            #  Robot is near the dumbbell
            if self.scanDataRanges[0] < 0.2:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                # pickup
                self.lift_dumbbell()
                self.has_dumbbell = True
                self.reseting = True

            else:
                # find pixel of x center of each mask
                cx = M['m10']/M['m00']
                cy = M['m01']/M['m00']
                err = w/2 - cx
                k_p = 0.001
                self.twist.linear.x = 0.1
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)

        else:
            # Rotate until the dumbbell is in sight
            self.twist.linear.x = 0
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)

    def process_blocks(self):
        if not self.scanDataRanges:
            return
        # Assumes robot is at the original/reset position and returns relative
        # location of blocks as a dict
        # 'L', 'C', 'R' stands for left, center, and right respectively
        
        # point camera towards blocks
        rospy.sleep(0.1)

       # Use color to locate the center of the block
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # detect black numbers
        lower_color = np.array([0, 0, 0])
        upper_color = np.array([180, 255, 30])

        mask = cv2.inRange(hsv, lower_color, upper_color)
        M = cv2.moments(mask)

        # No number is currently in view
        if M['m00'] <= 0:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.4
            self.cmd_vel_pub.publish(self.twist)  
            return
        # Block has been found
        if self.block_found:
            
            # the block is near
            if self.scanDataRanges[0] < 0.5:

                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                self.place_dumbbell()
                self.placed_dumbbell = True
                self.moved_dumbbells += 1
                return
            else:
                # erase all pixels
                h, w, d = image.shape
                search_top = int(3*h/4)
                search_bot = int(3*h/4 + 20)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                cx = M['m10']/M['m00']
                cy = M['m01']/M['m00']
                err = w/2 - cx
                k_p = 0.001
                self.twist.linear.x = 0.2
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
                return

        # Stop to read the block
        self.twist.linear.x = 0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist) 

        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='rgb8')
        nums_detected = self.pipeline.recognize([image])[0]
        
        # If it cannot read the block
        if not nums_detected:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.01
            self.cmd_vel_pub.publish(self.twist) 
            rospy.sleep(.5)
            return


        # Possible predictions from the Keras detection
        num_predictions ={
            "1":1,
            "2":2,
            "3":3,
            "s":3,
            "l":1,
            "d":3,
            "8":3,
            "7":1,
            "t":1,
            "a":2,
            "5":3,
            "b":3,
            "i":1,
            "c":2
        }

        curr_block = self.action_sequence[self.moved_dumbbells][1]
        block = num_predictions[nums_detected[0][0]]
        self.passed_blocks.add(block)

        if block == curr_block:
            self.block_found = True
            return
                
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)  
            rospy.sleep(2)
            return

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.moved_dumbbells != 3:
                self.complete_action()
            r.sleep()


if __name__=="__main__":
    node = Movement()
    node.run()
