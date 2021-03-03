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

        # ROS subscribe to Image topic to get rgb images
        rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)

        # ROS subscribe to Odometry to get pose information
        rospy.Subscriber('/odom', Odometry, self.get_rotation)

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

        # initialize dictionaries containing dumbbell & block location
        self.dumbbell_location = {'Red': None, 'Blue': None, 'Green': None}
        self.block_location = {1: None, 2: None, 3: None}

        # intialize image data to be able to process block/dumbbell image
        self.image = None

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # initialize euler orientation (3rd value of tuple is yaw)
        self.euler_orientation = None

        # initialize pipeline for keras_ocr
        self.pipeline = keras_ocr.pipeline.Pipeline()

        self.has_dumbbell = False

        self.reached_dumbbell = False
        self.reached_block = False

        # Process dumbbells and blocks position
        # self.process_dumbbells()
        # self.process_blocks()

        # Position the arm
        self.starting_arm_position()

        # Action Sequence based on Convergence (color, box)
        # 1 = red, 2 = green, 3 = blue
        self.action_sequence = [('Blue',2),('Red',1),('Green',3)]
        self.moved_dumbbells = 0
        
        # for testing
        self.dumbbell_location = {'Red': 'R', 'Blue': 'C', 'Green': 'L'}
        self.block_location = {1: 'L', 2: 'C', 3: 'R'}
        
        self.block_center={'L':None,'C':None,'R':None}
        # self.actions_completed = 0
        
        self.dumbbell_move_in_progress = False
        self.dumbbell_found = False
        self.block_found = False
        self.passed_block_hold =False
        self.reseting = False
        self.scanDataRanges = []

        self.picked_up_block = False
        self.lowered_block = False 
        self.has_dumbbell = False
        self.placed_dumbbell = False
        self.passed_blocks = set()
        self.shift = True

        # States
        # Find db, move to dumbbell towads dumbbell, picking-up dumbbell, finding

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
        self.open_grip()
        arm_joint_goal = [0,.4,.5,-.9]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        # Reverse away from the dumbbbell
        self.twist.linear.x = -0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)



    def image_callback(self, data):
        # keeps image updated for processing dumbbells/blocks
        self.image = data

    def process_dumbbells(self):
        if not self.scanDataRanges:
            return
        print("processed")
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
            print("see dumbbell")
            #  if the robot is near the dumbbell
            #
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
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2
            self.cmd_vel_pub.publish(self.twist)

    def get_rotation(self, data):
        # get yaw from rotation data
        q = data.pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        self.euler_orientation = euler_from_quaternion(orientation_list)

    def target_turn(self, target):
        # turns turtlebot to target yaw 
        
        # initalize k, convert target to radians, & get yaw
        k = 0.5
        target_rad = target * math.pi / 180
        # self.twist.linear.x = 0
        # keep turning until reaches target
        while abs(target_rad - self.euler_orientation[2]) > 0.03:
            self.twist.angular.z = k * (target_rad - self.euler_orientation[2])
            self.cmd_vel_pub.publish(self.twist)

        # stop once target reached
        self.twist.angular.z = 0
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
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)  
            return
        # Already Found block
        if self.block_found:
            print("Block found")
            print(self.scanDataRanges)
            if self.scanDataRanges[0] < 0.5:
                print("place dumbbell")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                # pickup
                self.place_dumbbell()
                self.placed_dumbbell = True
                self.moved_dumbbells += 1
                return
            # elif self.scanDataRanges[0] == float("inf"):
            else:
                # we now erase all pixels
                h, w, d = image.shape
                search_top = int(3*h/4)
                search_bot = int(3*h/4 + 20)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                cx = M['m10']/M['m00']
                cy = M['m01']/M['m00']
                err = w/2 - cx
                k_p = 0.0001
                self.twist.linear.x = 0.1
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
                return

        # Need to search for block
        self.twist.linear.x = 0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist) 

        # Check the block
        curr_block = self.action_sequence[self.moved_dumbbells][1]
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='rgb8')
        nums_detected = self.pipeline.recognize([image])[0]
        
        if not nums_detected:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.01
            self.cmd_vel_pub.publish(self.twist) 
            rospy.sleep(.5)
            return


        # if nums_detected
        
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
        print(nums_detected[0][0])
        block = num_predictions[nums_detected[0][0]]
        self.passed_blocks.add(block)
        print(block)
        if block == curr_block:
            self.block_found = True
            return
                
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)  
            rospy.sleep(2)
            return
       


    def robot_actions(self, data):
        pass

    def model_states_received(self, data):
        pass

    def processReward(self, data):
        pass

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.moved_dumbbells != 3:
                self.complete_action()
            r.sleep()


if __name__=="__main__":
    node = Movement()
    node.run()

# Problem: Memory Allocation when running image recognition