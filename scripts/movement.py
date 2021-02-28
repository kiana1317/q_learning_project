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

        self.has_block = False

        self.reached_dumbbell = False
        self.reached_block = False

        # Process dumbbells and blocks position
        # self.process_dumbbells()
        # self.process_blocks()

        # Position the arm
        self.move_arm()

        # Action Sequence based on Convergence (color, box)
        # 1 = red, 2 = green, 3 = blue
        self.action_sequence = [('Red',2),('Blue',1),('Green',3)]
        self.moved_dumbbells = []
        # for testing
        self.dumbbell_location = {'Red': 'R', 'Blue': 'C', 'Green': 'L'}
        self.block_location = {1: 'L', 2: 'C', 3: 'R'}

        # self.actions_completed = 0
        self.dumbbell_move_in_progress = False
        self.dumbbell_found = False
        self.block_found = False
        self.passed_block_hold =False

        self.initialized = True

    def processScan(self, data):
        locs = {'R':0, 'C':1, 'L':2}
        if not self.initialized:
            return
        if not self.dumbbell_move_in_progress:
            color = self.action_sequence[len(self.moved_dumbbells)][0]
            loc = self.dumbbell_location[color]
            pos = locs[loc]
            self.find_dumbbell(data.ranges,pos)
            self.dumbbell_move_in_progress = True
            self.dumbbell_found = True
            return 

        # Move the robot to the dumbbell, and lift in the air, and turn towards the blocks
        if data.ranges[0] < 0.2 and not self.reached_dumbbell:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            self.reached_dumbbell = True

            self.close_grip()
            self.lift_dumbbell()
            self.target_turn(179)
            self.has_block = True 
            return
        elif not self.reached_dumbbell:
            k = 0.1 * self.dumbbell_center(data.ranges, 15)
            self.twist.linear.x = 0.1
            self.twist.angular.z = k
            self.cmd_vel_pub.publish(self.twist)
            return 

        if not self.block_found:
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(5)
            self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)

            block = self.action_sequence[len(self.moved_dumbbells)][1]
            loc = self.block_location[block]
            pos = locs[loc]
            angles = {0:130, 1:179, 2:-130}
            self.target_turn(angles[pos])
            # self.twist.linear.x = 0.2
            # self.cmd_vel_pub.publish(self.twist)
            # rospy.sleep(3)
            # self.find_dumbbell(data.ranges,pos)
            self.block_found = True
            return
        # Move the robot to the block, and place the dumbbell down
        # try to tell if robot is approaching wall, given block is held
        area_around_dumbbell =  data.ranges[-30:] + data.ranges[:31]
        valid_nums_only = [i for i in area_around_dumbbell if i != float("inf")]
        # close_nums_only = [i for i in valid_nums_only if i < 0.2]
    
        # if len(valid_nums_only) != 0 and max(valid_nums_only) > 0.1:
        #     k = 0.1 * self.dumbbell_center(data.ranges, 30)
        #     self.twist.linear.x = 0.1 * k
        #     self.twist.angular.z = k
        #     self.cmd_vel_pub.publish(self.twist)
        #     return 
        print(data.ranges)

        if (self.has_block and (len(valid_nums_only) != len(area_around_dumbbell) or max(valid_nums_only) < 2)) and not self.passed_block_hold:
            k = 0.5 * self.dumbbell_center(data.ranges, 60)
            self.twist.linear.x = 0.1
            self.twist.angular.z = k
            self.cmd_vel_pub.publish(self.twist)
            return

        self.passed_block_hold = True 
        
        if data.ranges[0] < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            self.place_dumbbell()
            self.target_turn(0)
            self.has_block = False
            return 

        elif self.has_block:
            k = 0.5 * self.dumbbell_center(data.ranges, 60)
            self.twist.linear.x = 0.1
            self.twist.angular.z = k
            self.cmd_vel_pub.publish(self.twist)
            return
    
    def dumbbell_center(self, ranges, bound):
        angles = ranges[-bound:] + ranges[:bound]
        center_angle = ranges.index(min(angles))
        if center_angle > 179:
            center_angle -= 360
        turn = center_angle/bound
        return turn

    def find_dumbbell(self, ranges, pos):
        print(ranges)
        object_not_found = True
        object_start = 0
        object_end = 0
        objects=[]
        # Scan 180 degrees
        angles = list(range(269,360)) + list(range(0,90))
        for i in angles:
            if ranges[i] != float("inf") and object_not_found:
                object_start = i
                object_not_found = False
            elif ranges[i] == float("inf") and not object_not_found:
                object_end = i
                objects.append((object_start,object_end))
                if len(objects) == 3:
                    break

        if 'R' in self.moved_dumbbells:
            if 'C' in self.moved_dumbbells:
                object_start = objects[0][0]
                object_end = objects[0][1]
            else:
                object_start = objects[pos - 1][0]
                object_end = objects[pos - 1][1]
        elif 'C' in self.moved_dumbbells:
            object_start = objects[pos / 2][0]
            object_end = objects[pos / 2][1]
        else:
            object_start = objects[pos][0]
            object_end = objects[pos][1]

        if object_end < object_start:
            center_of_object = ranges.index(min(ranges[object_start:] + ranges[:object_end]))
        else:
            center_of_object = ranges.index(min(ranges[object_start:object_end]))
        if self.dumbbell_found:
            center_of_object += 179
        while center_of_object > 179:
            center_of_object -= 360
        print(center_of_object)
        self.target_turn(center_of_object)
        


    def move_arm(self):
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
        # wait=True ensures that the movement is synchronous
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

    def place_dumbbell(self):
        self.open_grip()
        # Lower the dumbbel to ground level
        arm_joint_goal = [0,.4,.5,-.9]
        # arm_joint_goal = [0,
        #     math.radians(0.0),
        #     math.radians(-40.0),
        #     math.radians(-20.0)]
        # wait=True ensures that the movement is synchronous
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()
        
        self.twist.linear.x = -0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)



    def image_callback(self, data):
        # keeps image updated for processing dumbbells/blocks
        self.image = data

    def process_dumbbells(self):
        # Assumes robot is at the original/reset position and returns relative
        # location of dumbbells as a dict
        # 'L', 'C', 'R' stands for left, center, and right respectively

        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        rospy.sleep(0.1)
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define upper & lower bounds for each color
        # found these values at 
        # https://pysource.com/2019/02/15/detecting-colors-hsv-color-space-opencv-with-python/
        # make mask and get moment of each color to find center
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        red_M = cv2.moments(red_mask)

        lower_blue = np.array([94, 80, 2])
        upper_blue = np.array([126, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_M = cv2.moments(blue_mask)

        lower_green = np.array([25, 52, 72])
        upper_green = np.array([102, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        green_M = cv2.moments(green_mask)

        # if all colors are found 
        if red_M['m00'] > 0 and blue_M['m00'] > 0 and green_M['m00'] > 0:
            # find pixel of x center of each mask
            cx_red = red_M['m10']/red_M['m00']
            cx_blue = blue_M['m10']/blue_M['m00']
            cx_green = green_M['m10']/green_M['m00']

            # make list of tuples containing location info
            loc_list = [('Red', cx_red), ('Blue', cx_blue), ('Green', cx_green)]

            # sort list by cx location
            loc_list.sort(key = lambda x: x[1])

            self.dumbbell_location[loc_list[0][0]] = 'L'
            self.dumbbell_location[loc_list[1][0]] = 'C'
            self.dumbbell_location[loc_list[2][0]] = 'R'

        else:
            print("Error dumbbells not found")

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
        # Assumes robot is at the original/reset position and returns relative
        # location of blocks as a dict
        # 'L', 'C', 'R' stands for left, center, and right respectively
        
        # point camera towards blocks
        rospy.sleep(0.1)

        # turn towards rightmost block
        self.target_turn(130)
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='rgb8')
        nums_detected = self.pipeline.recognize([image])[0]
        self.block_location[int(nums_detected[0][0])] = 'R'

        # turn towards center block
        self.target_turn(179)
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='rgb8')
        nums_detected = self.pipeline.recognize([image])[0]
        self.block_location[int(nums_detected[0][0])] = 'C'


        # turn towards left block
        self.target_turn(-130)
        image = self.bridge.imgmsg_to_cv2(self.image,desired_encoding='rgb8')
        nums_detected = self.pipeline.recognize([image])[0]
        self.block_location[int(nums_detected[0][0])] = 'L'

        self.target_turn(0)


    def robot_actions(self, data):
        pass

    def model_states_received(self, data):
        pass

    def processReward(self, data):
        pass

    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = Movement()
    node.run()
