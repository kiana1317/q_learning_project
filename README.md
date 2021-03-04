# Q Learning Project

## Team Members
Kiana Hobbs & Hunter Thompson

## Objectives Description 

The goal is this project is to program a robot with a mechanical arm and gripper to perceive and retrieve color coded dumbbells and place them near a specified numbered block. This project required the use of Q-Learning, perception, and robot manipulation. 

## High-level  Description

In the project, we used the Q-Learning algorithm to determine the optimal action sequence for moving the dumbbells. To do so, we compiled a matrix of all possible steps that the robot could take to place the dumbbells and iterated through those possibilities to determine which sequence yielded a reward. All other actions beyond the optimal one yielded no reward, which allowed our matrix to converge to the correct action sequence. By employing the technique of rewarding for reinforcement learning, we were able to determine which dumbbells belong in front of each numbered block.

## Q-learning algorithm description: 
Describe how you accomplished each of the following components of the Q-learning algorithm in 1-3 sentences, and also describe what functions / sections of the code executed each of these components(1-3 sentences per function / portion of code):
* Selecting and executing actions for the robot (or phantom robot) to take
* Updating the Q-matrix
* Determining when to stop iterating through the Q-learning algorithm
* Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot

## Robot perception description
Describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):

*Identifying the locations and identities of each of the colored dumbbells*

For our implementation, we used color perception to find the dumbbell necessary for the current action. To do so, we first set the upper and lower color spectrum ranges based off which color we were pursuing and generated a pixel mask for that color. Then, we rotated the robot in a constant z-angular shift until that color came into the robot’s line of sight. 

```process_dumbbells()```: Implements the process of rotating the robot until the dumbbell is spotted using the color recognition methods from the Line Follower exercise. After the robot spots the dumbbell, the program implements PID to center the robot on the dumbbell as it moves to pickup it.

*Identifying the locations and identities of each of the numbered blocks*

After the robot retrieves the dumbbell, it begins scanning at first for the color black with color perception to signify the presence of a numbered block, and then it uses the Keras number recognition to check if the block matches the block in the action sequence. These two forms of perception were used together to reduce the need for calling the Keras image recognition process, given it produced delayed results.

```process_blocks():``` Implements the process for rotating the robot until it perceives first a number with color perception and then if it is the correct number with image perception. The robot stops after finding a block to allow time for the image to completely process before proceeding. After finding and confirming the block, the program implements PID with perceiving black to center the robot on the block.

## Robot manipulation and movement

*Moving to the right spot in order to pick up a dumbbell*

To navigate the robot to the right spot, we used color perception to scan for the desired dumbbell color. Once the dumbbell was identified, we used the logic from the Line Follower code to keep the robot centered as it approached the dumbbell. This functionality was implemented in the ```process_dumbells():``` function mentioned in perception.

*Picking up the dumbbell*

To pick up the dumbbell, we experimented using the arm UI to see which arm positioning works best for both gripping and lifting the dumbbell. For gripping, we had to ensure that the robot approached the dumbbell centered, given the gripper has a limited extension width. Moreover, for the lifting, we had to guarantee that the end positioning allowed the robot to maneuver without dropping the dumbbell. 

```lift_dumbbell()```:  Function that implements the lift dumbbell mechanic mentioned above. This functionality was called in the ```process_dumbells():``` function mentioned in perception.

*Moving to the desired destination (numbered block) with the dumbbell*

To move to the desired numbered block, we had to employ a combination of image and color perception. While rotating, we used color perception for perceiving black to determine if the robot was facing a block. Then, we used image perception to determine which number it was facing. Once the block was confirmed, we utilized color perception again to keep the robot centered while approaching the block. This functionality was implemented in the ```process_blocks():``` function mentioned in perception.

*Putting the dumbbell back down at the desired destination*

Once the robot was in a short distance of the block, we opened the gripper and then lowered the arm to allow the block to fall to the ground with control.  

```place_dumbbell():``` Function that implements the place dumbbell mechanic mentioned above. This functionality was called in the ```process_blocks():``` function mentioned in perception.

#### Functions
Noted generally here because they were used for more than one action related to robot manipulation and movement.

```complete_action()```: Iterates through the action sequence for each dumbbell.
    
```processScan()```: Populates the scan data ranges for each function to use.

```reset()```: Resets the variables, such as Booleans and counters, after each iterations.

```starting_arm_position()```: Positions the arm in the starting positions to be best able to grip the dumbbells.

```close_grip()```: Closes the grip slightly to prevent the dumbbell from moving around while the robot is moving. For this function we had to experiment with the manipulator UI to find a balance to prevent the gripper from closing too much to push out the dumbbell.

```open_grip()```: Opens the grip to place the dumbbell once it reaches the block.

```image_callback()```: Stores the image information processed by the robot.

## Challenges 
(1 paragraph) Describe the challenges you faced and how you overcame them.

Before using perception, we tried to employ various techniques for finding the center of the blocks, such as scanning for the width of the object and finding the center based off that. However, while that did work for centering on the dumbbells, it proved difficult for approaching the block when we wanted to place the dumbbell at the face of the block. Another challenge was dealing with the tediousness of having to wait until the robot complemented an action sequence to see whether it encountered an error or incorrectly performed. To address this, we used the arm GUI to practice positioning before running the robot script, and we also tried to be more meticulous in our code to limit easy-to-spot bugs occurring during runtime.

## Future work 
(1 paragraph): If you had more time, how would you improve your implementation?

## Takeaways 
(at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

* Think more in-depth about how sections intersect - We initially divided the project with perception and movement being two distinct sections. As such one partner implemented perception and the other movement. However, after many failed trials at getting them to work disjointly (first perceive all objects, then move based off where you know the objects to be), we realized that to facilitate the best mechanics for robot movement, we had to employ perception during rather than before movement.

## Gif

![Motion and navigate to goal](robomovement.gif)

# Implementation Plan

## Q-Learning Algorithm
- We will create an action matrix to select a_t at random and give data to
  movement algorithm. Then, we update the Q-matrix according to the reward
  and check for convergence of the matrix. 

- We will run our program to figure out what it returns as the max reward
  state including all dumbbells. Then, we will go through each of the 64 
  states and find the state that maximizes rewards and see whether they
  correspond. 

## Robot Perception
- For the color perception, we will use a similar algorithm as from class 
  meeting 03 where we set a color range of mask and return the ordering of
  the dumbbells so that the robot may move. Similarly for block perception, we
  will might set a mask to detect each number or follow the algorithm presented
  in class meeting 11 for robotic vision. 

- We will use visual tests to check whether the robot returns the correct 
  location for each dumbbell/block. 

## Robot Manipulation & Movement
- We will split the movement & manipulation into smaller functions to perform
  the full task, including move to dumbbell, manipulate the arm and lifting 
  dumbbell, move to block, set down block, and reset robot position to inital
  state. 

- We will use visual tests to check whether the robot moves to the correct
  locations and that it interacts porperly with dumbbells. 

## Timeline
- Rough Draft of all components by Monday Feb. 22nd
- Final Draft by Friday Feb. 26th
