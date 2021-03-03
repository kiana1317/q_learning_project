# Q Learning Project

## Team Members
Kiana Hobbs & Hunter Thompson

## Objectives Description 
In the project, we used the Q-Learning algorithm to determine the optimal action sequence for moving the dumbbells. To do so, we compiled a matrix of all possible steps that the robot could take to place the dumbbells and iterated through those possibilities to determine which sequence yielded a reward. All other actions beyond the optimal one yielded no reward, which allowed our matrix to converge to the correct action sequence. By employing the technique of rewarding for reinforcement learning, we were able to determine which dumbbells belong in front of each numbered block.

## High-level  Description
(1 paragraph): At a high-level, describe how you used reinforcement learning to solve the task of determining which dumbbells belong in front of each numbered block.

## Q-learning algorithm description: 
Describe how you accomplished each of the following components of the Q-learning algorithm in 1-3 sentences, and also describe what functions / sections of the code executed each of these components(1-3 sentences per function / portion of code):
* Selecting and executing actions for the robot (or phantom robot) to take
* Updating the Q-matrix
* Determining when to stop iterating through the Q-learning algorithm
* Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot

## Robot perception description
Describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
* Identifying the locations and identities of each of the colored dumbbells
* Identifying the locations and identities of each of the numbered blocks
* 
# Robot manipulation and movement
Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
*Moving to the right spot in order to pick up a dumbbell:
To navigate the robot to the right spot, we used color perception to scan for the desired dumbbell color. Once the dumbbell was identified, we used the logic from the Line Follower code to keep the robot centered as it approached the dumbbell.
* Picking up the dumbbell
* Moving to the desired destination (numbered block) with the dumbbell
* Putting the dumbbell back down at the desired destination

# Challenges 
(1 paragraph) Describe the challenges you faced and how you overcame them.

# Future work 
(1 paragraph): If you had more time, how would you improve your implementation?
# Takeaways 
(at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

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
