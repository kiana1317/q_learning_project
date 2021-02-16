# q_learning_project

Team Members: Kiana Hobbs & Hunter Thompson

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