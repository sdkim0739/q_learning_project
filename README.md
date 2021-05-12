# q_learning_project

## Project Partners

Stephanie Kim ([stephaniekim@uchicago.edu](mailto:stephaniekim@uchicago.edu))

Oscar Michel ([ojmichel@uchicago.edu](mailto:ojmichel@uchicago.edu))

## Implementation Plan:

## Q-Learning Algorithm

### Executing the Q-Learning algorithm

Simply put, we will implement the Q-Learning algorithm presented in class.

Until we write the robot motion code, we will use the phantom movement. That way, we can run the algorithm without moving robots. We’ll store a representation of the Q-matrix in the QLearning class.

**Testing**: We will print our Q-Matrix after each state-action update and check to see that the Q-values are properly updating according to the reward function.

### Determining when the Q-matrix has converged

Initially, we will take a rolling average of the differences between Q-values across iterations and compare them to a fixed constant that we will experimentally define. If this average is less than the constant, this is likely a sign that the Q-Matrix has converged.

We will also experiment with other methods of testing convergence, as the above method may not be the most efficient.

**Testing**: We will determine whether we have converged at the right iteration of the Q-Matrix by 1) making sure the robot can actually accomplish the task at the iteration we stop at, and 2) checking to see that the rolling average is less than the constant for a sufficient number of iterations.

### Once the Q-matrix has converged, determining which actions the robot should take to maximize expected reward

We will keep track of the current state and find the action corresponding to that state with the highest Q-value. This will be the next action and we’ll update the current state after taking that action.

**Testing**: We will make sure the robot can consistently reach the goal state.


## Robot perception

### Determining the identities and locations of the three colored dumbbells

Assuming the robot can capture all three dumbbells in a single camera frame, we will use the RGB camera and a vision recognition model to determine their relative ordering. 

**Testing**: See below

### Determining the identities and locations of the three numbered blocks

As suggested in class, we will search online for possible implementations of number/digit recognition by the robot and use them as our inspiration for our method of distinguishing between the identities of the numbered blocks.

To determine the location of the block, our initial idea is to use the LiDAR to locate one of the blocks, e.g., the rightmost block, and grab its general heading. The robot can then zoom in using its camera on this block and check to see if it is the numbered block we are targeting. If not, it will use the LiDAR to grab the location of another block and repeat this process.

**Testing**: For both the dumbbells and the blocks, we will check that the dumbbell / block the robot ends up moving towards is actually the one that it perceived.

## Robot manipulation and movement

### Picking up and putting down the dumbbells with the OpenMANIPULATOR arm

We plan to start with two main functions that code the joint movements to center the grabber on the dumbbells, grab the dumbbell, and lift it vertically up/down.

**Testing**: We will check that the robot accurately grips the dumbbell and lifts it vertically.

### Navigating to the appropriate locations to pick up and put down the dumbbells

To pick up, first we will use the odometry to return the origin point. Once there, we will pick up the dumbbell in the desired location. Then, we will use LIDAR and odometry to move the dumbbell to the correct block. We will then call a function specifically designed to put the dumbbell down at whatever location the robot is currently in. 

**Testing**: We will try our method on a variety of different dumbbell/block location combinations. 

## Timeline

**Q-Learning algorithm**: May 3

**Robot perception**: May 5

**Robot manipulation & movement**: May 11

## Objectives
Our main objective was to use reinforcement learning to train the Turtlebot to drop dumbbells in front of blocks in such a manner that maximizes reward. To do this, our task was to use the Q-Learning algorithm, robot perception, and robot manipulation and movement using the OpenMANIPULATOR arm.

## High-level description
Through keeping track of the last `n` meaningful changes in Q-value during training, we outputted a Q-Matrix whose actions at select states maximize reward, subsequently becoming the goal of the robot's perception and movement.

## Q-Learning algorithm

### Selecting / executing actions:

**Code location**:

### Updating the Q-matrix:

**Code location**:

### Determining convergence of the Q-Learning algorithm:

**Code location**:

### Executing the path of maximum reward:

**Code location**:


## Robot perception

### Identifying locations & identities of the dumbbells:

**Code location**:

### Identifying locations & identities of the blocks:

**Code location**:


## Robot manipulation and movement

### Moving to the right spot to pick up a dumbbell:

**Code location**:

### Picking up the dumbbell:

**Code location**:

### Moving to the desired block:

**Code location**:

### Putting down the dumbbell:

**Code location**:


## Challenges


## Future work


## Takeaways






