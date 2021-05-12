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
From the current state, we selected a random valid action. Invalidity was defined by the action being equivalent to -1 in the action matrix. Our function `get_random_action` filters for the possible valid actions from the current state, and then selects one at random. If all blocks are occupied by a dumbbell at the current state, the current state resets to 0 (the origin state) and executes a random valid action from there.

**Code location**:

### Updating the Q-matrix: 
We update the Q-matrix in the callback function to the `/q_learning/reward` subscriber, `update_q_matrix()`. This function mainly makes use of the Q-Learning algorithm from class to calculate the new Q-value for the current state and action being performed. The change in Q-value is additionally appended to `q_history`, an array that keeps track of the last `n` meaningful changes. If the Q-matrix has not yet converged, we move to the new state and repeat the algorithm.

**Code location**: **TODO: Steph**

### Determining convergence of the Q-Learning algorithm:
Our standard for convergence was that the last 100 Q-value changes (in `q_history`) would all have to be less than `max_diff`, 0.01, and that there must be at least 10000 iterations. The numbers were chosen mainly through guess-and-check; initially, our numbers for the last `n` changes and the iterations were too small, which caused convergence far too early. `max_diff` was also decreased incrementally until we reached a satisfactory number in testing.

**Code location**: **TODO: Steph**

### Executing the path of maximum reward:
**TODO: Steph or anyone**

**Code location**: **TODO: Steph or anyone**


## Robot perception

### Identifying locations & identities of the dumbbells: 
**TODO: Oscar**

**Code location**: **TODO: Oscar**

### Identifying locations & identities of the blocks:
**TODO: Oscar**

**Code location**: **TODO: Oscar**


## Robot manipulation and movement

### Moving to the right spot to pick up a dumbbell:
**TODO: Oscar**

**Code location**: **TODO: Oscar**

### Picking up the dumbbell:
When the world is initialized, the robot arm should already be in a position where the gripper is aligned with the dumbbell handle. To lift the dumbbell, the robot need only close the gripper around the handle, and then it adjusts a couple of its joints to raise the arm above the ground.

**Code location**: **TODO: Steph**

### Moving to the desired block:
**TODO: Oscar**

**Code location**: **TODO: Oscar**

### Putting down the dumbbell:
To place the dumbbell back down, the robot arm joints move back to their starting position at world initialization. Since the robot arm is gripping the dumbbell throughout this process, the dumbbell should be touching the ground when it is back in the initial position. The robot then opens the gripper to let go of the dumbbell.

**Code location**: **TODO: Steph**


## Challenges
Honestly, the whole project was a challenge! Each of the three main components were time-consuming and difficult in their own ways. Determining convergence for the Q-Learning algorithm was certainly a hurdle, because all we could do was continually tweak the parameters until we hit a set of them that allowed for convergence. Even then, it might still not as perfect as it could be, depending on if we're also using optimal learning rate and gamma values. The robot perception and movement was possibly even more complicated. **TODO: Oscar?** The difficulties with the robot arm manipulation were similar to those of determining convergence. We had to find angles for the arm joints that allowed the gripper to align with the dumbbell without tipping over the robot (this happened frequently in testing with the GUI). Determining the gripper width was also a hassle, because it's difficult to tell when it's precisely too tight or too loose. Eventually, we just settled on a gripper width as long as the dumbbell didn't drop out or get squeezed out of the gripper.

## Future work
The Q-Learning convergence might still be less inaccurate than we would like. If we had more time, we would've played with the learning rate and set it equal to values != 1 to test how this affects convergence. I think it would also have been good for us to conference with one another and more precisely predict the Q-matrix should output; we didn't quite do this part synchronously, but rather looked at the Q-Learning code individually.
**TODO: anyone add more**

## Takeaways
1. Again, timing and proper pacing of the project is always a skill that can be worked on. For this project specifically, there were so many details and components that it was hard to tell which part would be the most time-consuming, if any. Thus, despite our initial timeline, it was hard for us to split up the project over the 2 weeks we had by component, since some ended up taking more time than others. I think in the future, as long as we make decent progress consistently on the project, it shouldn't need to matter that much which component of the project is getting completed on which day.
2. **TODO: anyone**






