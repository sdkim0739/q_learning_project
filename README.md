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

## Demonstration GIF
![GIF of Turtlebot picking up dumbbells and placing them in front of numbered blocks]()
*Note: in this GIF, the robot prematurely drops the red dumbbell, but it still executes the movement to "drop" it in front of the correct block.*

## Q-Learning algorithm

### Selecting / executing actions:
From the current state, we selected a random valid action. Invalidity was defined by the action being equivalent to -1 in the action matrix. Our function `get_random_action` filters for the possible valid actions from the current state, and then selects one at random. If all blocks are occupied by a dumbbell at the current state, the current state resets to 0 (the origin state) and executes a random valid action from there.

**Code location**: `get_random_action()` in `q_learning_training.py`

### Updating the Q-matrix: 
We update the Q-matrix in the callback function to the `/q_learning/reward` subscriber, `update_q_matrix()`. This function mainly makes use of the Q-Learning algorithm from class to calculate the new Q-value for the current state and action being performed. The change in Q-value is additionally appended to `q_history`, an array that keeps track of the last `n` meaningful changes. If the Q-matrix has not yet converged, we move to the new state and repeat the algorithm. The Q-matrix is stored as a `numpy` array during training, but it is converted into a `QMatrix()` message upon publishing. Once the algorithm converges, the Q-matrix is saved in a `.txt` file.

**Code location**: `update_q_matrix(), convert_send_qmatrix_msg()` in `q_learning_training.py`, `save_q_matrix` in `q_learning.py`

### Determining convergence of the Q-Learning algorithm:
Our standard for convergence was that the last 100 Q-value changes (in `q_history`) would all have to be less than `max_diff`, 0.01, and that there must be at least 10000 iterations. The numbers were chosen mainly through guess-and-check; initially, our numbers for the last `n` changes and the iterations were too small, which caused convergence far too early. `max_diff` was also decreased incrementally until we reached a satisfactory number in testing.

**Code location**: `has_converged()` in `q_learning_training.py`

### Executing the path of maximum reward:
We first load the trained Q Matrix into a 2D array. We then simulate state-action transition by chosing the action with the highest Q-value at each state. This gives the sequence of actions to execute. We execute each aciton in trasition order, but note that the order in which actions are executed does not matter. Actions are executed by publishing a `RobotMoveDBToBlock` message.
**Code location**: `extract_action()` in `robot_action.py`, `read_q_matrix()` in `q_learning.py`


## Robot perception

### Identifying locations & identities of the dumbbells: 
The robot spins around until the color of the desired dumbbell is in the center of the robot's camera frame. 

**Code location**: The `locate_dumbell ` method in `robot_action.py`

### Identifying locations & identities of the blocks:
The robot will repeatdely rotate 45 degrees and then send its camera footage to the `keras_ocr`. If the desired digit is detected, the robot will stop rotating and initiate the movement phase. Otherwise, the robot will continue to rotate and use the detector until the correct digit comes into view. We also account for common detector errors, such as identifying 1 as l.  

**Code location**: The `move_to_block` method in the `robot_action.py` file


## Robot manipulation and movement

### Moving to the right spot to pick up a dumbbell:
Once the correct dumbell is idenfitied, the robot uses proportional control to drive toward the dumbell. For angular velocity, the center of the color is computed as done in the line-follower demo. The pixel location of this center is used to compute the angular error. We also use proportional control in the linear direciton by looking at the scan topic to see the robot's distance to the object at zero degrees. To make sure this object is the correct dumbell, and not some other dumbell or block, we only update the linear velocity when the angular error is small enough. This ensures that the robot is properly alligned with the dumbell so it will be located at zero degrees. 

**Code location**: The `move_to_dumbell` method in `robot_action.py`

### Picking up the dumbbell:
When the world is initialized, the robot arm should already be in a position where the gripper is aligned with the dumbbell handle. To lift the dumbbell, the robot need only close the gripper around the handle, and then it adjusts a couple of its joints to raise the arm above the ground.

**Code location**: `lift_dumbbell()` in `robot_action.py`

### Moving to the desired block:
Once the desired block number is identified, the robot moves to the front using proportional control. The amount angular error is computed using the center of the bounding box returned by the detector. Often, the robot's camera will be in view of adjacent faces of the same block. This will return multiple bounding boxes for the same digit. To make sure the robot is moving to the front of the block, and not the side, we use the bounding box with higher area. 

**Code location**: The `move_to_block` method in the `robot_action.py` file

### Putting down the dumbbell:
To place the dumbbell back down, the robot arm joints move back to their starting position at world initialization. Since the robot arm is gripping the dumbbell throughout this process, the dumbbell should be touching the ground when it is back in the initial position. The robot then opens the gripper to let go of the dumbbell.

**Code location**: `drop_dumbbell()` in `robot_action.py`


## Challenges
Honestly, the whole project was a challenge! Each of the three main components were time-consuming and difficult in their own ways. Determining convergence for the Q-Learning algorithm was certainly a hurdle, because all we could do was continually tweak the parameters until we hit a set of them that allowed for convergence. Even then, it might still not as perfect as it could be, depending on if we're also using optimal learning rate and gamma values. The robot perception and movement was possibly even more complicated. We had a lot of trouble finding the correct color ranges for the dumbell. Ultimately what we did was place the robot in front of each dumbell in Gazebo and print the pixel values at the center of its camera. Also, proportional control is difficult to integrate with the task of recognition simultaneously. If the robot moves to far off course, the correct block may go out of view and the robot will not know how to navigate bac. The difficulties with the robot arm manipulation were similar to those of determining convergence. We had to find angles for the arm joints that allowed the gripper to align with the dumbbell without tipping over the robot (this happened frequently in testing with the GUI). Determining the gripper width was also a hassle, because it's difficult to tell when it's precisely too tight or too loose. Eventually, we just settled on a gripper width as long as the dumbbell didn't drop out or get squeezed out of the gripper.

## Future work
The Q-Learning convergence might still be less inaccurate than we would like. If we had more time, we would've played with the learning rate and set it equal to values != 1 to test how this affects convergence. I think it would also have been good for us to conference with one another and more precisely predict the Q-matrix should output; we didn't quite do this part synchronously, but rather looked at the Q-Learning code individually.

It would be good to experiemnt with better ways to integrate the `keras_ocr` detector with the robot's movements. The robot's movement is very slow when moving to the blocks because the detector is slow. It might be better to leverage the detector with the camera and scan topics so that the robot relies on the detector less. Doing this would speed up the movement. 

## Takeaways
1. Again, timing and proper pacing of the project is always a skill that can be worked on. For this project specifically, there were so many details and components that it was hard to tell which part would be the most time-consuming, if any. Thus, despite our initial timeline, it was hard for us to split up the project over the 2 weeks we had by component, since some ended up taking more time than others. I think in the future, as long as we make decent progress consistently on the project, it shouldn't need to matter that much which component of the project is getting completed on which day.
2. One thing that was very interesting about this project is the problem of integrating perception and motion. It is very difficult to do this effectively, especially without a map of the environment and information about the robot's exact location.






