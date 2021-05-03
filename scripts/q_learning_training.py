#!/usr/bin/env python3

import rospy
import numpy as np
import random
from q_learning import QLearning
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveDBToBlock

class QLearningTraining(object):
    # Initialize publishers and subscribers
    def __init__(self):
        # Subscribe to environment to receive reward updates
        self.reward = rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q_matrix)
        
        # Publish Q-matrix updates to Q-matrix topic
        self.q_matrix_pub = rospy.Publisher("/q_learning/QMatrix", QMatrix, queue_size=10)

        # TESTING: Publish to /robot_action for phantom robot movement
        self.q_learning = QLearning()
        self.phantom_bot = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

        # Initialize Q-matrix
        self.q_matrix = QMatrix()

        # Initialize rows in Q-matrix
        for i in range(64):
            self.q_matrix.q_matrix.append(QMatrixRow()) # Test this again

        # For ease of use in training, we'll store the Q-matrix in a numpy array
        self.q_matrix_arr = np.zeros((64,9))
        
        # Keeps track of current (starting) state, initialized to 0th state
        self.current_state = 0
        
        # Get new state and action from starting state
        new_state, action = self.get_random_action(self.current_state)

        # Current action is action from above
        self.current_action = action

        # Keep track of new state
        self.new_state = new_state

        # Move the robot according to the current action
        self.phantom_robot_move_DB(action) # update_q_matrix() will be called as callback

        # Keep track of the 5 most recently updated Q-values
        self.q_history = []
        

    # Selects a random valid (non -1) action from the current state
    def get_random_action(self, state):
        # Stores all valid actions at the current state
        valid_actions = []

        # Search the actions at the current state for the valid ones
        for (s,a) in enumerate(self.q_learning.action_matrix[state]):
            if a != -1.0: # If valid, append to valid_actions
                valid_actions.append((s,a))

        # Select a random action among the valid actions
        indx = random.randint(0,len(valid_actions) - 1)
        new_state, action = valid_actions[indx]
        
        # Return that action and next state
        return new_state, action


    # TESTING: "Moves" the phantom robot to move dumbbells to blocks
    def phantom_robot_move_DB(self, action):
        self.test_movement = RobotMoveDBToBlock()

        # Take the first action
        current_action = self.q_learning.actions[action]

        # From the action, store the dumbbell being moved and the block to move to
        self.test_movement.robot_db = current_action['dumbbell']
        self.test_movement.block_id = current_action['block']

        # Publish the action to the phantom bot
        rospy.sleep(1)
        self.phantom_bot.publish(self.test_movement)

    def update_q_matrix(self, data):
        # Receive r_t (this is just data.reward)

        # Discount factor = 0.8
        gamma = 0.8

        # Update Q(s,a)
        Q_s_a = data.reward + gamma * max(self.q_matrix_arr[self.new_state])
        
        self.q_history.append(Q_s_a - self.q_matrix_arr[self.current_state,self.current_action])
        if len(self.q_history) > 5:
            self.q_history.pop(0)
        
        self.q_matrix_arr[self.current_state,self.current_action] = Q_s_a
        self.convert_send_qmatrix_msg()

        # If not converged:
        if not self.has_converged():
            # Select the next action
            new_state, action = self.get_random_action(self.new_state)

            # Update current state to new state
            self.current_state = self.new_state
            self.new_state = new_state
            self.action = action

            # Perform the next action
            self.phantom_robot_move_DB(action)

    def has_converged(self): # Fill in later -- testing required
        # TODO: May have to tweak 0.1
        return np.mean(self.q_history) < 0.1 and len(self.q_history) == 5
    
    def convert_send_qmatrix_msg(self): # Converts numpy array to QMatrix msg
        # TODO: maybe pointer issues?? prob not
        for i in range(len(self.q_matrix_arr)):
            row = list(self.q_matrix_arr[i])
            self.q_matrix.q_matrix[i] = row

        # Publish Q-matrix message to Q-matrix topic
        self.q_matrix_pub.publish(self.q_matrix)

    # Runs until shutdown
    def run(self):
        # TESTING: Move the dumbbells with the phantom bot
        rospy.spin()

# Runs file
if __name__ == "__main__":
    training_node = QLearningTraining()
    training_node.run()

