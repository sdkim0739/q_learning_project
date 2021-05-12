#!/usr/bin/env python3

import rospy
import numpy as np
import random
from q_learning import QLearning
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveDBToBlock

class QLearningTraining(object):
    # Initialize publishers and subscribers
    def __init__(self):
        # Publish Q-matrix updates to Q-matrix topic
        self.q_matrix_pub = rospy.Publisher("/q_learning/QMatrix", QMatrix, queue_size=10)

        # Publish to /robot_action for phantom robot movement
        self.q_learning = QLearning()
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

        # Subscribe to environment to receive reward updates
        self.reward = rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q_matrix)
        rospy.sleep(1)

        # Initialize Q-matrix
        self.q_matrix = QMatrix()

        # For ease of use in training, we'll store the Q-matrix in a numpy array
        self.q_matrix_arr = np.zeros((64,9))

        # Keep track of the 5 most recently updated Q-values
        self.q_history = []
        
        # Keeps track of current (starting) state, initialized to 0th state
        self.current_state = 0
        
        # Get new state and action from starting state
        new_state, action = self.get_random_action(self.current_state)

        # Current action is action from above (action from current state to new state)
        self.current_action = action

        # Keep track of new state
        self.new_state = new_state

        # Move the robot according to the current action
        self.move_DB(self.current_action) # update_q_matrix() will be called as callback
        

    # Selects a random valid action from the current state
    def get_random_action(self, state):
        # Stores all valid actions at the current state
        valid_actions = []

        # Search each next possible state from the current state
        for (s, a) in enumerate(self.q_learning.action_matrix[state]):
            # If valid, append to valid_actions
            if int(a) != -1:
                valid_actions.append((s, int(a)))

        # Select a random action among the valid actions
        if len(valid_actions) > 0:
            (new_state, action) = random.choice(valid_actions)
            print("Valid action {1} exists to state {0}".format(new_state, action))
            return (new_state, action)
        else: # Otherwise, reset to the initial state
            self.current_state = 0
            s_a_from_origin = self.get_random_action(0)
            print("No valid actions exist. Taking action {} from state 0".format(s_a_from_origin[1]))
            return s_a_from_origin


    # TESTING: "Moves" the phantom robot to move dumbbells to blocks
    def move_DB(self, action):
        self.test_movement = RobotMoveDBToBlock()

        # Take the first action
        current_action = self.q_learning.actions[action]

        # From the action, store the dumbbell being moved and the block to move to
        self.test_movement.robot_db = current_action['dumbbell']
        self.test_movement.block_id = current_action['block']

        # Publish the action to the phantom bot
        self.action_pub.publish(self.test_movement)
        print(self.test_movement)

    def update_q_matrix(self, data):
        # data.reward receives the reward
        print("Reward: {}".format(data.reward))
        print("Iteration: {}".format(data.iteration_num))
        # Discount factor
        gamma = 0.9

        # Update Q(s,a)
        Q_s_a = data.reward + gamma * max(self.q_matrix_arr[self.new_state])
        old_q_value = self.q_matrix_arr[self.current_state, self.current_action]

        # Append the change in Q-value to q_history to see whether Q-value changes are plateauing
        self.q_history.append(Q_s_a - old_q_value)
        print("Q-value diff: {}".format(Q_s_a - old_q_value))
        
        # Update the Q-matrix in the current spot with the new Q-value
        self.q_matrix_arr[self.current_state, self.current_action] = Q_s_a
        print("Q-matrix entry: {}".format(self.q_matrix_arr[self.current_state, self.current_action]))
        
        # We need to convert the numpy Q-matrix used for testing back into a QMatrix() message type
        self.convert_send_qmatrix_msg()

        # If not converged:
        if not self.has_converged():
            # Update the current state
            self.current_state = self.new_state

            # Select the next action
            new_state, action = self.get_random_action(self.current_state)

            # Update the new state
            self.new_state = new_state
            self.current_action = action

            # Perform the next action
            self.move_DB(self.current_action)
        else:
            print(self.q_matrix_arr)
            return

    # Determines when the Q-matrix has converged
    def has_converged(self):
        # Establish a plateau threshold, i.e. a constant below which the q-value differences should fall in order to
        # qualify the changes as plateauing
        max_diff = 0.01

        # Algo has converged if 
        # 1) the last 100 q-value differences are below the plateau threshold
        # 2) we have iterated at least 10000 times 
        # TODO: debugging
        return len(self.q_history) > 10000 and max(self.q_history[-100:]) < max_diff
    
    def convert_send_qmatrix_msg(self): # Converts numpy array to QMatrix msg
        for i in range(len(self.q_matrix_arr)):
            row = list(self.q_matrix_arr[i])
            self.q_matrix.q_matrix.append(row)

        # Publish Q-matrix message to Q-matrix topic
        self.q_matrix_pub.publish(self.q_matrix)

    # Runs until shutdown
    def run(self):
        rospy.spin()

# Runs file
if __name__ == "__main__":
    training_node = QLearningTraining()
    training_node.run()

