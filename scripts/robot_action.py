import rospy
import numpy as np
from q_learning_project.msg import RobotMoveDBToBlock

class RobotAction(object):

    #TODO:
    #Can the robot capture all three dumbells/blocks in one camera frame?

    def __init__(self):
        rospy.init_node('robot_action')

        self.action_sub = rospy.Subscriber('/q_learning/robot_action',RobotMoveDBToBlock,self.action_received)
        self.block_ordering = {}
        self.get_dumbell_ordering = {}

    def action_received(self,data):
        pass

    def move_to_db(self):
        pass

    def move_to_block(self):
        pass

    def pick_up_db(self):
        pass
    
    def put_down_db(self):
        pass
    
    