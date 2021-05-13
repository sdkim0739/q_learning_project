#!/usr/bin/env python3


from q_learning_project.msg import RobotMoveDBToBlock
import rospy
import time

class ActionTest():

    def __init__(self):
        rospy.init_node('action_test')
        self.action_pub = rospy.Publisher('/q_learning/robot_action',RobotMoveDBToBlock,queue_size=1)
        rospy.sleep(2)
        self.test()
    
    def test(self):
        print('testing...')
        msg = RobotMoveDBToBlock()
        msg.robot_db = 'blue'
        msg.block_id = 2
        self.action_pub.publish(msg)

if __name__ == '__main__':
    ActionTest()
    


