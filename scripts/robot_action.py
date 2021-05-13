#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import cv_bridge
import moveit_commander
import math
from q_learning_project.msg import RobotMoveDBToBlock
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import keras_ocr
from q_learning import QLearning

pipeline = keras_ocr.pipeline.Pipeline()


class RobotAction(object):

    def __init__(self):
    
        # rospy.init_node('robot_action')
        rospy.on_shutdown(self.shutdown)
        self.bridge = cv_bridge.CvBridge()
        self.action_sub = rospy.Subscriber('/q_learning/robot_action',RobotMoveDBToBlock,self.action_received)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.camera_received)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_received)
        self.vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.action_pub = rospy.Publisher('/q_learning/robot_action',RobotMoveDBToBlock,queue_size=1)

        self.pipeline = keras_ocr.pipeline.Pipeline()
        self.send_next = True
      
        self.current_scan = []
        self.current_img = []
        self.q_learning = QLearning()

        
        rospy.sleep(2)

      
        #60 255 178
        self.color_info = { #HSV upper and lower bounds for each color (need to test these ranges work)
            'red':{'lower':np.array([0,250,150]),'upper':np.array([10,255,180])},
            'blue':{'lower':np.array([110,250,150]),'upper':np.array([130,255,190])},
            'green':{'lower':np.array([45,250,150]),'upper':np.array([75,255,190])}
        }

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set initial arm position
        self.move_group_arm.go([0.0, 0.9, -0.2, -0.79], wait=True)
        self.move_group_gripper.go([0.009, 0.009], wait=True)
        print("ready")
        # self.extract_action()
    
    def run(self):
        rospy.spin()
    
    def shutdown(self):
        self.vel_pub.publish(Twist())

    def extract_action(self):
        # Extract 3 actions from the Q-matrix

        # Publish next action if there are actions left to execute
        
        actions = []
        q_matrix_arr = self.q_learning.read_q_matrix()
        print(q_matrix_arr)
        states = self.q_learning.states
        state = 0

        #We simulate the state action transition to get the action sequences
        for i in range(3):
            max_reward = -1
            best_action = 0

            for (action, reward) in enumerate(q_matrix_arr[state]): 
                #chose the action with the highest reward for a given state
                # print(action,reward)
                if reward > max_reward:
                    max_reward = reward
                    best_action = action

            # print(max_reward)
            actions.append(best_action)
            next_state = 0
            # print(best_action)
            #get the next state from the action
            for (s,a) in enumerate(self.q_learning.action_matrix[state]):
                # print(type(a),type(action))
                if int(a) == best_action:
                    next_state = s
                    break
            state = next_state
        # print(actions)
        action_msgs = []
        for a in actions:
            #construct action message
            msg = RobotMoveDBToBlock()
            msg.block_id = self.q_learning.actions[a]['block']
            msg.robot_db = self.q_learning.actions[a]['dumbbell']
            action_msgs.append(msg)
        print(action_msgs)
        
        while action_msgs:
            #send action messages
            if self.send_next:
                msg = action_msgs.pop(0)
                print(msg)
                self.action_pub.publish(msg)
                self.send_next = False

    def action_received(self,data):
        print("action received")
        color = data.robot_db
        block = data.block_id

        #first scan around for dumbell
        self.locate_dumbell(color)
        #move to dumbell
        self.move_to_dumbell(color)
        #pickup dumbell
        self.lift_dumbbell()

        self.move_to_block(block)

        #put dumbell down
        self.drop_dumbbell()

        self.send_next = True


    def camera_received(self,data):
        image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
        image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) #convert to HSV

        # w,h,c = image.shape
        # print(image[w//3:2*w//3,h//3:2*h//3,:])
        
        self.current_img = image

    def locate_dumbell(self,color): #spin around until dumbell color is in sight
        found = False
        msg = Twist()
        msg.angular.z = np.pi / 12.0 #turn until color appears in camera
        upper, lower = self.color_info[color]['upper'], self.color_info[color]['lower']
        while not found:
            mask = cv2.inRange(self.current_img,lower,upper) #code from line follower to detect color
            w,h = mask.shape
            mask = mask[w//3:2*w//3,h//3:2*h//3]
            # print(np.sum(mask),np.sum(self.current_img))
            if np.sum(mask) > 0: #if this is true, then color detected
                found = True
                self.vel_pub.publish(Twist()) 
              
            else:
                self.vel_pub.publish(msg)
            
    def move_to_dumbell(self,color):

        upper, lower = self.color_info[color]['upper'], self.color_info[color]['lower']
        front_dist = np.inf
        stop_dist = 0.20
        
        while front_dist > stop_dist: #TODO: stop when close enough to dumbell
            
            mask = cv2.inRange(self.current_img, lower, upper)
            twist = Twist()
            M = cv2.moments(mask)
            h,w,d = self.current_img.shape
            img = cv2.cvtColor(self.current_img,cv2.COLOR_HSV2BGR)

            if M['m00'] > 0:

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # print(cx,cy)
            
                err = w/2 - cx
                if err < 100:
                    front_dist = self.current_scan.ranges[0] #only adjust linear when aligned to dumbell
                k_p = 1.0 / 100.0
                k_lin = 1.0
                twist.linear.x = k_lin * min(0.5,max(0,front_dist - stop_dist))
                twist.angular.z = k_p * err
                self.vel_pub.publish(twist)
                
        
    def move_to_block(self,block):

        dig_map = { #commonly returned labels for digits
            1: {'1','i','l'},
            2: {'2','s'},
            3: {'3','t','5'}
        }
        found = False #first turn until facing the correct block
        msg = Twist()
        msg.angular.z = -1.0 *np.pi / 8.0 #turn until color appears in camera
        while not found:
            pred = self.pipeline.recognize([self.current_img])[0] #get object predictions
            for (label,bb) in pred:
                print(label,bb)
                if label in dig_map[block]: #digit dectected 
                    found = True
            if not found: #if not found, turn robot to get new view
                r = rospy.Rate(2)
                for _ in range(4):
                    self.vel_pub.publish(msg)
                    r.sleep()
                self.vel_pub.publish(Twist())
        self.vel_pub.publish(Twist())

        front_dist = np.inf
        stop_dist = 0.35
        
        while front_dist > stop_dist: #now that block is in frame, move to block
            
            msg = Twist()
           
            h,w,d = self.current_img.shape

            correct_box_idx = 0
            max_area = 0
            pred = self.pipeline.recognize([self.current_img])[0]
            if pred:
                for i in range(len(pred)):
                    label, bb = pred[i][0], pred[i][1]
                    if label in dig_map[block]:
                        area = (bb[0][0] - bb[2][0])*(bb[1][1] - bb[3][1])
                        if area > max_area: #make sure we're looking at the right face (will have largest area)
                            max_area = area
                            correct_box_idx = i
                bbox = pred[correct_box_idx][1]
                cx = np.mean(bbox[:,0]) #this will get center of bounding box

                # print(bbox, pred[correct_box_idx][0],cx,correct_box_idx)
            
                err = w/2 - cx #offset from bounding box center
                if err < 100:
                    front_dist = self.current_scan.ranges[0]
                
                k_p = 1.0 / 400.0
                k_lin = 0.5
                msg.linear.x = k_lin * min(0.5,max(0,front_dist - stop_dist)) #proportional control
                msg.angular.z = k_p * err
                r = rospy.Rate(2)
                for _ in range(2):
                    self.vel_pub.publish(msg)
                    r.sleep()
                self.vel_pub.publish(Twist())
            else:
                break
                     
        
    def lift_dumbbell(self):
        print("lifting dumbbell")
        # Close gripper around dumbbell
        gripper_joint_goal = [0.006, 0.006]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        
        # Lift arm up
        arm_joint_goal = [0.0, 0.05, -0.15, -0.79]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

    def drop_dumbbell(self):
        print("dropping dumbbell")
        # Drop arm down
        arm_joint_goal = [0.0, 0.9, -0.2, -0.79]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

        # Open gripper around dumbbell
        gripper_joint_goal = [0.009, 0.009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        # Move backwards away from the dumbbell so robot can rotate
        msg = Twist()
        msg.linear.x = -1
        self.vel_pub.publish(msg)
        rospy.sleep(1)
        self.vel_pub.publish(Twist())

    def scan_received(self,data):
        self.current_scan = data
    
if __name__ == '__main__':
    r = RobotAction()
    r.run()