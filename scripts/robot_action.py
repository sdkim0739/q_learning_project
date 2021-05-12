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

pipeline = keras_ocr.pipeline.Pipeline()


class RobotAction(object):

    def __init__(self):
        rospy.init_node('robot_action')
        rospy.on_shutdown(self.shutdown)
        self.bridge = cv_bridge.CvBridge()
        self.action_sub = rospy.Subscriber('/q_learning/robot_action',RobotMoveDBToBlock,self.action_received)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.camera_received)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_received)
        self.vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)

        self.pipeline = keras_ocr.pipeline.Pipeline()
        
      
        self.current_scan = []
        self.current_img = []

        
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
        self.move_group_arm.go([0.0, 0.65, 0.05, -0.79], wait=True)
        self.move_group_gripper.go([0.009, 0.009], wait=True)
        print("ready")
    
    def run(self):
        rospy.spin()
    
    def shutdown(self):
        self.vel_pub.publish(Twist())

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
            if np.sum(mask) > 0:
                found = True
                self.vel_pub.publish(Twist())
              
            else:
                self.vel_pub.publish(msg)
            
    def move_to_dumbell(self,color):

        upper, lower = self.color_info[color]['upper'], self.color_info[color]['lower']
        front_dist = np.inf
        stop_dist = 0.2
        
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

                # visualize a red circle in our debugging window to indicate
                # the center point of the yellow pixels
                # cv2.circle(img, (cx, cy), 20, (0,0,255), -1)

            
                err = w/2 - cx
                if err < 100:
                    front_dist = self.current_scan.ranges[0]
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
            pred = self.pipeline.recognize([self.current_img])[0]
            for (label,bb) in pred:
                print(label,bb)
                if label in dig_map[block]:
                    found = True
            if not found:
                r = rospy.Rate(2)
                for _ in range(4):
                    self.vel_pub.publish(msg)
                    r.sleep()
                self.vel_pub.publish(Twist())
        self.vel_pub.publish(Twist())

        front_dist = np.inf
        stop_dist = 0.2
        
        while front_dist > stop_dist: #now that block is in frame, move to block
            
            msg = Twist()
           
            h,w,d = self.current_img.shape

            correct_box_idx = 0
            max_area = 0
            pred = self.pipeline.recognize([self.current_img])[0]
            for i in range(len(pred)):
                label, bb = pred[i][0], pred[i][1]
                if label in dig_map[block]:
                    area = (bb[0][0] - bb[2][0])*(bb[1][1] - bb[3][1])
                    if area > max_area: #make sure we're looking at the right face (will have largest area)
                        max_area = area
                        correct_box_idx = i
            bbox = pred[correct_box_idx][1]
            cx = np.mean(bbox[:,0])

            # print(bbox, pred[correct_box_idx][0],cx,correct_box_idx)
        
            err = w/2 - cx
            if err < 100:
                front_dist = self.current_scan.ranges[0]
            k_p = 1.0 / 400.0
            k_lin = 0.25
            msg.linear.x = k_lin * min(0.5,max(0,front_dist - stop_dist))
            msg.angular.z = k_p * err
            r = rospy.Rate(2)
            for _ in range(2):
                self.vel_pub.publish(msg)
                r.sleep()
            self.vel_pub.publish(Twist())         
        
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
        arm_joint_goal = [0.0, 0.65, 0.05, -0.79]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

        # Open gripper around dumbbell
        gripper_joint_goal = [0.009, 0.009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def scan_received(self,data):
        self.current_scan = data
    
if __name__ == '__main__':
    r = RobotAction()
    r.run()