import rospy
import numpy as np
import cv2
import cv_bridge
from q_learning_project.msg import RobotMoveDBToBlock
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import keras_ocr

pipeline = keras_ocr.pipeline.Pipeline()


class RobotAction(object):

    #TODO:
    #Can the robot capture all three dumbells/blocks in one camera frame?

    def __init__(self):
        rospy.init_node('robot_action')

        self.action_sub = rospy.Subscriber('/q_learning/robot_action',RobotMoveDBToBlock,self.action_received)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.camera_received)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_received)
        self.vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)

      
        self.current_scan = []
        self.current_img = []

        self.bridge = cv_bridge.CvBridge()

        self.color_info = { #HSV upper and lower bounds for each color (need to test these ranges work)
            'red':{'lower':np.array([155,25,0]),'upper':np.array([179,255,255])},
            'blue':{'lower':np.array([110,50,50]),'upper':np.array([130,255,255])},
            'green':{'lower':np.array([45,100,50]),'upper':np.array([75,255,255])}
        }

    def action_received(self,data):

        color = data.robot_db

        #first scan around for dumbell
        self.locate_dumbell(color)
        #move to dumbell
        self.move_to_dumbell(color)
        #pickup dumbell

        #scan for block
        #move to block
        #put dumbell down


    def camera_received(self,data):
        
        image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
        image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) #convert to HSV
        self.current_img = image

    def locate_dumbell(self,color): #spin around until dumbell color is in sight
        found = False
        msg = Twist()
        msg.angular.z = np.pi / 12.0 #turn until color appears in camera
        upper, lower = self.color_info[color]['upper'], self.color_info[color]['lower']

        while not found:
            
            mask = cv2.inRange(self.current_img,lower,upper) #code from line follower to detect color
            h, w, d = self.current_img.shape
            search_top = int(3*h/4)
            search_bot = int(3*h/4 + 20)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)

            if M['m00'] > 0:
                found = True
            else:
                self.vel_pub.publish(msg)
            
    def move_to_dumbell(self,color):

        upper, lower = self.color_info[color]['upper'], self.color_info[color]['lower']
        stop_dist = 0.2 #TODO: figure out how close to get to dumbell
        front_dist = self.current_scan.ranges[0]
        mask = cv2.inRange(self.current_img,lower,upper) #code from line follower to detect color
        h, w, d = self.current_img.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        msg = Twist()
        while front_dist > stop_dist:
            front_dist = self.current_scan.ranges[0]
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                #proportional control
                err_ang = w/2 - cx
                k_ang = 1.0 / 100.0
                k_lin = 0.5

                err_lin = max(0,front_dist - stop_dist)
                msg.linear.x = 0.5*err_lin
                msg.angular.z = k_p * err_ang
                self.vel_pub.publish(msg)
    
    def detect_block(self):
        pass
            




    def scan_received(self,data):
        self.current_scan = data
    
    