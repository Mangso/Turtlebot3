#!/usr/bin/env python
# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함

import cv2
import numpy as np
import sys
import math
import rospy
import time
import os

from std_msgs.msg import Float64, UInt8, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf

from S_TrafficLight import Light
from S_LineDetector import Line
from ObstacleDetector import Obstacle
#from TunnelDetector import Tunnel
from StopbarDetector import Stopbar

class AutoDrive:
    
    def __init__(self):

        self.mode = 4

        self.left_lane = False
        self.right_lane = False
        self.center = 0
        self.lastError = 0
        self.MAX_VEL = 0.2

        self.cam_img = np.zeros(shape=(360, 640, 3), dtype=np.uint8)
        self.cam_img2 = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.go = False

        # Construction
        self.phase = 0
        self.status = 0
        self.check =0

        # Parking
        self.park = 0
        self.park_set= 0
        self.zero = 0
        # Stop
        self.Chadan = 0
        self.Chadan_go = False
        

        #
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.mode_msg = UInt8()
        
        #self.line = Line('/usb_cam/image_raw/compressed')
        self.line = Line('/camera1/usb_cam1/image_raw/compressed')
        self.obstacle = Obstacle('/scan')
        #self.tunnel = Tunnel('/scan')

        self.sub_img1 = rospy.Subscriber('/camera1/usb_cam1/image_raw/compressed', CompressedImage,
                                     self.callback1, queue_size=1)
        self.sub_img2= rospy.Subscriber('/camera2/usb_cam2/image_raw/compressed', CompressedImage,
                                     self.callback2, queue_size=1)
        self.sub_detect_sign = rospy.Subscriber('/detect/traffic_sign', UInt8, self.mode_selector, queue_size=1)
        

    def PD_control(self, kp =0.012,kd = 0.004):
        error = self.center - 300
        print('error : ', error)
        angular_z = kp * error + kd * (error - self.lastError)
        self.lastError = error
        speed = min(self.MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.2)
        angular = -max(angular_z, -1.6 if angular_z < 0 else -min(angular_z, 1.6))
        return speed, angular

    def callback1(self,img_data):
        self.cam_img = self.bridge.compressed_imgmsg_to_cv2(img_data, "bgr8")

    def callback2(self,img_data):
        self.cam_img2 = self.bridge.compressed_imgmsg_to_cv2(img_data,"bgr8")

    def mode_selector(self, mode_msg):
        # 1 left 2 right 4 construction 5 stop 6 parking 7 tunnel
        self.mode_msg = mode_msg.data
    
    def construction(self):
        msg = self.obstacle.msg
        #print msg
        speed = 0
        angular = 0
        if self.phase == 0:
            # print 'Phase 1' , self.status
            if self.status == 0:
                speed = 0.2
                if msg.ranges[90] < 0.3 and msg.ranges[90] != 0:
                    self.status = 1
                else:
                    angular = 0
                    
            elif self.status == 1:
                speed = 0.1
                angular = 0.37
                
            if msg.ranges[270] < 0.4 and msg.ranges[270] != 0:
                self.phase += 1
                self.status = 0

            self.move(speed,angular)
                
        elif self.phase == 1:
            speed = 0.1
            angular = -0.4

            if msg.ranges[0] < 0.5 and msg.ranges[0] != 0:
                self.check += 1

            if self.check != 0 and msg.ranges[70] < 0.5 and msg.ranges[70] != 0:
                self.phase += 1

            self.move(speed,angular)
                
        elif self.phase == 2:
            
            speed = 0.12
            angular = 0.36

            if msg.ranges[200] < 0.5 and msg.ranges[200] != 0:
                self.phase += 1

            self.move(speed,angular)
        
        elif self.phase == 3:

            self.mode = 3
            # print 'Phase 4'# rospy.sleep(rospy.Duration(2))
            print('break construction')
            #self.mode += 1
        print('speed, angluar : ', speed,angular)


    def run(self):
        rospy.on_shutdown(self.myhook)
        img = self.cam_img
        img2 = self.cam_img2

        self.left_lane, self.right_lane, self.center = self.line.detect_lines()

        msg = self.obstacle.msg
        speed = 0
        angular = 0
        self.mode_msg =5

        if self.mode == 0 :
            #self.go = Light(img).find()
            self.go = True
            #print(self.go)
            if self.go == True :
                self.mode += 1

        # green light && Left, Rigth Sign
        if self.mode == 1 and self.go == True:
                
            if self.mode_msg == 1:
                self.center = 200
                speed, angular = self.PD_control(kp = 0.012,kd = 0.004)
                self.move(speed,angular)
            elif self.mode_msg == 2:
                self.center = 420
                speed, angular = self.PD_control(kp = 0.012,kd = 0.004)
                self.move(speed,angular)
            
            else :
                speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
                self.move(speed,angular)
                if self.mode_msg ==4:
                    self.mode=2

        #Construction
        if self.mode == 2 and self.mode_msg == 4:
            self.construction()
        
        if self.mode == 3 and self.mode_msg == 6:
            self.park_set = self.line.parking_set()
            
            if self.park ==0 and self.park_set == 1:
                self.move(0.1,0)
                rospy.sleep(rospy.Duration(5))
                self.move(0,45)
                rospy.sleep(rospy.Duration(1))
                self.move(0,0)
                rospy.sleep(rospy.Duration(2))
                self.zero = self.line.parking()
                self.park =3
            if self.park ==3:
                if self.zero < 100:
                    self.move(-0.1,0)
                    rospy.sleep(rospy.Duration(3))
                    self.park=4
                else :
                    self.move(0.1,0)
                    rospy.sleep(rospy.Duration(3))
                    self.park=4
            if self.park ==4:
                self.move(0,0)
                rospy.sleep(rospy.Duration(3))
            else:
                speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
                self.move(speed,angular)
            
                

        
        # Stop bar
        if self.mode == 4  and self.mode_msg == 5:
            self.Chadan = Stopbar(img2).find()
            #print(self.Chadan)
            if self.Chadan > 1:
                self.move(0,0)
                self.Chadan_go = True
            elif self.Chadan == 0 and self.Chadan_go == True:
                self.mode =5
                
            else:
                speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
                self.move(speed,angular)
        if self.mode == 5:
            speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
            self.move(speed,angular)
            print(3333)
            #a = self.stopbar.find
            #print(a)
        # Tunne
 
        #if self.mode == 3:
        
            
        
        #print("traffic", self.go)
        print("mode,sign ", self.mode, self.mode_msg)
        
        print("left : ", self.left_lane)
        print("right: ", self.right_lane)
        print("center : ", self.center)
        #print("speed : ",speed)
        print("angular : ", angular)
        #self.move(speed, angular)
    def myhook(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        
        self.pub_cmd_vel.publish(twist)
    
    def move(self, speed, angular):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular

        self.pub_cmd_vel.publish(twist)

if __name__ == '__main__':
    rospy.init_node('line_trace')
    foscar = AutoDrive()
    time.sleep(1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        foscar.run()
        rate.sleep()
