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

from TrafficLight_1029 import Light
from LineDetector_1029 import detect_sign
from ObstacleDetector import Obstacle
from StopbarDetector import Stopbar

class AutoDrive:
    
    def __init__(self):

        self.mode = 0

        self.left_lane = False
        self.right_lane = False
        self.center = 0
        self.lastError = 0
        self.MAX_VEL = 0.2

        self.cam_img = np.zeros(shape=(360, 640, 3), dtype=np.uint8)
        self.cam_img2 = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.go = False

        self.version = 1
        self.msg = []
        # three-street
        self.right_step = 1

        # Construction
        self.phase = 0
        self.status = 0
        self.check =0

        # Parking

        # Stop
        self.Chadan = 0
        self.Chadan_go = False
        
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.mode_msg = UInt8()

        self.sub_img1 = rospy.Subscriber('/camera1/usb_cam1/image_raw/compressed', CompressedImage,
                                     self.callback1, queue_size=1)
        self.sub_img2= rospy.Subscriber('/camera2/usb_cam2/image_raw/compressed', CompressedImage,
                                     self.callback2, queue_size=1)
        self.sub_detect_sign = rospy.Subscriber('/detect/traffic_sign', UInt8, self.mode_selector, queue_size=1)
        self.sub_obstacle = rospy.Subscriber('/scan',LaserScan,self.obstacle_callback,queue_size=1)
        

    def PD_control(self, kp =0.012,kd = 0.004):
        error = self.center - 327
        print('error : ', error)
        angular_z = kp * error + kd * (error - self.lastError)
        self.lastError = error
        speed = min(self.MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.2:)
        if abs(error) > 200:
            angular = -max(angular_z, -2.0 if angular_z < 0 else -min(angular_z,2.0))
        else:
            angular = -max(angular_z, -1.3 if angular_z < 0 else -min(angular_z,1.3))
        
        return speed, angular

    def callback1(self,img_data):
        self.cam_img = self.bridge.compressed_imgmsg_to_cv2(img_data, "bgr8")
        self.left_lane, self.right_lane, self.center = Line(self.cam_img,self.version).detect_lines()

    def callback2(self,img_data):
        self.cam_img2 = self.bridge.compressed_imgmsg_to_cv2(img_data,"bgr8")

    def mode_selector(self, mode_msg):
        # 1 left 2 right 4 construction 5 stop 6 parking 7 tunnel
        if mode_msg == 2:
            self.right_sign = True
        

    def obstacle_callback(self,obstacle_msg):
        self.msg = obstacle_msg
    
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
        
        speed = 0
        angular = 0 
        if self.mode == 0 :
            self.go = Light(img).find()
            if self.go == True :
                self.mode += 1

        # green light && Left, Rigth Sign
        if self.mode == 1 and self.go == True:
            speed, angular = self.PD_control(kp = 0.012,kd = 0.004)

            if self.mode_msg == 1:
                self.mode = 2
            if self.mode_msg == 2:
                if self.right_step == 1:
                    self.move(0,0)
                    rospy.sleep(rospy.Duration(1))
                    self.move(0.1,-0.37)
                    rospy.sleep(rospy.Duration(3))
                    self.mode = 2

        #Construction
        if self.mode == 2 :
            if self.mode_msg == 4:
            self.construction()
        
        # parking 
        if self.mode == 3:
            if self.mode_msg ==6 :
                self.execute_parking_mode_with_lidar(msg)
            else:
                speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
                self.move(speed,angular)

        # Stop bar
        if self.mode == 5 :
            self.Chadan = Stopbar(img2).find()
            #print(self.Chadan)
            if self.Chadan > 1:
                self.move(0,0)
                self.Chadan_go = True
            elif self.Chadan == 0 and self.Chadan_go == True:
                self.mode =6
                
            else:
                speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
                self.move(speed,angular)
        if self.mode == 6:
            speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
            self.move(speed,angular)
            #a = self.stopbar.find
            #print(a)

        # Tunnel
        '''
        if self.mode == 6 and self.mode_msg == 7:
            if 0.05 < msg.ranges[290] <0.3:
                if self.tunnel_step == 3:
                    os.system('roslaunch foscar_turtlebot3_autorace tunnel.launch')
                    os.system('rosnode kill /detect_signs')
                    os.system('rosnode kill /line_trace')
                    self.tunnel_step = 4
            
            speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
            self.move(speed,angular)
        '''

        
            
        
        #print("traffic", self.go)
        print("mode,sign ", self.mode, self.mode_msg)
        print("left : ", self.left_lane)
        print("right: ", self.right_lane)
        print("center : ", self.center)
        #print("speed : ",speed)
        # print("angular : ", angular)
        
        self.move(speed, angular)
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
