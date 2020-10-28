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

from Linetest import Line
from TrafficLight import Light
#from ObstacleDetector import Obstacle

class AutoDrive:
    
    def __init__(self):

        self.mode = 0

        self.left_lane = False
        self.right_lane = False
        self.center = 0
        self.lastError = 0
        self.MAX_VEL = 0.2

        self.cam_img = np.zeros(shape=(360, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.go = False

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        
        self.traffic_light = Light('/usb_cam/image_raw/compressed')
        self.sub_img = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage,
                                               self.callback, queue_size=1)
        self.sub_detect_sign = rospy.Subscriber('/detect/traffic_sign', UInt8, self.mode_selector, queue_size=1)
        self.mode_msg = UInt8()

    def PD_control(self, kp =0.012,kd = 0.004):
        error = self.center - 300
        angular_z = kp * error + kd * (error - self.lastError)
        self.lastError = error
        speed = min(self.MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.22)
        angular = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        return speed, angular

    def callback(self,img_data):
        self.left_lane = False
        self.right_lane = False
        self.cam_img = self.bridge.compressed_imgmsg_to_cv2(img_data, "bgr8")

    def mode_selector(self, mode_msg):
        # 1 left 2 right 4 construction 5 stop 6 parking 7 tunnel
        self.mode_msg = mode_msg.data

    def run(self):
        img = self.cam_img
        self.left_lane, self.right_lane, self.center = Line(img).detect_lines()
        self.go = self.traffic_light.find()
        
        if self.mode == 0 and self.go == True:
            self.mode += 1
        
        if self.mode == 1 : # 삼거리.
            if self.mode_msg == 1:
                self.center = 200
            if self.mode_msg == 2:
                self.center = 420
        

        
        speed , angular = self.PD_control(kp = 0.016, kd= 0.006)
        print("mode : ", self.mode)
        print("sign : ", self.mode_msg)

        print("left : ", self.left_lane)
        print("right: ", self.right_lane)
        print("center : ", self.center)
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
    time.sleep(3)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        foscar.run()
        rate.sleep()
