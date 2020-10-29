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

        # three-street
        self.right_step = 1

        # Construction
        self.phase = 0
        self.status = 0
        self.check =0

        # Parking
        self.parking_phase = 1
        self.is_left_turtlebot = False
        self.is_right_turtlebot = False
        self.found_turtlebot =False

        # Stop
        self.Chadan = 0
        self.Chadan_go = False

        self.tunnel_step =3
        

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
        error = self.center - 280
        print('error : ', error)
        angular_z = kp * error + kd * (error - self.lastError)
        self.lastError = error
        speed = min(self.MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.2)
        angular = -max(angular_z, -1.4 if angular_z < 0 else -min(angular_z,1.4))
        
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

    def execute_parking_mode_with_lidar(self, msg):
        left_range = msg.ranges[10:30]
        right_range = msg.ranges[270:290]
        back_range = msg.ranges[175:185]

        #print(left_range)
        #print(right_range)
        if self.parking_phase == 1:  # both lanes are yellow
            if self.found_turtlebot == False:  # until find parking turtlebot
                for i in left_range:
                    # print left_range
                    if 0 < i < 0.6:
                        print 'parking turlebot in left'
                        self.is_left_turtlebot = True
                        self.is_right_turtlebot = False
                        self.parking_phase = 2
                        break
                for i in right_range:
                    if 0 < i < 0.6:
                        print 'parking turlebot in right'
                        self.is_left_turtlebot = False
                        self.is_right_turtlebot = True
                        break

        elif self.parking_phase == 2:  # '''왼쪽에잇으면 오른쪽으로 회전, 오른쪽에 잇으면 왼쪽으로 회전// 뒤통수에 터틀봇이 잡힐때까지 회전#'''
            if self.is_left_turtlebot == True:
                speed = 0
                angular = math.radians(-45)
            else:
                speed = 0
                angular = math.radians(45)

        elif self.parking_phase == 3:  # '''3초동안 직진#'''
            speed = 0.1
            angular = 0
            self.move(speed, angular)
            rospy.sleep(rospy.Duration(3))
            self.parking_phase += 1

        elif self.parking_phase == 4:  # '''뒤 돈다#'''
            speed = 0
            if self.is_left_turtlebot == True:
                angular = math.radians(-90)
                self.move(speed, angular)
                rospy.sleep(rospy.Duration(2))  # '''2초동안#'''
                self.parking_phase += 1

            else:
                angular = math.radians(90)
                self.move(speed, angular)
                rospy.sleep(rospy.Duration(2))
                self.parking_phase += 1

            speed = 0
            angular = 0
            self.move(speed, angular)
            rospy.sleep(rospy.Duration(0.1))  # '''이거는 주차하고 속도가 0인 순간이 필요해서 넣은 슬립 함수#'''

        elif self.parking_phase == 5:  # '''여기서 탈출 한다 2.8초동안 하드 코딩함#'''
            speed = 0.2
            if self.is_left_turtlebot == True:
                angular = math.radians(55)
            else:
                angular = math.radians(-55)
            self.move(speed, angular)
            rospy.sleep(rospy.Duration(2.8))
            self.mode = 6

    def run(self):

        rospy.on_shutdown(self.myhook)
        img = self.cam_img
        img2 = self.cam_img2

        self.left_lane, self.right_lane, self.center = self.line.detect_lines()
        msg = self.obstacle.msg
        speed = 0
        angular = 0
        #self.mode_msg =6
        #print(msg)
        
        if self.mode == 0 :
            #self.go = Light(img).find()
            self.go = True
            #print(self.go)
            if self.go == True :
                self.mode += 1

        # green light && Left, Rigth Sign
        if self.mode == 1 and self.go == True:
                
            if self.mode_msg == 1:
               # self.center = 200
                speed, angular = self.PD_control(kp = 0.012,kd = 0.004)
                self.move(speed,angular)
            elif self.mode_msg == 2:
                if self.right_step == 1:
                    self.move(0,0)
                    rospy.sleep(rospy.Duration(1.5))
                    self.move(0,-0.37)
                    rospy.sleep(rospy.Duration(3))
                    self.right_step =2
                elif self.right_step == 2:
                    speed, angular = self.PD_control(kp = 0.012,kd = 0.004)
                    self.move(speed,angular)
                    if self.mode_msg == 4:
                        self.mode = 2
            else :
                speed, angular = self.PD_control(kp = 0.008,kd = 0.004)
                self.move(speed,angular)
                if self.mode_msg ==4:
                    self.mode= 2

        #Construction
        if self.mode == 2 and self.mode_msg == 4:
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
        # print("mode,sign ", self.mode, self.mode_msg)
        print("left : ", self.left_lane)
        print("right: ", self.right_lane)
        print("center : ", self.center)
        #print("speed : ",speed)
        # print("angular : ", angular)
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
