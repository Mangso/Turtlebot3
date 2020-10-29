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


class line_traceee:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub_select_ang = rospy.Subscriber('/camera1/usb_cam1/image_raw/compressed', CompressedImage, self.line_trace, queue_size=1)
        self.sub_detect_sigh = rospy.Subscriber('/detect/traffic_sign', UInt8, self.mode_selector, queue_size=1)
        self.sub_obstacle = rospy.Subscriber('/scan', LaserScan, self.obstacle, queue_size=1)
        #self.out2 = cv2.VideoWriter('0920_2.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (640, 360))
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # self.trafficSign = Enum('trafficSign', 'left right dontgo construction stop parking tunnel'
        ## left line
        self.left_lane = False
        self.left_pos = 0

        ## right line
        self.right_lane = False
        self.right_pos = 0

        self.center = 0
        self.lastError = 0
        self.MAX_VEL = 0.2

        self.obstacle_msg = []
        # sign
        self.left_sign = False
        self.right_sign = False
        self.construction_sign = False
        self.parking_sign = False
        self.tunnel_sign = False
        self.stop_sign = False

        # mode
        self.curr_mode = 0
        self.mode = False
        self.construction_mode = False
        self.parking_mode = False

        # construction
        self.phase = 1
        self.check = 0
        self.status = 0

        # stop
        self.stop_phase = 1
        self.is_stop = False
        self.stopchk = False
        self.stop_flag = False

        # parking
        self.parking_phase = 0
        self.is_left_turtlebot = False
        self.is_right_turtlebot = False
        self.found_turtlebot = False
        self.parking_sign_sign = False

        # for detect lane
        self.HSL_YELLOW_LOWER = np.array([0, 35, 0])
        self.HSL_YELLOW_UPPER = np.array([60, 255, 255])

        self.HSL_YELLOW_LOWER2 = np.array([0, 35, 0])
        self.HSL_YELLOW_UPPER2 = np.array([60, 255, 255])

        self.HSL_WHITE_LOWER = np.array([80, 30, 95])
        self.HSL_WHITE_UPPER = np.array([180, 145, 255])

        self.speed = 0.1
        self.angular = 30 * np.pi / 180  # 0#35 * np.pi/180#0.0


    def reset_sign(self):
        self.left_sign = False
        self.right_sign = False
        self.construction_sign = False
        self.parking_sign = False
        self.tunnel_sign = False
        self.stop_sign = False

    def mode_selector(self, mode_msg):
        if self.curr_mode == mode_msg.data:
            pass
        else:
            if mode_msg.data == 1:  # left
                print
                self.mode = True
                self.left_sign = True
                self.angular = 30 * np.pi / 180
                self.parking_sign_sign = False
            elif mode_msg.data == 2:  # right
                print 'Read right'
                self.reset_sign()
                self.mode = True
                self.right_sign = True
                self.angular = -28 * np.pi / 180
            elif mode_msg.data == 4:  # construction
                print 'Read construction'
                self.reset_sign()
                self.construction_sign = True
            elif mode_msg.data == 5:  # stop
                print 'Read stop'
                self.reset_sign()
                self.stop_sign = True
            elif mode_msg.data == 6:  # parking
                print 'Read parking'
                self.mode = True
                self.reset_sign()
                self.parking_sign_sign = True
            elif mode_msg.data == 7:  # tunnel
                return

            else:
                pass


    def execute_construction_mode(self, msg):

        if 0 < msg.ranges[0] < 0.5 and self.construction_mode is False:
            print 'execute_construction_mode'
            # self.construction_sign = False
            self.construction_mode = True

        if self.construction_mode:
            
            if self.phase == 1:
                self.move(0,0)
                rospy.sleep(rospy.Duration(0.3))
                self.move(0,0.37)
                rospy.sleep(rospy.Duration(0.5))
                self.move(0.2,0)
                rospy.sleep(rospy.Duration(1))
                self.status = 1


            elif self.phase == 2:
               

            elif self.phase == 3:
                # self.mode = False
                # print 'Phase 3'
                self.speed = 0.12
                self.angular = 0.36
                if msg.ranges[200] < 0.5 and msg.ranges[200] != 0:
                    self.phase += 1

            elif self.phase == 4:
                # print 'Phase 4'
                # rospy.sleep(rospy.Duration(2))
                self.mode = False
                self.construction_mode = False

            self.move(self.speed, self.angular)
    
    def obstacle(self, obstacle_msg):
        self.obstacle_msg = obstacle_msg

    def myhook(self):  
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def PD_control(self, kp=0.012, kd=0.004):
        error = self.center - 290
        print error
        angular_z = kp * error + kd * (error - self.lastError)
        self.lastError = error
        speed = min(self.MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.2)
        angular = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)

        return speed, angular

    def line_trace(self, img_data):  ### left : -  ### right : +
        rospy.on_shutdown(self.myhook)
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_data, "bgr8")
        
        roi = cv_image[270:300, : 600 ]
        self.get_line(roi)


        self.left_lane = False
        self.right_lane = False

    def process_img(self, frame):
        hsl = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        global yellow_binary
        global white_binary
        #print self.parking_sign_sign
        if self.parking_sign_sign:
            yellow_binary = cv2.inRange(hsl, self.HSL_YELLOW_LOWER2, self.HSL_YELLOW_UPPER2)
            white_binary = cv2.inRange(hsl, self.HSL_YELLOW_LOWER2, self.HSL_YELLOW_UPPER2)
        else:
            yellow_binary = cv2.inRange(hsl, self.HSL_YELLOW_LOWER, self.HSL_YELLOW_UPPER)
            white_binary = cv2.inRange(hsl, self.HSL_WHITE_LOWER, self.HSL_WHITE_UPPER)
        concat_binary = cv2.hconcat([yellow_binary, white_binary])
        #cv2.imshow('concat_binary', concat_binary)
        #cv2.imshow('white',white_binary)
        #cv2.imshow('11',hsl)
        #cv2.waitKey(1)
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gaussianB = cv2.GaussianBlur(gray_img, (5, 5), 0)
        canny = cv2.Canny(gaussianB, 50, 150)

        return canny

    def get_line(self, frame):
        canny_img = self.process_img(frame)
        #cv2.imshow('canny_img', canny_img)
        lines = cv2.HoughLinesP(canny_img, 1, np.pi / 180, 10, 5, 10)

        if lines is not None:
            lines = [l[0] for l in lines]
            left_arr = []
            right_arr = []
            for x1, y1, x2, y2 in lines:
                if x1 < 320 and self.left_lane == False: # left lane
                    if x1 - 20 < 0:
                        x = 0
                    else:
                        x = x1 - 20
                    detect_area = yellow_binary[5: 15, x: x1]
                    nonzero = cv2.countNonZero(detect_area)
                    if nonzero > 30:
                        left_arr.append(x1)

                elif x2 > 320 and self.right_lane == False: # right lane

                    #print x1,y1,x2,y2
                   # print 'right degree', degree
                    detect_area = white_binary[5: 15, x2: x2 + 20]
                    nonzero = cv2.countNonZero(detect_area)
                    # print 'r_x2', x2, nonzero
                    if nonzero > 30:
                        # self.right_pos = x1
                        right_arr.append(x2)

                else:
                    continue

            if len(left_arr) > 0:
                self.left_lane = True
                self.left_pos = left_arr[0]

            if len(right_arr) > 0:
                # print 'bbbbbbbbbbbbbbbbbb'
                self.right_lane = True
                self.right_pos = right_arr[0]

            
        #cv2.circle(frame, (int(self.center),20 ), 5, (0, 0, 255), 3, -1)
        #cv2.circle(frame, (int(self.right_pos),20 ), 5, (255, 0, 0), 3, -1)
        #cv2.circle(frame, (int(self.left_pos),20 ), 5, (0, 255, 0), 3, -1)
            
        #cv2.imshow('frame',frame)
        #cv2.waitKey(1)

    def move(self, speed, angle):
        angular_z = angle
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular_z

        self.pub_cmd_vel.publish(twist)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('line_trace')  # , anonymous=True)
    node = line_traceee()
    node.main()



