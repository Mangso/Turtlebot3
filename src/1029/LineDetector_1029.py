#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class Line:
    def __init__(self,img,version):
        self.cam_img = img
        # yellow
        self.HSV_YELLOW_LOWER = np.array([0,35,0])
        self.HSV_YELLOW_UPPER = np.array([60,255,255])
        
        self.HSV_WHITE_LOWER = np.array([80, 30, 95])
        self.HSV_WHITE_UPPER = np.array([180, 145, 255])

        self.v = version

    def process_img(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        global yellow_binary
        global white_binary

        if self.v == 1:
            yellow_binary = cv2.inRange(hsv, self.HSV_YELLOW_LOWER, self.HSV_YELLOW_UPPER)
            white_binary = cv2.inRange(hsv, self.HSV_WHITE_LOWER, self.HSV_WHITE_UPPER)
        elif self.v ==2:
            yellow_binary = cv2.inRange(hsv, self.HSV_YELLOW_LOWER, self.HSV_YELLOW_UPPER)
            white_binary = cv2.inRange(hsv, self.HSV_YELLOW_LOWER, self.HSV_YELLOW_UPPER)
       
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gaussianB = cv2.GaussianBlur(gray_img, (5, 5), 0)
        canny = cv2.Canny(gaussianB, 50, 150)

        return canny

    def detect_lines(self):
        frame = self.cam_img
        roi = frame[270:300,:]
        canny_img = self.process_img(roi)
        left_lane = False
        right_lane = False
        left_pos = 0 
        right_pos =0
        center = 0
        lines = cv2.HoughLinesP(canny_img, 1, np.pi / 180, 10, 5, 10)

        
        if lines is not None:
            lines = [l[0] for l in lines]
            left_arr = []
            right_arr = []
            for x1, y1, x2, y2 in lines:
                if x1 < 320 and left_lane == False: # left lane
                    if x1 - 20 < 0:
                        x = 0
                    else:
                        x = x1 - 20
                    detect_area = yellow_binary[5:15 , x: x1]
                    nonzero = cv2.countNonZero(detect_area)
                    if nonzero > 20:
                        left_arr.append(x1)

                elif x2 > 320 and right_lane == False: # right lane

                    detect_area = white_binary[5 : 15, x2: x2 + 20]
                    nonzero = cv2.countNonZero(detect_area)
                    # print 'r_x2', x2, nonzero
                    if nonzero > 30:
                        # self.right_pos = x1
                        right_arr.append(x2)

                else:
                    continue

            if len(left_arr) > 0:
                left_lane = True
                left_pos = left_arr[0]
            if len(right_arr) > 0:

                right_lane = True
                right_pos = right_arr[0]

        if left_lane == True and right_lane == True:
            center = (left_pos + right_pos) // 2

        elif left_lane == True and right_lane == False:
            center = left_pos + 289

        elif left_lane == False and right_lane == True:
            center = right_pos - 289

        else:
            pass
        #cv2.circle(frame, (int(center), 355), 5, (0, 0, 255), 3, -1)
        #cv2.circle(frame, (int(right_pos), 355), 5, (255, 0, 0), 3, -1)
        #cv2.circle(frame, (int(left_pos), 355), 5, (0, 255, 0), 3, -1)
        #cv2.imshow('1',frame)
        #cv2.imshow('img',white_binary)
        #cv2.imshow("canny " ,canny_img)
        #cv2.waitKey(1)
        return left_lane, right_lane, center
