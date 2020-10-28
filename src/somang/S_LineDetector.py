#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class Line:
    def __init__(self,topic):
        self.cam_img = np.zeros(shape=(360, 640, 3), dtype=np.uint8)
        self.out = cv2.VideoWriter('0920_2.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,360))
        self.bridge = CvBridge()
        # yellow
        self.HSV_YELLOW_LOWER = np.array([20,0,0])
        self.HSV_YELLOW_UPPER = np.array([90,255,255])
        # 
        self.HSV_WHITE_LOWER = np.array([50, 10, 115])
        self.HSV_WHITE_UPPER = np.array([180, 120, 255])

        rospy.Subscriber(topic,CompressedImage,self.conv_img)

    def __def__(self):
        self.out.release()
        cv2.destroyAllWindows()

    def conv_img(self,data):
        self.cam_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.out.write(self.cam_img)

    def process_img(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        global yellow_binary
        global white_binary
        yellow_binary = cv2.inRange(hsv, self.HSV_YELLOW_LOWER, self.HSV_YELLOW_UPPER)
        white_binary = cv2.inRange(hsv, self.HSV_WHITE_LOWER, self.HSV_WHITE_UPPER)
       
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gaussianB = cv2.GaussianBlur(gray_img, (5, 5), 0)
        canny = cv2.Canny(gaussianB, 50, 150)

        return canny

    def detect_lines(self):
        frame = self.cam_img
        roi = frame[230:250,:]
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

                    detect_area = white_binary[5: 15, x2: x2 + 20]
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
            center = left_pos + 330

        elif left_lane == False and right_lane == True:
            center = right_pos - 330

        else:
            pass
        
        #cv2.imshow('img1',yellow_binary)
        #cv2.imshow('img',white_binary)
        #cv2.imshow("canny " ,canny_img)
        #cv2.waitKey(1)
        return left_lane, right_lane, center

    def parking(self):
        frame = self.cam_img
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, self.HSV_WHITE_LOWER, self.HSV_WHITE_UPPER)
        
        #left_area =white[: ,0: 320]
        #right_area = white[: , 320: 640]
        area = white[50:200,250:450]
        zero = cv2.countNonZero(area)
        #leftzero = cv2.countNonZero(left_area)
        #rightzero = cv2.countNonZero(right_area)

        #cv2.imshow('left',area)
        #cv2.imshow('right', right_area)
        #cv2.waitKey(1)
        
        return zero
    def parking_set(self):
        frame = self.cam_img
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, self.HSV_WHITE_LOWER, self.HSV_WHITE_UPPER)
        #area = white[:,200:400]
        
        left_area =white[50:150: ,0: 100]
        right_area = white[50:150 , 540: 640]
        #area = white[50:200,250:450]
        #zero = cv2.countNonZero(area)
        leftzero = cv2.countNonZero(left_area)
        rightzero = cv2.countNonZero(right_area)

        #cv2.imshow('left',left_area)
        #cv2.imshow('right', right_area)
        #cv2.waitKey(1)
        if leftzero < rightzero:
            return 1
        else :
            return 2


