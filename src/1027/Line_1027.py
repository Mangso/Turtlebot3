#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class Line:
    def __init__(self,topic):
        self.now = datetime.datetime.now()
        self.cam_img = np.zeros(shape=(360, 640, 3), dtype=np.uint8)
        self.out = cv2.VideoWriter('/home/foscar/{}-{}-{}.avi'.format(self.now.day,self.now.hour,self.now.minute),cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,360))
        self.bridge = CvBridge()
        # yellow
        self.HSV_YELLOW_LOWER = np.array([0,0,200])
        self.HSV_YELLOW_UPPER = np.array([180,120,255])
        # 0 0 200 180 31 255
        self.HSV_WHITE_LOWER = np.array([0, 0, 200])
        self.HSV_WHITE_UPPER = np.array([180, 120, 255])

        rospy.Subscriber(topic,CompressedImage,self.conv_img)

    def __def__(self):
        self.out.release()
        cv2.destroyAllWindows()

    def conv_img(self,data):
        self.cam_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.out.write(self.cam_img)

    def process_img(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )
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
        roi = frame[220:240,:]
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
                    detect_area = white_binary[:, x: 320]
                    nonzero = cv2.countNonZero(detect_area)
                    if nonzero > 20:
                        left_arr.append(x1)

                elif x2 > 320 and right_lane == False: # right lane

                    detect_area = white_binary[0: 20, x2: x2 + 20]
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
            center = left_pos + 280

        elif left_lane == False and right_lane == True:
            center = right_pos - 300

        else:
            pass
        
        #cv2.imshow('yellow',yellow_binary)
        #cv2.imshow('binary',white_binary)
        #cv2.imshow("canny " ,canny_img)
        #cv2.waitKey(1)
        return left_lane, right_lane, center

    def parking(self):
        frame = self.cam_img
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, self.HSV_WHITE_LOWER, self.HSV_WHITE_UPPER)
        
        # left_area =white[: ,0: 320]
        # right_area = white[: , 320: 640]
        area = white[50:200,200:500]
        zero = cv2.countNonZero(area)
        #leftzero = cv2.countNonZero(left_area)
        #rightzero = cv2.countNonZero(right_area)

        #cv2.imshow('left',area)
        #cv2.imshow('right', right_area)
        #cv2.waitKey(1)
        
        return zero



