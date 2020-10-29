#!/usr/bin/env python
# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class Stopbar:
    
    def __init__(self,frame):
        self.cam_img = frame
        
        self.red_lower=np.array([0,60,60],np.uint8)
        self.red_upper=np.array([180,155,120],np.uint8)

        self.HSV_WHITE_LOWER = np.array([0, 0, 159])
        self.HSV_WHITE_UPPER = np.array([180, 30, 255])
        
        self.red_cnt=0
        self.red = 0
        
    def find(self):
        img = self.cam_img
        img = img[100:200, 50:600]
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        #finding the range of red,green and yellow color in the image
        red=cv2.inRange(hsv,self.red_lower,self.red_upper)
        
        
        kernal = np.ones((5,5), "uint8")
        
        red = cv2.dilate(red, kernal)
        
        res1 = cv2.bitwise_and(img, img, mask = red)
        
        contours, hierarchy =cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>1500):
                x,y,w,h = cv2.boundingRect(contour)	
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(img,"red",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.1, (0,0,255)); #print('red')
                self.red += 1
        #v2.imshow("img",img)
        #cv2.imshow("img2",res1)
        #cv2.waitKey(1)
        
        return self.red
        
    def parking(self):
        img = self.cam_img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, self.HSV_WHITE_LOWER, self.HSV_WHITE_UPPER)
        
        #left_area =white[: ,0: 320]
        #right_area = white[: , 320: 640]
        area = white[400:,:100]
        zero = cv2.countNonZero(area)

        #leftzero = cv2.countNonZero(left_area)
        #rightzero = cv2.countNonZero(right_area)

        cv2.imshow('left',area)
        #cv2.imshow('right', right_area)
        cv2.waitKey(1)
        return zero
        
        return zero
    def parking_set(self):
        img = self.cam_img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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



