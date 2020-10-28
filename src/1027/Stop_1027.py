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
        
        self.red_lower=np.array([161,155,84],np.uint8)
        self.red_upper=np.array([180,210,158],np.uint8)
        
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
            if(area>50):
                x,y,w,h = cv2.boundingRect(contour)	
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(img,"red",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.1, (0,0,255)); #print('red')
                self.red += 1
        
        #cv2.imshow("img",img)
        #cv2.waitKey(1)
        
        return self.red
