#!/usr/bin/env python
# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class Light:
	def __init__(self,topic):
		self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
		self.bridge = CvBridge()
		
		self.green_lower=np.array([50,80,80],np.uint8)
		self.green_upper=np.array([80,220,200],np.uint8)
		self.green_cnt=0
		self.green = False
		
		rospy.Subscriber(topic,CompressedImage,self.callback)
		
	def callback(self, img_data):
		self.cam_img = self.bridge.compressed_imgmsg_to_cv2(img_data, "bgr8")

	def find(self):
		img = self.cam_img
		img = img[100:, 320:]
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		#finding the range of red,green and yellow color in the image
		green=cv2.inRange(hsv,self.green_lower,self.green_upper)
		
		kernal = np.ones((5,5), "uint8")
		
		green = cv2.dilate(green, kernal)
		res1 = cv2.bitwise_and(img, img, mask = green)
		
		contours, hierarchy =cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area>300):
				x,y,w,h = cv2.boundingRect(contour)	
				img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
				cv2.putText(img,"Green",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0)); print('green')
				self.green_cnt += 1
				if self.green_cnt > 10:
					self.green=True
					break


                #:cv2.imshow("img",img)
		#cv2.waitKey(1)
		
		return self.green
