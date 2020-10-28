#!/usr/bin/env python
# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함

from sklearn.neighbors import KNeighborsClassifier
from skimage import feature
from std_msgs.msg import UInt8
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

import rospy
import argparse
import cv2
import numpy as np
import os

class Sign:
    def __init__(self,topic):
        #print('init')
        self.cam_img = np.zeros(shape=(360, 640, 3), dtype=np.uint8)
        self.trafficSign = Enum('trafficSign', 'left right dontgo construction stop parking tunnel')
        self.bridge = CvBridge()
        self.HSL_RED_LOWER = np.array([0, 50, 80])
        self.HSL_RED_UPPER = np.array([10, 210, 215])
        self.HSL_RED_LOWER1 = np.array([170, 50, 80])
        self.HSL_RED_UPPER1 = np.array([180, 210, 158])
        self.HSL_YELLOW_LOWER = np.array([10, 77, 84])
        self.HSL_YELLOW_UPPER = np.array([28, 255, 255])
        self.HSL_BLUE_LOWER = np.array([0, 180, 55])
        self.HSL_BLUE_UPPER = np.array([20, 255, 200])
        
        self.data = []
        self.labels = []
        self.path_list = ["/home/somang/catkin_ws/src/Turtlebot/src/img/right/1.jpg", "/home/somang/catkin_ws/src/Turtlebot/src/img/right/2.jpg", 
		"/home/somang/catkin_ws/src/Turtlebot/src/img/right/3.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/right/4.jpg", "/home/somang/catkin_ws/src/Turtlebot/src/img/right/5.jpg", "/home/somang/catkin_ws/src/Turtlebot/src/img/right/6.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/tunnel/1.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/tunnel/2.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/tunnel/3.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/tunnel/4.png", 
        "/home/somang/catkin_ws/src/Turtlebot/src/img/dontgo/1.jpeg", "/home/somang/catkin_ws/src/Turtlebot/src/img/dontgo/2.jpeg", "/home/somang/catkin_ws/src/Turtlebot/src/img/dontgo/3.jpg", "/home/somang/catkin_ws/src/Turtlebot/src/img/dontgo/4.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/stop/1.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/stop/2.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/stop/3.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/stop/4.png",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/construction/1.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/construction/2.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/construction/3.png",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/left/1.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/left/2.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/left/3.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/left/4.jpg", "/home/somang/catkin_ws/src/Turtlebot/src/img/left/5.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/left/6.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/parking/1.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/parking/2.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/parking/3.jpg","/home/somang/catkin_ws/src/Turtlebot/src/img/parking/4.jpg",
        "/home/somang/catkin_ws/src/Turtlebot/src/img/T/1.png", "/home/somang/catkin_ws/src/Turtlebot/src/img/T/2.png"]
        
        self.learn_images()
        self.ready_to_detect()


        
        self.left_count = 0
        self.right_count = 0
        self.dontgo_count = 0
        self.construction_count = 0
        self.stop_count = 0
        self.parking_count = 0
        self.tunnel_count = 0
        
        self.is_sign = False
        self.traffic_sign_msg = UInt8()

        rospy.Subscriber(topic,CompressedImage,self.find_sign)

    def learn_images(self):
        for imagePath in self.path_list:
            make = imagePath.split("/")[-2]
            image = cv2.imread(imagePath)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, (400, 400))
            kernel = np.ones((3, 3), np.uint8)
            erosion = cv2.erode(gray, kernel, iterations=1)
            logo = cv2.resize(erosion, (128, 128))
            
            H = feature.hog(logo, orientations=8, pixels_per_cell=(16, 16),
                                cells_per_block=(4, 4))
                                
            #print(H)
            # update the data and labels
            self.data.append(H)
            self.labels.append(make)
            
            
    def ready_to_detect(self):
        self.model = KNeighborsClassifier(n_neighbors=1)
        self.model.fit(self.data, self.labels)
        
    def find_sign(self,img_data):
        img = self.bridge.compressed_imgmsg_to_cv2(img_data, "bgr8")
        #roi_img = img
        global contours_ty
        global contours_b
        roi_img = img[50:205, :640]
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsl = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        redBinary = cv2.inRange(hsl, self.HSL_RED_LOWER, self.HSL_RED_UPPER)
        redBinary1 = cv2.inRange(hsl, self.HSL_RED_LOWER1, self.HSL_RED_UPPER1)
        # redBinary = cv2.bitwise_or(redBinary, redBinary1)
        binary_red = cv2.bitwise_or(redBinary, redBinary1)
        blueBinary = cv2.inRange(hsl, self.HSL_BLUE_LOWER, self.HSL_BLUE_UPPER)
        # binary = cv2.bitwise_or( blueBinary, redBinary)
        binary_blue = cv2.bitwise_or( blueBinary, blueBinary)
        yellowBinary = cv2.inRange(hsl, self.HSL_YELLOW_LOWER, self.HSL_YELLOW_UPPER)
        # binary = cv2.bitwise_or(binary, yellowBinary)
        binary_yellow = cv2.bitwise_or( binary_red, yellowBinary)
        cv2.imshow('binary_yellow', binary_yellow)
        cv2.imshow('binary_blue', binary_blue)
        _, contours_ry, hierachy = cv2.findContours(binary_yellow, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        _, contours_b, hierachy = cv2.findContours(binary_blue, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # print(len(contours))

        #traffic_sign_msg = UInt8()
        
        kernel = np.ones((3, 3), np.uint8)
        contours_ry_count = 0
        contours_b_count = 0
        for cnt_1 in contours_ry:
            # area = cv2.contourArea(cnt)
            # print(area)
            binary_ry = cv2.drawContours(binary_yellow, [cnt_1], -1, (255,255,255), -1)
            contours_ry_count += 1
            contours_b_count = 0
            
        for cnt_2 in contours_b:
            # area = cv2.contourArea(cnt)
            # print(area)
			binary_b = cv2.drawContours(binary_blue, [cnt_2], -1, (255,255,255), -1)
			contours_b_count += 1
            
        if contours_b_count is not 0:
            _, goodContours_b, hierachy = cv2.findContours(binary_b, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            gray_b = cv2.bitwise_and(binary_b, gray)
            
        else:
            goodContours_b = 0
            
        if contours_ry_count is not 0:
            _, goodContours_ry, hierachy = cv2.findContours(binary_ry, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            gray_ry = cv2.bitwise_and(binary_ry, gray)
        else:
            goodContours_ry = 0
            
        if goodContours_ry is not 0:
            for cnt_1 in goodContours_ry:
                area = cv2.contourArea(cnt_1)
                # print(area)
                if area > 2000.0 :
                    x, y, w, h = cv2.boundingRect(cnt_1)
                    rate = float(float(h) / float(w))
                    # print w, h
                    if 60 < w < 100 and 60 < h < 100 and 0.8 < rate < 1.2:
                        # print rate, h, w
                        cv2.rectangle(roi_img, (x, y), (x+w, y+h), (250, 152, 50), 2)
                        inputImage = gray[y:y+h, x:x+w]
                        logo = cv2.resize(inputImage, (128, 128))
                        # print rate, "rate"
                        #cv2.imshow("logo", logo)
                        H = feature.hog(logo, orientations=8, pixels_per_cell=(16, 16),
                                            cells_per_block=(4, 4))
                        #cv2.imshow("hog", hogImage)
                        parking_bool = False
                        pred = self.model.predict(H.reshape(1, -1))[0]
                        # print(pred.title())
                        # left right dontgo construction stop parking tunnel
                        
                        #traffic_sign_msg = UInt8()
                        
                        #print(pred.title())
                        if pred.title() in ['Construction', 'Tunnel', 'Stop', 'T'] and x < 150:
                            self.tunnel_count += 1
                            if self.tunnel_count > 5 :
                                self.traffic_sign_msg = self.trafficSign.tunnel.value
                                #self.pub.publish(traffic_sign_msg)
                                rospy.loginfo('tunnel')
                                self.is_sign = True
                        elif pred.title() == 'Dontgo' :
                            print 'dontgo', self.dontgo_count
                            self.dontgo_count += 1
                            if self.dontgo_count > 10 :
                                self.traffic_sign_msg = self.trafficSign.dontgo.value
                                #self.pub.publish(traffic_sign_msg)
                                rospy.loginfo('dontgo')
                                self.is_sign = True
                                
                        elif pred.title() == 'Construction' and x > 150 :
                            self.construction_count += 1
                            #print(pred.title())
                            #print(self.construction_count)
                            if self.construction_count > 5 :
                                self.traffic_sign_msg = self.trafficSign.construction.value
                                #self.pub.publish(traffic_sign_msg)
                                rospy.loginfo('construction')
                                self.is_sign = True
                        elif pred.title() == 'Stop':
                            self.stop_count += 1
                            if self.stop_count > 5 :
                                self.traffic_sign_msg = self.trafficSign.stop.value
                                #self.pub.publish(traffic_sign_msg)
                                rospy.loginfo('stop')
                                self.is_sign = True
                        else : 
                            pass
                                
                        if self.is_sign == True:
                            self.left_count = 0
                            self.right_count = 0
                            self.dontgo_count = 0
                            self.construction_count = 0
                            self.stop_count = 0
                            self.parking_count = 0
                            self.tunnel_count = 0
                            self.is_sign =False
                        cv2.putText(roi_img, pred.title(), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, \
                        (70, 255, 0), 2)	
                        
        if goodContours_b is not 0:
            for cnt_2 in goodContours_b:
				area = cv2.contourArea(cnt_2)
				# print(area)
				if area > 2000.0 :
					x, y, w, h = cv2.boundingRect(cnt_2)
					rate = float(float(h) / float(w))
					# print w, h
					if 60 < w < 100 and 60 < h < 100 and 0.8 < rate < 1.1:
						# print rate, h, w
						cv2.rectangle(roi_img, (x, y), (x+w, y+h), (250, 152, 50), 2)
						inputImage = gray[y:y+h, x:x+w]
						logo = cv2.resize(inputImage, (128, 128))
						# print rate, "rate"
						#cv2.imshow("logo", logo)
						H = feature.hog(logo, orientations=8, pixels_per_cell=(16, 16),
							cells_per_block=(4, 4))
						#cv2.imshow("hog", hogImage)
						parking_bool = False
						pred = self.model.predict(H.reshape(1, -1))[0]
						# print(pred.title())
						# left right dontgo construction stop parking tunnel
						#traffic_sign_msg = UInt8()
				
						#print(pred.title())
						if pred.title() == 'Left' :
							print 'left', self.left_count
							self.left_count += 1
							if self.left_count > 5 :
								self.traffic_sign_msg = self.trafficSign.left.value
								#self.pub.publish(traffic_sign_msg)
								rospy.loginfo('left')
								self.is_sign = True
						elif pred.title() == 'Right' :
							self.right_count += 1
							print 'right', self.right_count
							if self.right_count > 5 :
								self.traffic_sign_msg = self.trafficSign.right.value
								#self.pub.publish(traffic_sign_msg)
								rospy.loginfo('right')
								self.is_sign = True
						elif pred.title() == 'Parking':
							self.parking_count += 1
							if self.parking_count > 3 :
								self.traffic_sign_msg = self.trafficSign.parking.value
								# rospy.sleep(rospy.Duration(1.5))
								#self.pub.publish(traffic_sign_msg)
								rospy.loginfo('parking')
								self.is_sign = True
						else : 
							pass
						if self.is_sign == True:
							self.left_count = 0
							self.right_count = 0
							self.dontgo_count = 0
							self.construction_count = 0
							self.stop_count = 0
							self.parking_count = 0
							self.tunnel_count = 0
							self.is_sign =False
						cv2.putText(roi_img, pred.title(), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, \
							(70, 255, 0), 2)
                            
        cv2.imshow("candidates", roi_img)
        cv2.waitKey(1)