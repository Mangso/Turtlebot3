#!/usr/bin/env python
# -- coding: utf-8 -- # 한글 주석쓰려면 이거 해야함
# Standard libraries
#from argparse import ArgumentParser
# ROS libraries
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# Other libraries
import cv2
import numpy as np
def nothing(x):
   pass

def main(subscriber):
   """Creates a camera calibration node and keeps it running."""
   # Initialize node
   rospy.init_node('node')
   # Initialize CV Bridge
   bridge = CvBridge()
   # Create a named window to calibrate HSV values in
   cv2.namedWindow('LAB Calibrator')
   # Creating track bar
   cv2.createTrackbar('L_low', 'HSV Calibrator', 0, 100, nothing)
   cv2.createTrackbar('A_low', 'HSV Calibrator', -126, 126, nothing)
   cv2.createTrackbar('B_low', 'HSV Calibrator', -126, 126, nothing)
   cv2.createTrackbar('L_high', 'HSV Calibrator', 0, 100, nothing)
   cv2.createTrackbar('A_high', 'HSV Calibrator', -126, 126, nothing)
   cv2.createTrackbar('B_high', 'HSV Calibrator', -126, 126, nothing)
   cv2.setTrackbarPos('L_low', 'HSV Calibrator', 0)
   cv2.setTrackbarPos('L_high', 'HSV Calibrator',100)
   cv2.setTrackbarPos('A_low', 'HSV Calibrator', -126)
   cv2.setTrackbarPos('A_high', 'HSV Calibrator',126)
   cv2.setTrackbarPos('B_low', 'HSV Calibrator', -126)
   cv2.setTrackbarPos('B_high', 'HSV Calibrator',126)
   # Subscribe to the specified ROS topic and process it continuously
   # rospy.Subscriber(subscriber, Image, calibrator, callback_args=(bridge))
   while not rospy.is_shutdown():
       data = rospy.wait_for_message('/camera1/usb_cam1/image_raw', Image)
       raw = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
       hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2LAB)
       # get info from track bar and appy to result
       h_low = cv2.getTrackbarPos('L_low', 'LAB Calibrator')
       s_low = cv2.getTrackbarPos('A_low', 'LAB Calibrator')
       v_low = cv2.getTrackbarPos('B_low', 'LAB Calibrator')
       h_high = cv2.getTrackbarPos('L_high', 'LAB Calibrator')
       s_high = cv2.getTrackbarPos('A_high', 'LAB Calibrator')
       v_high = cv2.getTrackbarPos('B_high', 'LAB Calibrator')
       # Normal masking algorithm
       lower = np.array([h_low, s_low, v_low])
       upper = np.array([h_high, s_high, v_high])
       mask = cv2.inRange(hsv, lower, upper)
       result = cv2.bitwise_and(raw, raw, mask=mask)
       # return result
       # cv2.imshow('HSV Calibrator', result)
       # do stuff
       # cv2.imshow('HSV Calibrator', calibrator())
       cv2.imshow('LAB Calibrator', result)
       cv2.waitKey(1)
if __name__ == "__main__":
   main('camera1/usb_cam1/image_raw')
