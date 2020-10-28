#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class Obstacle:

    def __init__(self,topic):
        
        self.msg = []

        rospy.Subscriber(topic,LaserScan,self.callback)

    def callback(self,obstacle_msg):
        self.msg = obstacle_msg
