#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import LaserScan
from foscar_turtlebot3_autorace.msg import raw_sensor_8_10
from std_msgs.msg import Int8
import copy


class Tunnel:

    def __init__(self,topic):

        self.msg = []
        self.raw = raw_sensor_8_10
        

        rospy.Subscriber(topic,LaserScan,self.callback)

    def callback(self,data):
        self.msg = lidar_msg
        global data2
		global copy_data
		SensorData = data.ranges       # Get the data from Lidar
	

		Front = SensorData[1:18]+SensorData[342:359]
		real_Front = SensorData[1:5] + SensorData[354:359]
		
		Front_Left = SensorData[18:54]                   # Seperate the data depend of direction of turtlebot
		Front_Right = SensorData[306:342]
		
	
		Left_side = SensorData[85:95]
		Right_side = SensorData[265:275]
		
		
		Front_Left_avg = self.avg(Front_Left)
		Front_Right_avg = self.avg(Front_Right)
						# Each data has least 10 values including error values like coming up zero value
					# so if error occured, except error values and make the average of their data			
		
		Left_side_avg = self.avg(Left_side)
		Right_side_avg = self.avg(Right_side)
		Front_avg = self.avg(Front)
		
		real_Front_avg = self.avg(real_Front) 

########################################################################################################################################
####  Save the process values in messege form  #########################################################################################
########################################################################################################################################

		self.raw.data = [real_Front_avg, Left_side_avg, Right_side_avg]	

		self.raw.sharp = [SensorData[40], SensorData[29], SensorData[20], SensorData[9], SensorData[350], SensorData[339], SensorData[329], SensorData[320]]
	
		self.raw.front = [avg(SensorData[37:40]), avg(SensorData[33:35]), avg(SensorData[26:29]), avg(SensorData[23:25]), avg(SensorData[16:18]), avg(SensorData[13:15]), avg(SensorData[344:346]), avg(SensorData[338:340]), avg(SensorData[332:335]), avg(SensorData[330:334]), avg(SensorData[329:331]), avg(SensorData[321:324]) ]
	
########################################################################################################################################


		data2 = [Front_Left_avg, Front_avg , Front_Right_avg]

		copy_data = copy.copy(data2)           # data2 values are relocated by 'data2.sort()' So they have to be copied. 

		data2.sort(reverse=True)	       # Relocation values to large values to small values. 

		find_same(data2[0])		       # Give the large value to Function. Large value means far distance from turtlebot



    def find_same(data1):
        global data2
        global copy_data
        
        for i in range(0, len(copy_data)):   # Find the same value from copied array 'data2'  
            if copy_data[i] == data1:
                raw.Flag = i+1	      # Publish the symbolic number of the Large value before the 'data2' array is relocated  	
                raw.Far = data1	      # Symbolic number means the direction
				pub.publish(raw)
    def avg(data):
        return 	sum(data)/(len(data)-data.count(0)+0.01)