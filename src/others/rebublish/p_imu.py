#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

rospy.init_node('fix_imu', anonymous=False)  
imu_pub = rospy.Publisher('/imu_enu_new', Imu, queue_size=10)
imuMsg = Imu() 

while 1:
	imuMsg.header.stamp=rospy.Time.now()
	imu_pub.publish(imuMsg)
	print "v"




