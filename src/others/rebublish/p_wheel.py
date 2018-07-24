#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('listener', anonymous=True)   
odom_pub = rospy.Publisher('/wheel_odometry_new', Odometry, queue_size=10)
odomMsg = Odometry()

while 1:
	odomMsg.header.stamp=rospy.Time.now()
	odom_pub.publish(odomMsg)
	print "v"
