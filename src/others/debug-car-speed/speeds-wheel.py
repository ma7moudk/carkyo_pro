#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('speeds', anonymous=True)   

def callback_wheel(data): 
    v_wheel=data.twist.twist.linear.x
    print "v_wheel," , v_wheel ,"," , data.header.stamp.secs, "," ,data.header.stamp.nsecs

def listener():
    rospy.Subscriber("/rdwheel_odometry_new", Odometry, callback_wheel)
    rospy.spin()

if __name__ == '__main__':
    listener()
