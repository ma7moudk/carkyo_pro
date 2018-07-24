#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('speeds', anonymous=True)   

def callback_odom(data): 
    v_odom=data.twist.twist.linear.x
    print "v_odom," , v_odom ,"," , data.header.stamp.secs, "," ,data.header.stamp.nsecs


def listener():
    rospy.Subscriber("/rdodom", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()
