#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('speeds', anonymous=True)   

def callback_wheel(data): 
    v_wheel=data.twist.twist.linear.x
    print "v_wheel," , v_wheel

def callback_odom(data): 
    v_odom=data.twist.twist.linear.x
    print "v_odom," , v_odom


def listener():
    rospy.Subscriber("/rdwhfeel_odometry_new", Odometry, callback_wheel)
    rospy.Subscriber("/rdodom", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()
