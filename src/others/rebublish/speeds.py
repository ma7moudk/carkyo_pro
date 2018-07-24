#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('speeds', anonymous=True)   

def callback_odom(data): 
    LenghtBetweenTwoWheels=1.01
    wheelBase=1.68
    vR=data.twist.twist.linear.x
    vL=data.twist.twist.linear.y
    V_mps=(vR+vL)/2.0
    print V_mps

def listener():
    rospy.Subscriber("/wheel_odometry", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()
