#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('speed2', anonymous=True)   

def callback_odom(data): 
    print  data.pose.pose.position.x , "," , data.pose.pose.position.y


def listener():
    rospy.Subscriber("/imu2_gps_fix", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()
