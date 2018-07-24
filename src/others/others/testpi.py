#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('listener', anonymous=True)   
odom_pub = rospy.Publisher('/wheel_odometry_new', Odometry, queue_size=10)
odomMsg = Odometry()

def callback_odom(data): 
    print rospy.Time.now() , data.header.stamp , rospy.Time.now()-data.header.stamp , data.child_frame_id



def listener():
    rospy.Subscriber("/wheel_odometry", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()
