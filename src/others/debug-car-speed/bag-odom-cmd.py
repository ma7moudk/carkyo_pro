#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math , time
from ackermann_msgs.msg import AckermannDriveStamped
rospy.init_node('wheel_listener', anonymous=True)   


steering=0.0 ; cmd_speed=0.0 ; cmd_sec=0.0
wheel_speed =0.0 ;  wheel_sec=0.0
odom_speed=0.0 ; odom_sec=0.0
wheel_nano=0.0 ; odom_nano=0.0 ;  cmd_nano=0.0

print "steering , cmd_speed , pi_cmd_speed , wheel_speed , wheel_sec, wheel_nano , odom_speed , odom_sec, odom_nano"

def callback_odom(data): 
    global odom_speed , odom_sec , odom_nano
    odom_speed=data.twist.twist.linear.x
    odom_sec=data.header.stamp.secs
    odom_nano=data.header.stamp.nsecs

def callback_wheel(data): 
    global steering , cmd_speed , cmd_sec , cmd_nano
    global wheel_speed , wheel_sec
    global odom_speed , odom_sec , odom_nano
    wheel_speed=data.twist.twist.linear.x
    pi_cmd_speed = data.twist.twist.angular.x
    wheel_sec=data.header.stamp.secs
    wheel_nano=data.header.stamp.nsecs
    print steering ,",", cmd_speed ,",", pi_cmd_speed ,",",  wheel_speed ,",", wheel_sec ,",",  wheel_nano ,",", odom_speed ,",", odom_sec ,",", odom_nano

def callback_cmd(data): 
    global steering , cmd_speed
    steering=data.angular.z
    cmd_speed=data.linear.x



def listener():
    rospy.Subscriber("/rdwheel_odometry_new", Odometry, callback_wheel)
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd)
  #  rospy.Timer(rospy.Duration(0.1), publish)
    rospy.spin()

if __name__ == '__main__':
    listener()
