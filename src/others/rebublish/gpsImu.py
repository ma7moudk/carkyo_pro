#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
from sensor_msgs.msg import Imu
rospy.init_node('wheel_listener', anonymous=True)   
odom1_pub = rospy.Publisher('/imu_gps_fix', Odometry, queue_size=10)
odom2_pub = rospy.Publisher('/imu_gps_float', Odometry, queue_size=10)
odom1Msg = Odometry()
odom2Msg = Odometry()

orientation_list=[0.0,0.0,0.0,1.0]


def callback_odom(data): 
    global orientation_list
    odom1Msg=data
    odom1Msg.pose.pose.position.x= data.pose.pose.position.x
    odom1Msg.pose.pose.position.y= data.pose.pose.position.y
    odom1Msg.pose.pose.orientation.x= orientation_list[0]
    odom1Msg.pose.pose.orientation.y= orientation_list[1]
    odom1Msg.pose.pose.orientation.z= orientation_list[2]
    odom1Msg.pose.pose.orientation.w= orientation_list[3]

    odom1Msg.header.stamp= rospy.Time.now()
    odom1_pub.publish(odom1Msg)

def callback_odom2(data): 
    global orientation_list
    odom2Msg=data
    odom2Msg.pose.pose.position.x= data.pose.pose.position.x
    odom2Msg.pose.pose.position.y= data.pose.pose.position.y
    odom2Msg.pose.pose.orientation.x= orientation_list[0]
    odom2Msg.pose.pose.orientation.y= orientation_list[1]
    odom2Msg.pose.pose.orientation.z= orientation_list[2]
    odom2Msg.pose.pose.orientation.w= orientation_list[3]
    odom2Msg.header.stamp= rospy.Time.now()
    odom2_pub.publish(odom2Msg)

def callback_IMU(data): 
    global orientation_list
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 

def listener():
    rospy.Subscriber("/odometry/gps", Odometry, callback_odom)
    rospy.Subscriber("/odometry2/gps", Odometry, callback_odom2)
    rospy.Subscriber("/imu_enu_new", Imu, callback_IMU)
    rospy.spin()

if __name__ == '__main__':
    listener()
