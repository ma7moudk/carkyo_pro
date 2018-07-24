#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
from sensor_msgs.msg import Imu
rospy.init_node('wheel_listener', anonymous=True)   


odom1Fix_pub = rospy.Publisher('/imu1_gps_fix', Odometry, queue_size=10)
odom1Float_pub = rospy.Publisher('/imu1_gps_float', Odometry, queue_size=10)
odom2Fix_pub = rospy.Publisher('/imu2_gps_fix', Odometry, queue_size=10)
odom2Float_pub = rospy.Publisher('/imu2_gps_float', Odometry, queue_size=10)


odomFix1Msg = Odometry()
odomFloat1Msg = Odometry()
odomFix2Msg = Odometry()
odomFloat2Msg = Odometry()

orientation_list =[0.0,0.0,0.0,1.0]
orientation_list2=[0.0,0.0,0.0,1.0]

fix=Odometry()
flooat=Odometry()

def callback_odom(data): 
    global fix
    fix=data

def callback_odom2(data): 
    global flooat
    flooat=data



def callback_IMU(data): 
    global orientation_list
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 

def callback_IMU2(data2): 
    global orientation_list2
    orientation_list2 = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w] 


def publish(event):
    global orientation_list ,orientation_list2 , fix , flooat
    data1=fix
    odomFix1Msg.header.frame_id="map"
    odomFix1Msg.pose.pose.position.x= data1.pose.pose.position.x
    odomFix1Msg.pose.pose.position.y= data1.pose.pose.position.y
    odomFix1Msg.pose.pose.orientation.x= 0.0 #orientation_list[0]
    odomFix1Msg.pose.pose.orientation.y= 0.0 #orientation_list[1]
    odomFix1Msg.pose.pose.orientation.z= orientation_list[2]
    odomFix1Msg.pose.pose.orientation.w= orientation_list[3]
    odomFix1Msg.header.stamp= rospy.Time.now()
    odom1Fix_pub.publish(odomFix1Msg)

    odomFix2Msg.header.frame_id="map"
    odomFix2Msg.pose.pose.position.x= data1.pose.pose.position.x
    odomFix2Msg.pose.pose.position.y= data1.pose.pose.position.y
    odomFix2Msg.pose.pose.orientation.x= 0.0 #orientation_list2[0]
    odomFix2Msg.pose.pose.orientation.y= 0.0 #orientation_list2[1]
    odomFix2Msg.pose.pose.orientation.z= orientation_list2[2]
    odomFix2Msg.pose.pose.orientation.w= orientation_list2[3]
    odomFix2Msg.header.stamp= rospy.Time.now()
    odom2Fix_pub.publish(odomFix2Msg)

    data2=flooat
    odomFloat1Msg.header.frame_id="map"
    odomFloat1Msg.pose.pose.position.x= data2.pose.pose.position.x
    odomFloat1Msg.pose.pose.position.y= data2.pose.pose.position.y
    odomFloat1Msg.pose.pose.orientation.x= 0.0 #orientation_list[0]
    odomFloat1Msg.pose.pose.orientation.y= 0.0 #orientation_list[1]
    odomFloat1Msg.pose.pose.orientation.z= orientation_list[2]
    odomFloat1Msg.pose.pose.orientation.w= orientation_list[3]
    odomFloat1Msg.header.stamp= rospy.Time.now()

    odomFloat2Msg.header.frame_id="map"
    odomFloat2Msg.pose.pose.position.x= data2.pose.pose.position.x
    odomFloat2Msg.pose.pose.position.y= data2.pose.pose.position.y
    odomFloat2Msg.pose.pose.orientation.x= 0.0 #orientation_list2[0]
    odomFloat2Msg.pose.pose.orientation.y= 0.0 #orientation_list2[1]
    odomFloat2Msg.pose.pose.orientation.z= orientation_list2[2]
    odomFloat2Msg.pose.pose.orientation.w= orientation_list2[3]
    odomFloat2Msg.header.stamp= rospy.Time.now()

    odom1Float_pub.publish(odomFloat1Msg)
    odom2Float_pub.publish(odomFloat2Msg)


def listener():
    rospy.Subscriber("/odometry/gps", Odometry, callback_odom)
    rospy.Subscriber("/odometry2/gps", Odometry, callback_odom2)
    rospy.Subscriber("/imu_enu_new", Imu, callback_IMU)
    rospy.Subscriber("/imu2_enu_new", Imu, callback_IMU2)
    rospy.Timer(rospy.Duration(0.5), publish)
    rospy.spin()

if __name__ == '__main__':
    listener()
