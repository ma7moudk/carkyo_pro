#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('wheel_listener', anonymous=True)   
stodom_pub = rospy.Publisher('/wheel_odometry_new', Odometry, queue_size=10)
ndodom_pub = rospy.Publisher('/ndwheel_odometry_new', Odometry, queue_size=10)
rdodom_pub = rospy.Publisher('/rdwheel_odometry_new', Odometry, queue_size=10)
stodomMsg = Odometry()
ndodomMsg = Odometry()
rdodomMsg = Odometry()
stodomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]
stodomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ndodomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]
ndodomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
rdodomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]
rdodomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def callback_odom(data): 
    LenghtBetweenTwoWheels=1.01
    wheelBase=1.68
    vR=data.twist.twist.linear.x
    vL=data.twist.twist.linear.y
    wheelAngle=data.twist.twist.linear.z
    V_mps=(vR+vL)/2.0

    stodomMsg.twist.twist.linear.x=V_mps * math.cos(wheelAngle)
    stodomMsg.twist.twist.linear.y=V_mps * math.sin(wheelAngle)
    stodomMsg.twist.twist.angular.z=0.0
    stodomMsg.header.stamp= rospy.Time.now()

    ndodomMsg.twist.twist.linear.x=V_mps
    ndodomMsg.twist.twist.linear.y=0.0
    ndodomMsg.twist.twist.angular.z= (vR - vL)/ LenghtBetweenTwoWheels
    ndodomMsg.header.stamp= rospy.Time.now()

    rdodomMsg.twist.twist.linear.x=V_mps
    rdodomMsg.twist.twist.linear.y=0.0
    rdodomMsg.twist.twist.angular.x=data.twist.twist.angular.x
    rdodomMsg.twist.twist.angular.z=V_mps*math.tan(wheelAngle)/ wheelBase
    rdodomMsg.header.stamp= rospy.Time.now()

    stodom_pub.publish(stodomMsg)
    ndodom_pub.publish(ndodomMsg)
    rdodom_pub.publish(rdodomMsg)


def listener():
    rospy.Subscriber("/wheel_odometry", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()
