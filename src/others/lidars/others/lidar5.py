#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node('thlidar5_node', anonymous=True)   
lidar_pub = rospy.Publisher('/lidar5', LaserScan, queue_size=2)
lidarMsg = LaserScan()


def callback_lidar(data):
    lidarMsg = data
    lidarMsg.header.stamp = rospy.Time.now()
    lidar_pub.publish(lidarMsg)

def listener():
    rospy.Subscriber("/lidarPi5", LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    listener()
