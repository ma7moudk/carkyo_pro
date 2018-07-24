#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan


rospy.init_node('lid_node', anonymous=True)
lidarL_msg=LaserScan()
lidars_pub= rospy.Publisher('lidarsL', LaserScan, queue_size=2)

def callback_lid(data):
    lidarL_msg=data
    lidarL_msg.header.frame_id='base_link'
    lidarL_msg.header.stamp=rospy.Time.now()
    r=[1.0]*51
    lidarL_msg.ranges=r
    lidars_pub.publish(lidarL_msg)
    print "v"

def listener():
    rospy.Subscriber("/lidars", LaserScan, callback_lid)
    rospy.spin()

if __name__ == '__main__':
    listener()



