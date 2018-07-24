#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
import math , time
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
quat=Quaternion()
imuMsg=Imu()

angle =0.0
num_y=0.0
y2 = 0.0
y_all = 0.0

rospy.init_node('fix_imu', anonymous=False)  
imu_pub = rospy.Publisher('/imu/data2', Imu, queue_size=10)

def callback_IMU(data): 
	yaw=-180.0
	quat = quaternion_from_euler (0.0, 0.0, yaw*math.pi/180.0)
	imuMsg.orientation.x = quat[0]
	imuMsg.orientation.y = quat[1]
	imuMsg.orientation.z = quat[2]
	imuMsg.orientation.w = quat[3]
	imuMsg.header.stamp=rospy.Time.now()
	imu_pub.publish(imuMsg)
	print yaw ,quat








def listener():
    rospy.Subscriber("/imu2_enu", Imu, callback_IMU)
    rospy.spin()

if __name__ == '__main__':
    listener()
