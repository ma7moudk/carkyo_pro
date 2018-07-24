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

angle =0.0
num_y=0.0
y_all = 0.0
rospy.init_node('fix_imu', anonymous=False)  
imu_pub = rospy.Publisher('/imu2_enu_new', Imu, queue_size=10)
imuMsg = Imu() 

def callback_IMU1(data): 
    global num_y, y_all , quat , imuMsg
    imuMsg = data
    imuMsg.header.stamp=rospy.Time.now()
    imu_pub.publish(imuMsg)

def callback_IMU(event): 
    global num_y, y_all , quat , imuMsg
    imuMsg.orientation.x = 0
    imuMsg.orientation.y = 0.0
    imuMsg.orientation.z = 0.0
    imuMsg.orientation.w = 1.0
    #imuMsg.angular_velocity.x = case
    imuMsg.header.stamp=rospy.Time.now()
    imuMsg.orientation_covariance=[0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
    imu_pub.publish(imuMsg)

#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
 #   rospy.Subscriber("/imu_enu", Imu, callback_IMU)
    rospy.Timer(rospy.Duration(0.1), callback_IMU)
    rospy.spin()



def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()
