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
rospy.init_node('debug_odom', anonymous=False)  

st_x =0.0 ; st_y =0.0 ; nd_x =0.0 ; nd_y =0.0 ; rd_x =0.0 ; rd_y=0.0
st_vx=0.0 ; st_vy =0.0 ; st_vth =0.0 ; nd_vx =0.0 ; nd_vy =0.0 ; nd_vth =0.0 ; rd_vx =0.0 ; rd_vy =0.0 ; rd_vth=0.0
yaw =0.0 ; st_yaw =0.0 ; nd_yaw =0.0 ; rd_yaw=0.0 
print "time ,st_x , st_y , nd_x , nd_y , rd_x , rd_y ,st_vx, st_vy ,st_vth, nd_vx , nd_vy ,nd_vth , rd_vx , rd_vy ,rd_vth , yaw , st_yaw , nd_yaw , rd_yaw"

def callback_IMU(data): 
    global yaw
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw=yaw*180.0/math.pi    

######################################################################

def st_callback_Odom(data):
    global st_x , st_y , st_yaw
    st_x=data.pose.pose.position.x
    st_y=data.pose.pose.position.y
    st_orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
    (stroll, stpitch, st_yaw) = euler_from_quaternion (st_orientation_list)
    st_yaw=st_yaw*180.0/math.pi    

def st_callback_wheel(data): 
    global st_vx , st_vy ,st_vth
    st_vx=data.twist.twist.linear.x
    st_vy=data.twist.twist.linear.y
    st_vth=data.twist.twist.angular.z

######################################################################

def nd_callback_Odom(data):
    global nd_x , nd_y , nd_yaw
    nd_x=data.pose.pose.position.x
    nd_y=data.pose.pose.position.y
    nd_orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
    (nd_roll, nd_pitch, nd_yaw) = euler_from_quaternion (nd_orientation_list)
    nd_yaw=nd_yaw*180.0/math.pi    
    
def nd_callback_wheel(data): 
    global nd_vx , nd_vy ,nd_vth
    nd_vx=data.twist.twist.linear.x
    nd_vy=data.twist.twist.linear.y
    nd_vth=data.twist.twist.angular.z

######################################################################

def rd_callback_Odom(data):
    global rd_x , rd_y , rd_yaw
    rd_x=data.pose.pose.position.x
    rd_y=data.pose.pose.position.y
    rd_orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
    (rd_roll, rd_pitch, rd_yaw) = euler_from_quaternion (rd_orientation_list)
    rd_yaw=rd_yaw*180.0/math.pi    
    
def rd_callback_wheel(data): 
    global rd_vx , rd_vy ,rd_vth
    rd_vx=data.twist.twist.linear.x
    rd_vy=data.twist.twist.linear.y
    rd_vth=data.twist.twist.angular.z

######################################################################

def callback(event): 
    global st_x , st_y , nd_x , nd_y , rd_x , rd_y 
    global st_vx, st_vy , st_vth , nd_vx , nd_vy , nd_vth , rd_vx , rd_vy , rd_vth
    global yaw , st_yaw , nd_yaw , rd_yaw 
    print time.time() ,"," , st_x ,",", st_y ,",", nd_x ,",", nd_y ,",", rd_x ,",", rd_y ,",",st_vx,",", st_vy ,",", st_vth ,",", nd_vx ,",", nd_vy ,",",nd_vth ,",", rd_vx ,",", rd_vy ,",",rd_vth ,",", yaw ,",", st_yaw ,",", nd_yaw ,",", rd_yaw



def listener():
    rospy.Subscriber("/imu_enu_new", Imu, callback_IMU)
    rospy.Subscriber("/odom", Odometry, st_callback_Odom)
    rospy.Subscriber("/wheel_odometry_new", Odometry, st_callback_wheel)

    rospy.Subscriber("/ndodom", Odometry, nd_callback_Odom)
    rospy.Subscriber("/ndwheel_odometry_new", Odometry, nd_callback_wheel)

    rospy.Subscriber("/rdodom", Odometry, rd_callback_Odom)
    rospy.Subscriber("/rdwheel_odometry_new", Odometry, rd_callback_wheel)

    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
