#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import math , time

rospy.init_node('joint_listener', anonymous=True)   
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
jointMsg = JointState()

jointMsg.position=[4.3242536129639575e-07, 0.0, 3.4489326505848794e-07, 0.0, -0.0333112856467368, -0.0883257707479045, -0.007578545363969624, -0.088221149575849, 0.0013622470335077352, 0.020484959856338136, -0.0887804693970572, 2.328813065725541, -0.05453898807210931, -0.0013771270984719308]
jointMsg.velocity=[-3.153277000313842e-06, 0.0, -3.9280788931318755e-06, 0.0, 0.00016852803173235398, 0.00431761694750128, -1.597526668933023e-05, 0.0010291882184467322, 0.227353814351, -4.4508515387956665e-05, 0.00263168383571161, -0.0056027830581943765, -0.002682227694243383, 0.10291047193562543]
jointMsg.effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 50.0]
jointMsg.name=['elp_left_pan_joint', 'elp_left_tilt_joint', 'elp_right_pan_joint', 'elp_right_tilt_joint', 'left_front_axle', 'left_front_shock', 'left_rear_axle', 'left_rear_shock', 'left_steering_joint', 'right_front_axle', 'right_front_shock', 'right_rear_axle', 'right_rear_shock', 'right_steering_joint']

d1 =0.0
d = 1.65 #RBCAR_D_WHEELS_M
zero=False

def callback_joint(data):
    global jointMsg
    jointMsg=data
    joint_pub.publish(jointMsg)

def listener():
    rospy.Subscriber("/rbcar/joint_states", JointState, callback_joint)
    rospy.spin()

if __name__ == '__main__':
    listener()
