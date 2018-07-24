#!/usr/bin/env python
import math , time
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
quat=Quaternion()


rospy.init_node('enu_listener', anonymous=False)
deg=-30
rad = deg*math.pi/180.0  
print deg , quaternion_from_euler(0,0,rad) 
