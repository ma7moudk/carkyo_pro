#!/usr/bin/env python
import rospy, sys, tf
import tf2_ros
import geometry_msgs.msg  
rospy.init_node('static_tf')


def transform10(name,x,y,z,a,b,c):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    L1_tf = geometry_msgs.msg.TransformStamped()
    L1_tf.header.stamp = rospy.Time.now()
    L1_tf.header.frame_id = "base_footprint"
    L1_tf.child_frame_id = name 
    L1_tf.transform.translation.x = float(x)
    L1_tf.transform.translation.y = float(y)
    L1_tf.transform.translation.z = float(z)  
    quat = tf.transformations.quaternion_from_euler(float(a),float(b),float(c))
    L1_tf.transform.rotation.x = quat[0]
    L1_tf.transform.rotation.y = quat[1]
    L1_tf.transform.rotation.z = quat[2]
    L1_tf.transform.rotation.w = quat[3]
    broadcaster.sendTransform(L1_tf)

def transform20(name,x,y,z,a,b,c):
    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    L2_tf = geometry_msgs.msg.TransformStamped()
    L2_tf.header.stamp = rospy.Time.now()
    L2_tf.header.frame_id = "base_footprint"
    L2_tf.child_frame_id = name 
    L2_tf.transform.translation.x = float(x)
    L2_tf.transform.translation.y = float(y)
    L2_tf.transform.translation.z = float(z)  
    quat = tf.transformations.quaternion_from_euler(float(a),float(b),float(c))
    L2_tf.transform.rotation.x = quat[0]
    L2_tf.transform.rotation.y = quat[1]
    L2_tf.transform.rotation.z = quat[2]
    L2_tf.transform.rotation.w = quat[3]
    broadcaster2.sendTransform(L2_tf)

def trans():
 #   transform("lidar1_link", 1.0, 0.24 , 0.56 ,0 ,0 ,0)
 #   transform("lidar2_link", 1.0,-0.24 , 0.56 ,0 ,0 ,-0.785398163)
 #   transform("lidar3_link",0.84, 0.47 , 0.56 ,0 ,0 ,-0.785398163)
 #   transform("lidar4_link",0.84, -0.47, 0.56 ,0 ,0 ,0)
 #   transform("lidar5_link", 1.0, 0.0  , 0.65 ,0 ,0 ,0)
 
#   transform10("imu_link"   , 1.0, -0.5, 1.065, 0, 0, 0)
    transform20("gp_link"   , 0.3,  0  , 1.495, 0, 0, 0)




if __name__ == '__main__':
    trans()
    rospy.spin()
