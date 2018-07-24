#!/usr/bin/env python
import rospy, sys, tf
import tf2_ros
import geometry_msgs.msg  
rospy.init_node('static_tf')


def transform(name,x,y,z,a,b,c):
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
    print (L1_tf.header.stamp,name)



def trans(event):
    transform("lidar1_link", 1.0, 0.24 , 0.56 ,0 ,0 ,0)
    transform("lidar2_link", 1.0,-0.24 , 0.56 ,0 ,0 ,-0.785398163)
    transform("lidar3_link",0.84, 0.47 , 0.56 ,0 ,0 ,-0.785398163)
    transform("lidar4_link",0.84, -0.47, 0.56 ,0 ,0 ,0)
    transform("lidar5_link", 1.0, 0.0  , 0.65 ,0 ,0 ,0)
    transform("imu_link"   , 1.0, -0.5, 1.065, 0, 0, 0)
    transform("gps_link"   , 0.3,  0  , 1.495, 0, 0, 0)


if __name__ == '__main__':
    rospy.Timer(rospy.Duration(2), trans)
    rospy.spin()
