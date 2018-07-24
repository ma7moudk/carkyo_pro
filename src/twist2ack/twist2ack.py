#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

last_v=0.0
def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id, lastSpeed
  global pub , last_v

  v = data.linear.x  
  if ( (v >0.001 and last_v < -0.001) or ( v <-0.001 and last_v > 0.001) ):
    v=0.0
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = data.angular.z #steering
  msg.drive.speed = v
  last_v=v
  
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/rbcar_robot_control/command')
    wheelbase = rospy.get_param('~wheelbase', 1.68)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
