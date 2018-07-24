#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
import serial
import os


rospy.init_node('lidar_node', anonymous=True)
lidar_msg=LaserScan()
lidar_pub= rospy.Publisher('lidarC', LaserScan, queue_size=2)

command ='udevadm info -e'
p=os.popen(command,"r")

data = p.read()
lines=data.split('\n\n')
default_port='/dev/ttyACM0'
for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'arduino' in lines[i] and 'dev/ttyACM' in lines[i]:
        print "Arduino has been found"
        start = lines[i].find('/dev/ttyACM')
        default_port = lines[i][start:start+12]

ser = serial.Serial(default_port,115200)
ser.flushInput()
step=5
line1=[]
line2=[]
lidar_msg.range_min = 0.01
lidar_msg.range_max = 30.0  
while 1:
	txt = ser.readline().strip()
	temp = txt.split(',')
	if len(temp) != 18:
		print " **** ERROR new****"
		print txt

	elif temp[0] == "C":
		line1 = temp[1:-1] # (len(temp)-1)]
		for i in range(len(line1)):
			line1[i] =float(line1[i])

		lidar_msg.angle_min = -40.0*math.pi/180.0
		lidar_msg.angle_max = 40.0*math.pi/180.0
		lidar_msg.angle_increment = step*math.pi/180.0
		lidar_msg.time_increment = 0.058
		lidar_msg.scan_time = 1.0
		lidar_msg.ranges = line1
		print(lidar_msg.ranges )
		lidar_msg.header.frame_id='hokuyo5_link' #'lidarC_link'
		lidar_msg.header.stamp=rospy.Time.now()
		lidar_pub.publish(lidar_msg)  
