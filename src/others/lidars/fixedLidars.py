#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
import serial
import os


rospy.init_node('fixed_lidar_node', anonymous=True)
lidarL1_msg=LaserScan()
lidarR1_msg=LaserScan()
lidarL2_msg=LaserScan()
lidarR2_msg=LaserScan()
lidarL1_pub= rospy.Publisher('lidarL1', LaserScan, queue_size=2)
lidarR1_pub= rospy.Publisher('lidarR1', LaserScan, queue_size=2)
lidarL2_pub= rospy.Publisher('lidarL2', LaserScan, queue_size=2)
lidarR2_pub= rospy.Publisher('lidarR2', LaserScan, queue_size=2)

command ='udevadm info -e'
p=os.popen(command,"r")

data = p.read()
lines=data.split('\n\n')
default_port='/dev/ttyACM0'
for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'usb-1a86' in lines[i] and 'dev/ttyUSB' in lines[i]:
        start = lines[i].find('/dev/ttyUSB')
        default_port = lines[i][start:start+12]
        print "############ Mega found ################ port is :" , default_port

ser = serial.Serial(default_port,115200)
ser.flushInput()
step=1

lineL1=[30.0]*3
lineR1=[30.0]*3
lineL2=[30.0]*3
lineR2=[30.0]*3
lidarL1_msg.range_min = 0.01
lidarL1_msg.range_max = 30.0
lidarR1_msg.range_min = 0.01
lidarR1_msg.range_max = 30.0
lidarL2_msg.range_min = 0.01
lidarL2_msg.range_max = 30.0
lidarR2_msg.range_min = 0.01
lidarR2_msg.range_max = 30.0

while 1:
    txt = ser.readline().strip()
    print txt
    temp = txt.split(',')
    if len(temp) != 2:
        print " **** ERROR **** , Line was"
        print txt
        print " *********** ERROR ***********"

    elif "WatchDog" in txt:
        print "watchdog reset"

    elif temp[0] == "L1":
        lineL1[1] =float(temp[1])

        lidarL1_msg.angle_min = -1.0*math.pi/180.0
        lidarL1_msg.angle_max = 1.0*math.pi/180.0
        lidarL1_msg.angle_increment = step*math.pi/180.0
        lidarL1_msg.time_increment = 0.03
        lidarL1_msg.scan_time = 0.1
        lidarL1_msg.ranges = lineL1
        print(lidarL1_msg.ranges )
        lidarL1_msg.header.frame_id='lidarL1_link'
        lidarL1_msg.header.stamp=rospy.Time.now()
        lidarL1_pub.publish(lidarL1_msg)  


    elif temp[0] == "R1":
        lineR1[1] = float(temp[1])

        lidarR1_msg.angle_min = -1.0*math.pi/180.0
        lidarR1_msg.angle_max = 1.0*math.pi/180.0
        lidarR1_msg.angle_increment = step*math.pi/180.0
        lidarR1_msg.time_increment = 0.03
        lidarR1_msg.scan_time = 0.1
        lidarR1_msg.ranges = lineR1
        print(lidarR1_msg.ranges )
        lidarR1_msg.header.frame_id='lidarR1_link'
        lidarR1_msg.header.stamp=rospy.Time.now()
        lidarR1_pub.publish(lidarR1_msg)   

    elif temp[0] == "L2":
        lineL2[1] = float(temp[1])

        lidarL2_msg.angle_min = -1.0*math.pi/180.0
        lidarL2_msg.angle_max = 1.0*math.pi/180.0
        lidarL2_msg.angle_increment = step*math.pi/180.0
        lidarL2_msg.time_increment = 0.03
        lidarL2_msg.scan_time = 0.1
        lidarL2_msg.ranges = lineL2
        print(lidarL2_msg.ranges )
        lidarL2_msg.header.frame_id='lidarL2_link'
        lidarL2_msg.header.stamp=rospy.Time.now()
        lidarL2_pub.publish(lidarL2_msg)  

    elif temp[0] == "R2":
        lineR2[1] = float(temp[1])

        lidarR2_msg.angle_min = -1.0*math.pi/180.0
        lidarR2_msg.angle_max = 1.0*math.pi/180.0
        lidarR2_msg.angle_increment = step*math.pi/180.0
        lidarR2_msg.time_increment = 0.03
        lidarR2_msg.scan_time = 0.1
        lidarR2_msg.ranges = lineR2
        print(lidarR2_msg.ranges )
        lidarR2_msg.header.frame_id='lidarR2_link'
        lidarR2_msg.header.stamp=rospy.Time.now()
        lidarR2_pub.publish(lidarR2_msg) 

