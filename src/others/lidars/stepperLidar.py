#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
import serial
import os

full_range=90.0
step=1.8*math.pi/180.0
no_steps=51

rospy.init_node('stepper_lidar_node', anonymous=True)
lidarL_msg=LaserScan()
lidarR_msg=LaserScan()
lidarsL_pub= rospy.Publisher('lidarsL', LaserScan, queue_size=2)
lidarsR_pub= rospy.Publisher('lidarsR', LaserScan, queue_size=2)

command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines=data.split('\n\n')
default_port1='/dev/ttyACM0'
default_port2='/dev/ttyUSB0'

for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'usb-Arduino__www.arduino.cc__0042_95437313534351E0E0F1-if00' in lines[i] and 'dev/ttyACM' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        default_port1 = lines[i][start:start+12]
        print "first Mega found ################ port is :" , default_port1
    elif 'usb-1a86' in lines[i] and 'dev/ttyUSB' in lines[i]:
        start = lines[i].find('/dev/ttyUSB')
        default_port2 = lines[i][start:start+12]
        print "second Mega found ################ port is :" , default_port2

ser1 = serial.Serial(default_port1,115200)
ser1.flushInput()
lineL=[50.0]*52
lidarL_msg.range_min = 0.01
lidarL_msg.range_max = 50.0
lidarL_msg.angle_min = 0.0*math.pi/180.0
lidarL_msg.angle_max = full_range*math.pi/180.0
lidarL_msg.angle_increment = step
lidarL_msg.time_increment = 0.522/51
lidarL_msg.scan_time = 0.522
lidarL_msg.header.frame_id='/lidarL_link'

ser2 = serial.Serial(default_port2,115200)
ser2.flushInput()
lineR=[50.0]*52
lidarR_msg.range_min = 0.01
lidarR_msg.range_max = 50.0
lidarR_msg.angle_min = -full_range*math.pi/180.0
lidarR_msg.angle_max = 0.0*math.pi/180.0
lidarR_msg.angle_increment = step
lidarR_msg.time_increment = 0.522/51
lidarR_msg.scan_time = 0.522
lidarR_msg.header.frame_id='/lidarR_link'

lineLTemp = [50.0]*51
lineRTemp = [50.0]*51

while 1:
    txt2 = ser1.readline().strip()
    txt1 = ser2.readline().strip()
    print "----------" , txt1 , "----------"
    temp1 = txt1.split(',')
    temp2 = txt2.split(',')
    if len(temp1) != 52:
        print " **** ERROR **** , Line1 was ", len(temp1) , temp1
        print txt1
    elif len(temp2) != 52:
        print " **** ERROR **** , Line2 was ", len(temp2) , temp2
        print txt2
    else:
        for i in range(len(temp1)):
            lineL[i] = float(temp1[i])
        lineLeft = lineL[1:]
        if lineL[0]<0.5:
            for i in range(len(lineLeft)):
                lineLTemp[len(lineLeft)-i-1] = lineLeft[i]
            lineLeft=lineLTemp


        lidarL_msg.ranges = lineLeft
        print("L",lidarL_msg.ranges )
        lidarL_msg.header.stamp=rospy.Time.now()
        lidarsL_pub.publish(lidarL_msg)  

        for i in range(len(temp2)):
			lineR[i] = float(temp2[i])
        lineRight = lineR[1:]
        if lineR[0]<0.5:
            for i in range(len(lineRight)):
                lineRTemp[len(lineRight)-i-1] = lineRight[i]
            lineRight=lineRTemp
        lidarR_msg.ranges = lineRight
        print("R" , lidarR_msg.ranges )
        lidarR_msg.header.stamp=rospy.Time.now()
        lidarsR_pub.publish(lidarR_msg)

