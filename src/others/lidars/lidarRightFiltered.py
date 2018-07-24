#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
import serial
import os

full_range=90.0
step=1.8*math.pi/180.0
no_steps=51

rospy.init_node('right_lidar_node', anonymous=True)
lidarR_msg=LaserScan()
lidarsR_pub= rospy.Publisher('lidarsR', LaserScan, queue_size=2)

command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines=data.split('\n\n')
default_port1='/dev/ttyACM0'

for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'usb-Arduino__www.arduino.cc__0042_95437313534351E0E0F1-if00' in lines[i] and 'dev/ttyACM' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        default_port1 = lines[i][start:start+12]
        print "first Mega found ################ port is :" , default_port1

ser1 = serial.Serial(default_port1,115200)
ser1.flushInput()
lineR=[50.0]*52
lidarR_msg.range_min = 0.01
lidarR_msg.range_max = 50.0
lidarR_msg.angle_min = -full_range*math.pi/180.0
lidarR_msg.angle_max = 0.0*math.pi/180.0
lidarR_msg.angle_increment = step
lidarR_msg.time_increment = 0.522/51
lidarR_msg.scan_time = 0.522
lidarR_msg.header.frame_id='/lidarR_link'

lineRTemp = [50.0]*51
oldLineRight=[50.0]*51
oldLineRight2=[50.0]*51
newLineRight=[50.0]*51

while 1:
    txt1 = ser1.readline().strip()
    temp1 = txt1.split(',')
    if len(temp1) != 52:
        print " **** ERROR **** , Line1 was ", len(temp1) , temp1
        print txt1
    else:
        for i in range(len(temp1)):
		    lineR[i] = float(temp1[i])
        lineRight = lineR[1:]
        if lineR[0]<0.5:
            for i in range(len(lineRight)):
                lineRTemp[len(lineRight)-i-1] = lineRight[i]
            lineRight=lineRTemp

#        for i in range(1,len(lineRight)-1):
#            if abs((lineRight[i] - lineRight[i+1])) <= 1.0  or abs((lineRight[i] - lineRight[i-1])) <= 1.0 or  abs(lineRight[i] -oldLineRight[i]) <=1.0 or  abs(lineRight[i] -oldLineRight2[i]) <=1.0:
#                newLineRight[i]=lineRight[i]
#            else:
#                newLineRight[i]=200.0

        for i in range(len(lineRight)-1):
            if abs((lineRight[i] - lineRight[i+1])) <= 1.0  or  abs(lineRight[i] -oldLineRight[i]) <=1.0 :
                newLineRight[i]=lineRight[i]
            else:
                newLineRight[i]=200.0

        oldLineRight2=oldLineRight
        oldLineRight=lineRight
        lidarR_msg.ranges = newLineRight
        print("R" , lidarR_msg.ranges )
        lidarR_msg.header.stamp=rospy.Time.now()
        lidarsR_pub.publish(lidarR_msg)
