#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
import serial
import os

full_range = 90.0
step = 1.8*math.pi/180.0
no_steps = 51

rospy.init_node('left_lidar_node', anonymous=True)
lidarL_msg=LaserScan()
lidarsL_pub= rospy.Publisher('lidarsL', LaserScan, queue_size=2)

command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines=data.split('\n\n')
default_port2='/dev/ttyUSB0'

for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'usb-1a86' in lines[i] and 'dev/ttyUSB' in lines[i]:
        start = lines[i].find('/dev/ttyUSB')
        default_port2 = lines[i][start:start+12]
        print "second Mega found ################ port is :" , default_port2

ser2 = serial.Serial(default_port2,115200)
ser2.flushInput()
lineL=[50.0]*52
lidarL_msg.range_min = 0.01
lidarL_msg.range_max = 50.0
lidarL_msg.angle_min = 0.0*math.pi/180.0
lidarL_msg.angle_max = full_range*math.pi/180.0
lidarL_msg.angle_increment = step
lidarL_msg.time_increment = 0.522/51
lidarL_msg.scan_time = 0.522
lidarL_msg.header.frame_id='/lidarL_link'


lineLTemp = [50.0]*51
oldLineLeft=[50.0]*51
oldLineLeft2=[50.0]*51
newLineLeft=[50.0]*51

while 1:
    txt2 = ser2.readline().strip()
    temp2 = txt2.split(',')
    if len(temp2) != 52:
        print " **** ERROR **** , Line2 was ", len(temp2) , temp2
        print txt2
    else:
        for i in range(len(temp2)):
            lineL[i] = float(temp2[i])
        lineLeft = lineL[1:]
        if lineL[0]<0.5:
            for i in range(len(lineLeft)):
                lineLTemp[len(lineLeft)-i-1] = lineLeft[i]
            lineLeft=lineLTemp

#        for i in range(1,len(lineLeft)-1):
#            if abs((lineLeft[i] - lineLeft[i+1])) <= 1.0  or abs((lineLeft[i] - lineLeft[i-1])) <= 1.0 or  abs(lineLeft[i] -oldLineLeft[i]) <=1.0 or  abs(lineLeft[i] -oldLineLeft2[i]) <=1.0:
#                newLineLeft[i]=lineLeft[i]
#            else:
#                newLineLeft[i]=200.0


        for i in range(len(lineLeft)-1):
            if abs((lineLeft[i] - lineLeft[i+1])) <= 1.0 or  abs(lineLeft[i] -oldLineLeft[i]) <=1.0 :
                newLineLeft[i]=lineLeft[i]
            else:
                newLineLeft[i]=200.0


        oldLineLeft2=oldLineLeft
        oldLineLeft=lineLeft
        lidarL_msg.ranges = newLineLeft
        print("L",lidarL_msg.ranges )
#        print "oldLineLeft" , oldLineLeft
#        print "lineLeft" , lineLeft
#        print "newLineLeft" , newLineLeft
        lidarL_msg.header.stamp=rospy.Time.now()
        lidarsL_pub.publish(lidarL_msg)  


