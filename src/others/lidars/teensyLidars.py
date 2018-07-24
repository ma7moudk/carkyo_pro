#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
import serial
import os

full_range = 90.0
step = 1.8*math.pi/180.0
no_steps = 51

rospy.init_node('teensy_lidars_node', anonymous=True)
lidarL_msg=LaserScan()
lidarR_msg=LaserScan()
lidarsL_pub= rospy.Publisher('lidarsL', LaserScan, queue_size=2)
lidarsR_pub= rospy.Publisher('lidarsR', LaserScan, queue_size=2)

command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines=data.split('\n\n')
default_port='/dev/ttyACM0'

for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if '4527060-if00' in lines[i] and 'dev/ttyACM' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        default_port = lines[i][start:start+12]
        print "Teensy Found :" , default_port

ser = serial.Serial(default_port,115200)
ser.flushInput()

lineL=[97.0]*52
lineLeft=[97.0]*50
newLineLeft=[97.0]*50
oldLineLeft=[97.0]*50
oldLineLeft2=[97.0]*50
lineLTemp = [97.0]*50

lidarL_msg.range_min = 0.1
lidarL_msg.range_max = 41.0
lidarL_msg.angle_min = 0.0*math.pi/180.0
lidarL_msg.angle_max = full_range*math.pi/180.0
lidarL_msg.angle_increment = step
lidarL_msg.time_increment = 0.664/51
lidarL_msg.scan_time = 0.664
lidarL_msg.header.frame_id='/lidarL_link'

lineR=[97.0]*52
lineRight=[97.0]*50
newLineRight=[97.0]*50
oldLineRight=[97.0]*50
oldLineRight2=[97.0]*50
lineRTemp = [97.0]*50

lidarR_msg.range_min = 0.01
lidarR_msg.range_max = 41.0
lidarR_msg.angle_min = -full_range*math.pi/180.0
lidarR_msg.angle_max = 0.0*math.pi/180.0
lidarR_msg.angle_increment = step
lidarR_msg.time_increment = 0.664/51
lidarR_msg.scan_time = 0.664
lidarR_msg.header.frame_id='/lidarR_link'

while 1:
    txt = ser.readline().strip()
    temp = txt.split(',')
    if len(temp) != 53:
        print " **** ERROR **** , Line2 was ", len(temp) , temp
        print txt
    else:
        if temp[0] == 'R':
            for i in range(1,len(temp)-1):
                lineR[i-1] = float(temp[i])    # len(lineR) = 52
            lineRight = lineR[1:len(lineR)-1]  # len(lineRight) = 50
            #print "*************************"
            #print "Length",len(lineR)," - LineR : ",lineR
            #print "Length",len(lineRight)," - LineRight : ",lineRight
            #print "*************************"


            if lineR[0]>0.5:
                for i in range(len(lineRight)):
                    lineRTemp[len(lineRight)-i-1] = lineRight[i]
                lineRight=lineRTemp



            for i in range(len(lineRight)-1):
                if abs((lineRight[i] - lineRight[i+1])) <= 1.0 or  abs(lineRight[i] -oldLineRight[i]) <=1.0 :
                    newLineRight[i]=lineRight[i]                
                else:
                    newLineRight[i]=35.0
            newLineRight[49] = lineRight[49]


            lidarR_msg.ranges = newLineRight
            print "Right : --",lineRight
            lidarR_msg.header.stamp=rospy.Time.now()
            lidarsR_pub.publish(lidarR_msg)
            oldLineRight2 = oldLineRight
            oldLineRight  = lineRight

        elif temp[0] == 'L':
            for i in range(1,len(temp)-1):
                lineL[i-1] = float(temp[i])
            lineLeft = lineL[1:len(lineL)-1]
            if lineL[0]>0.5:
                for i in range(len(lineLeft)):
                    lineLTemp[len(lineLeft)-i-1] = lineLeft[i]
                lineLeft=lineLTemp


            for i in range(len(lineLeft)-1):
                if abs((lineLeft[i] - lineLeft[i+1])) <= 1.0 or  abs(lineLeft[i] -oldLineLeft[i]) <=1.0 :
                    newLineLeft[i]=lineLeft[i]
                else:
                    newLineLeft[i]=35.0
            newLineLeft[49]=lineLeft[49]


            lidarL_msg.ranges = newLineLeft
            print "Left : --",lineLeft        
            lidarL_msg.header.stamp=rospy.Time.now()
            lidarsL_pub.publish(lidarL_msg)
            oldLineLeft2  = oldLineLeft
            oldLineLeft   = lineLeft


