#!/usr/bin/env python
import rospy,math
import math , time
import serial , os
import thread

from serial.tools import list_ports
print list(list_ports.comports())

print "1"
command ='udevadm info -e'
p=os.popen(command,"r")
print "2"
data = p.read()
lines=data.split('\n\n')
default_port='/dev/ttyACM0'
print "3"
for i in range(len(lines)):
    lines[i].replace("P","\nP")
 #   print lines[i]
  #  print "--------------------------"
    if 'Teensyduino' in lines[i] and 'dev/ttyUSB' in lines[i]:
        print "Found"
        print lines[i]
        start = lines[i].find('/dev/ttyACM')
        default_port = lines[i][start:start+12]
        print "############ Mega found ################ port is :" , default_port
print "4"

ser = serial.Serial(default_port)
#ser.flushInput()
print ser.isOpen()
print "5"
while 1:
    print "6"
    print ser.isOpen()
    txt = ser.readline()
    print "7"
    print "8"
    print txt





def read():
    print "10"
    with open("/dev/ttyACM0","r") as readBuff:
        print "11"
        while True :
            print "12"
            print readBuff.read()
            print "13"
            time.sleep(0.5)

print "8"
#$thread.start_new_thread(read()) #,(None,))
print "9"
