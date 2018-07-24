#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
import math , time
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
quat=Quaternion()

angle =0.0
num_y=0.0
y_all = 0.0
rospy.init_node('fix_imu', anonymous=False)  
imu_pub = rospy.Publisher('/imu_enu_new', Imu, queue_size=10)
imuMsg = Imu() 

def callback_IMU1(data): 
    global num_y, y_all , quat , imuMsg
    imuMsg = data
    imuMsg.header.stamp=rospy.Time.now()
    imu_pub.publish(imuMsg)

def callback_IMU(data): 
    global num_y, y_all , quat , imuMsg
    imuMsg = data
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw=yaw*180.0/math.pi


    if ( yaw >=    5.913 and yaw <54.175):
        new_yaw=translate(yaw, 5.913, 54.175, 8.3, 53.3)    # 48.262
        case = 1

    elif ((yaw >= -41.385 and yaw <=0) or (yaw <5.913 and yaw > 0) ): #48.083
        n_yaw=yaw+180.0        
        new_yaw=translate(n_yaw, -41.385+180.0, 5.913+180.0, -36.7+180.0, 8.3+180.0) 
        new_yaw=new_yaw-180.0        
        case = 6

    elif ( yaw >=    -83.747 and yaw <-41.385 ): #done
        new_yaw=translate(yaw, -83.747, -41.385, -80.1, -36.7) #42.362
        case = 3

    elif ( yaw >=    -120.008 and yaw <-83.747 ): #done
        new_yaw=translate(yaw, -120.008, -83.747, -125.1, -80.1) # 36.261
        case = 3

    elif ( yaw >=-155.44     and yaw <-120.008 ): #done
        new_yaw=translate(yaw, -155.44, -120.008, -171.7, -125.1) # 35.432
        case = 3

    elif ( yaw >= 156.533 or yaw < -155.44 ):  # 48.027
        if yaw<0:
            n_yaw=yaw+360
        else:
            n_yaw=yaw  # 152.82 180 , 180 , (-180 > -152.8 >> 180 >> 127  >> 360 >> 233

        new_yaw=translate(n_yaw, 156.533, -155.44+360.0, 143.3, -171.7+360.0) #done
        if new_yaw>180.0:
            new_yaw=new_yaw-360.0
        case = 2

    elif ( yaw >=    101.8 and yaw <156.533 ):  #54.733
        new_yaw=translate(yaw, 101.8, 156.533, 99.9, 143.3)    
        case = 1

    elif ( yaw >=    54.175 and yaw <101.8 ):  #47.625
        new_yaw=translate(yaw, 54.175, 101.8, 53.3 ,99.9)    
        case = 1
        
            
    else:
        print "there is something wrong , yaw val : ", yaw
        case =  8

#    print case, "yaw" , yaw , "new_yaw" , new_yaw
    new_yaw= new_yaw*math.pi/180.0        
    quat = quaternion_from_euler (roll, pitch, new_yaw)
    imuMsg.orientation.x = quat[0]
    imuMsg.orientation.y = quat[1]
    imuMsg.orientation.z = quat[2]
    imuMsg.orientation.w = quat[3]
    #imuMsg.angular_velocity.x = case
    imuMsg.header.stamp=rospy.Time.now()
    imuMsg.orientation_covariance=[0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
    imu_pub.publish(imuMsg)

#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/imu_enu", Imu, callback_IMU)
    rospy.spin()



def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()
