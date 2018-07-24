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
rospy.init_node('log_fix_imu', anonymous=False)  
imu_pub = rospy.Publisher('/1imu_enu_new', Imu, queue_size=10)
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

    if ( yaw >=    62.29 and yaw <120.5 ):  #done
        new_yaw=translate(yaw, 62.29, 120.5, 54.6, 99.3)    
        case = 1
        
    elif ( yaw >=    120.5 and yaw <163.1 ):  #done
        new_yaw=translate(yaw, 120.5, 163.1, 99.3, 143.3)    
        case = 1

    elif ( yaw >= 163.1 or yaw < -121.73 ):
        if yaw<0:
            n_yaw=yaw+360
        else:
            n_yaw=yaw  # 152.82 180 , 180 , (-180 > -152.8 >> 180 >> 127  >> 360 >> 233

        new_yaw=translate(n_yaw, 163.1, -121.73+360.0, 143.3, -125.1+360.0) #done
        if new_yaw>180.0:
            new_yaw=new_yaw-360.0
        case = 2
                    
    elif ( yaw >=    -121.73 and yaw <-80.72 ): #done
        new_yaw=translate(yaw, -121.73, -80.72, -125.1, -80.7)
        case = 3
            
    elif ( yaw >=   -80.72 and yaw <-42.17 ): #done
        new_yaw=translate(yaw, -80.72, -42.17, -80.7, -36.7) ##
        case = 4
            
                
    elif ((yaw >= -42.17 and yaw <=0) or (yaw <62.29 and yaw > 0) ): #done
        n_yaw=yaw+180.0        
        new_yaw=translate(n_yaw, -42.17+180.0, 62.29+180.0, -36.7+180.0, 54.6+180.0)
        new_yaw=new_yaw-180.0        
        case = 6
        
            
    else:
        print "there is something wrong , yaw val : ", yaw
        case =  8

    
    print "imu1," , time.time() ,"," , case, ",yaw," , yaw , ",new_yaw," , new_yaw 
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
