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
rospy.init_node('log_fix_imu2', anonymous=False)  
imu_pub = rospy.Publisher('/1imu2_enu_new', Imu, queue_size=10)
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

    yaw=yaw+180.0
    if (yaw > 180.0):
        yaw=yaw-360.0

    if ( yaw >=    58.0 and yaw <135.2 ):  #done
        new_yaw=translate(yaw, 58.0, 135.2, 54.6, 99.3)    
        case = 1
        
    elif ( yaw >=    135.2 or yaw <-175.2 ):  #done  181
        if yaw<0:
            n_yaw=yaw+360
        else:
            n_yaw=yaw

        new_yaw=translate(n_yaw, 135.2, -175.2+360.0, 99.3, 143.3)    
        case = 2

    elif ( yaw >= -175.2 and yaw < -120.4):

        new_yaw=translate(yaw, -175.2, -120.4, 143.3-360.0, -125.1) #done
        if new_yaw<-180.0:
            new_yaw=new_yaw+360.0
        case = 3
                    
    elif ( yaw >=    -120.4 and yaw <-91.7 ): #done
        new_yaw=translate(yaw, -120.4, -91.7, -125.1, -80.7)
        case = 4
            
    elif ( yaw >=   -91.7 and yaw <-57.0 ): #done
        new_yaw=translate(yaw, -91.7, -57.0, -80.7, -36.7) ##
        case = 5
            
                
    elif ((yaw >= -57.0 and yaw <=0) or (yaw <58.0 and yaw > 0) ): #done
        n_yaw=yaw+180.0        
        new_yaw=translate(n_yaw, -57.0+180.0, 58.0+180.0, -36.7+180.0, 54.6+180.0)
        new_yaw=new_yaw-180.0        
        case = 6
        
            
    else:
        print "there is something wrong , yaw val : ", yaw
        case =  8

    print "imu2," , time.time() ,"," , case, ",yaw," , yaw , ",new_yaw," , new_yaw 
    new_yaw= new_yaw*math.pi/180.0        
    quat = quaternion_from_euler (roll, pitch, new_yaw)
    imuMsg.orientation.x = quat[0]
    imuMsg.orientation.y = quat[1]
    imuMsg.orientation.z = quat[2]
    imuMsg.orientation.w = quat[3]
    imuMsg.orientation_covariance=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    #imuMsg.angular_velocity.x = case
    imuMsg.header.stamp=rospy.Time.now()
    imu_pub.publish(imuMsg)

#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/imu2_enu", Imu, callback_IMU)
    rospy.spin()



def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()
