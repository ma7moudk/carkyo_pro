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
rospy.init_node('fix_imu2', anonymous=False)  
imu_pub = rospy.Publisher('/imu2_enu_new', Imu, queue_size=10)
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

    if ( yaw >=    -14.2 and yaw <55.61):  #######################
        new_yaw=translate(yaw, -14.2, 55.61, 8.3, 53.3)    # 48.262
        case = 1


    elif ((yaw >= -14.2 and yaw <=0) or (yaw <55.61 and yaw > 0) ): #48.083
        n_yaw=yaw+180.0        
        new_yaw=translate(n_yaw, -14.2+180.0, 55.61+180.0, 8.3,53.3)


    elif ( yaw >=    -58.32 and yaw <-14.2 ): #done #####################
        new_yaw=translate(yaw, -58.32, -14.2, -36.7,8.3 ) #42.362

    elif ( yaw >=    -92.48 and yaw <-58.32 ): #done
        new_yaw=translate(yaw, -92.48, -58.32, -80.1, -36.7) #42.362

    elif ( yaw >=    -119.65 and yaw <-92.48 ): #done
        new_yaw=translate(yaw, -119.65, -92.48, -125.1, -80.1) # 36.261

    elif ( yaw >=-149.42     and yaw <-119.65 ): #done
        new_yaw=translate(yaw, -149.42, -119.65, -171.7, -125.1) # 35.432

    elif ( yaw >= 171.15 or yaw < -149.42 ):  # 48.027
        if yaw<0:
            n_yaw=yaw+360
        else:
            n_yaw=yaw  # 152.82 180 , 180 , (-180 > -152.8 >> 180 >> 127  >> 360 >> 233

        new_yaw=translate(n_yaw, 171.15, -149.42+360.0, 143.3, -171.7+360.0) #done
        if new_yaw>180.0:
            new_yaw=new_yaw-360.0

    elif ( yaw >=    122.92 and yaw <171.15 ):  #54.733
        new_yaw=translate(yaw, 122.92, 171.15, 99.9, 143.3)    


    elif ( yaw >=    55.61 and yaw <122.92 ):  #47.625
        new_yaw=translate(yaw, 55.61, 122.92, 53.3 ,99.9)    
        case = 1
        
            
    else:
        print "there is something wrong , yaw val : ", yaw
        case =  8


    print  "yaw" , yaw , "new_yaw" , new_yaw
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
