#!/usr/bin/env python2.7

import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Header
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler 

rospy.init_node('slider_joints')
target_twist = None


def quaternion_multiply(Q0,Q1):

    # Extract Values
    w0 = Q0.w
    x0 = Q0.x
    y0 = Q0.y
    z0 = Q0.z   

    w1 = Q1.w
    x1 = Q1.x
    y1 = Q1.y
    z1 = Q1.z
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = Quaternion()
    final_quaternion.w = Q0Q1_w
    final_quaternion.x = Q0Q1_x
    final_quaternion.y = Q0Q1_y
    final_quaternion.z = Q0Q1_z
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion


def vec_length(vector):
    sum = 0
    for term in vector:
        sum += term**2
    return np.sqrt(sum)



def pub_pose(posx, posy, posz):
    
    # sagepose = Pose()
    sagepose = PoseStamped()
    sagepose.header.frame_id = 'base_link'
    sagepose.pose.position.x = posx
    sagepose.pose.position.y = posy
    sagepose.pose.position.z = posz - 0.18

    currentdirection = [0, 1, 0]


    #convert target_twist.linear.x, target_twist.linear.y, target_twist.linear.z into the frame of the end effector

    xgoal = target_twist.linear.x 
    ygoal = target_twist.linear.y 
    zgoal =target_twist.linear.z 

    goaldirection = [xgoal, ygoal, zgoal]

    length_goaldirection = vec_length(goaldirection)
    length_currentdirection = vec_length(currentdirection)

    crossedvectors = np.cross(currentdirection, goaldirection)
    quatx = crossedvectors[0]
    quaty = crossedvectors[1]
    quatz = crossedvectors[2]
    quatw = np.sqrt((length_currentdirection**2) * (length_goaldirection**2)) + np.dot(currentdirection, goaldirection)

    #normalizing quaternion
    quatmag = quatx**2 + quaty**2 + quatz**2 + quatw**2
    qx = quatx/quatmag
    qy = quaty/quatmag
    qz = quatz/quatmag
    qw = quatw/quatmag

    sage_quat = Quaternion()
    sage_quat.x = qx
    sage_quat.y = qy
    sage_quat.z = qz
    sage_quat.w = qw

    # qr90 = Quaternion()
    # qr90.w = -np.sqrt(2)/2
    # qr90.x = 0
    # qr90.y = 0
    # qr90.z = np.sqrt(2)/2
    # sage_quat = quaternion_multiply(sage_quat, qr90)


    sagepose.pose.orientation = sage_quat
        
    sagepose.header.stamp = rospy.Time.now()
    
    return sagepose



def cb(msg):
    global target_twist
    target_twist = msg

if __name__=='__main__':
    pos_euler_sub = rospy.Subscriber('pos_euler_setpoint',Twist, cb)
    pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)
    print("running")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target_twist is not None:
            sagepose = pub_pose(target_twist.linear.x,
                     target_twist.linear.y,
                     target_twist.linear.z,
                    )
            pub.publish(sagepose)
        rate.sleep()