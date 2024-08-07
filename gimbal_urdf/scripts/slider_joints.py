#!/usr/bin/env python2.7

import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Header
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

rospy.init_node('slider_joints')
target_twist = None



def pub_pose(posx, posy, posz, roll, pitch, yaw):
    
    # sagepose = Pose()
    sagepose = PoseStamped()
    sagepose.header.frame_id = 'base_link'
    sagepose.pose.position.x = posx
    sagepose.pose.position.y = posy
    sagepose.pose.position.z = posz
    quatx, quaty, quatz, quatw = quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
    sagepose.pose.orientation.x = quatx
    sagepose.pose.orientation.y = quaty
    sagepose.pose.orientation.z = quatz
    sagepose.pose.orientation.w = quatw

    
    sagepose.header.stamp = rospy.Time.now()
    return sagepose
    

def cb(msg):
    global target_twist
    target_twist = msg

if __name__=='__main__':
    pos_euler_sub = rospy.Subscriber('pos_euler_setpoint',Twist, cb)
    pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target_twist is not None:
            sagepose = pub_pose(target_twist.linear.x,
                     target_twist.linear.y,
                     target_twist.linear.z,
                     target_twist.angular.x,
                     target_twist.angular.y,
                     target_twist.angular.z,
                    )
            pub.publish(sagepose)
        rate.sleep()