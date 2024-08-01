#!/usr/bin/env python2.7

import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Header

from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

rospy.init_node('slider_joints')
target_twist = None



def pub_pose(posx, posy, posz, roll, pitch, yaw):
    
    pub = rospy.Publisher('pose_position', PoseStamped, queue_size=10)
    # sagepose = Pose()
    sagepose = PoseStamped()
    sagepose.header.frame_id = 'world'
    sagepose.pose.position.x = posx
    sagepose.pose.position.y = posy
    sagepose.pose.position.z = posz
    quatx, quaty, quatz, quatw = quaternion_from_euler(roll, pitch, yaw)
    sagepose.pose.orientation.x = quatx
    sagepose.pose.orientation.y = quaty
    sagepose.pose.orientation.z = quatz
    sagepose.pose.orientation.w = quatw

    
    sagepose.header.stamp = rospy.Time.now()
    
    pub.publish(sagepose)

def cb(msg):
    global target_twist
    target_twist = msg

if __name__=='__main__':
    pos_euler_sub = rospy.Subscriber('pos_euler_setpoint',Twist, cb)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target_twist is not None:
            pub_pose(target_twist.linear.x,
                     target_twist.linear.y,
                     target_twist.linear.z,
                     target_twist.angular.x,
                     target_twist.angular.y,
                     target_twist.angular.z,
                    )
        rate.sleep()