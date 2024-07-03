#!/usr/bin/env python2.7
import rospy 
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from tf.transformations import *


rospy.init_node('ArrowMaker')
# tf_subscriber = rospy.Subscriber("/tf2_msgs", TFMessage, queue_size=2)

class tf_listener:
    def __init__(self, target, source):
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.target_frame = target
        self.source_frame = source
    
    def listen(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(),rospy.Duration(0.3))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        return None

class marker_maker:

    def __init__(self):
        self.direction_arrow_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=2)

        self.marker = Marker()

        self.marker.header.frame_id = "/world"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = 0
        self.marker.id = 0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.position.x = 1
        self.marker.pose.position.y = 0.1
        self.marker.pose.position.z = 2
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1

    def publish(self, pos, orn):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = pos.x
        self.marker.pose.position.y = pos.y
        self.marker.pose.position.z = pos.z
        self.marker.pose.orientation = orn
        self.direction_arrow_publisher.publish(self.marker)

class target_tracker():
    def __init__(self, target_to_track):
        self.jointlistener = tf_listener('world', '4th_link')
        self.joint_txn = self.jointlistener.listen()
        if self.joint_txn is not None:
            self.joint_rotations = self.joint_txn.transform.rotation
            self.newangles = euler_from_quaternion([self.joint_rotations.x, self.joint_rotations.y, self.joint_rotations.z, self.joint_rotations.w])
            self.roll = self.newangles[0]
            self.pitch = self.newangles[1]
            self.yaw = self.newangles[2]

        self.jointlocator = tf_listener('world', '4th_link')
        self.joint_txn = self.jointlocator.listen()
        if self.joint_txn is not None:
            self.joint_pos = self.joint_txn.transform.translation

        self.plantlocator = tf_listener('world', target_to_track)
        self.plant_txn = self.plantlocator.listen()
        if self.plant_txn is not None:
            self.plant_pos = self.plant_txn.transform.translation

        self.x = self.plant_pos.x - self.joint_pos.x
        self.y = self.plant_pos.y - self.joint_pos.y
        self.z = self.plant_pos.z - self.joint_pos.z

    def yaw_correction(self):
        self.go_yaw = math.atan2(self.y, self.x) - self.yaw
        if self.go_yaw > math.pi:
            self.go_yaw -= 2*math.pi
        if self.go_yaw < -math.pi:
            self.go_yaw += 2*math.pi
        return self.go_yaw

    def pitch_correction(self):
        self.horiz_dist = math.sqrt(self.x*self.x + self.y*self.y)
        self.pitch *= -1 #I don't know why but pitch measurements are weird
        self.go_pitch = math.atan2(self.z, self.horiz_dist) - self.pitch
        if self.go_pitch > math.pi:
            self.go_pitch -= 2*math.pi
        if self.go_pitch < -math.pi:
            self.go_pitch += 2*math.pi
        return self.go_pitch
        
    def roll_correction(self):
        self.plant_rot = self.plant_txn.transform.rotation
        self.new_plant_angles = euler_from_quaternion([self.plant_rot.x, self.plant_rot.y, self.plant_rot.z, self.plant_rot.w])
        self.plant_roll = self.new_plant_angles[0]

        self.go_roll = self.plant_roll - self.roll
        return self.go_roll 



def make_pointing_arrow():
    arrow1 = marker_maker()
    arrowlistener = tf_listener('world', '4th_link')
    txn = arrowlistener.listen()
    # print("transform =", txn.transform)
    if txn is not None:
        arrow1.publish(txn.transform.translation, txn.transform.rotation)



def make_direction_correction():
    sage_tracker = target_tracker('sagebrush')
    print("just", sage_tracker.yaw_correction()*180/math.pi, "degrees to yaw")
    print("just", sage_tracker.pitch_correction()*180/math.pi, "degrees to pitch")
    print("just", sage_tracker.roll_correction()*180/math.pi, "degrees to roll")
    print("-----------------")
    # newQuat = quaternion_from_euler(sage_tracker.roll_correction, sage_tracker.pitch_correction, sage_tracker.yaw_correction)
    # moveit = tf2_ros.transform_broadcaster.TransformBroadcaster() 

    # moveit.sendTransform()



while not rospy.is_shutdown():
    rate = rospy.Rate(50)
    make_pointing_arrow()
    make_direction_correction()
    rate.sleep()
    