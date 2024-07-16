#!/usr/bin/env python2.7
import rospy 
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from tf.transformations import *


rospy.init_node('ArrowMaker')
# tf_subscriber = rospy.Subscriber("/tf2_msgs", TFMessage, queue_size=2)

class tf_listener:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
    def listen(self,target,source):
        try:
            trans = self.tfBuffer.lookup_transform(target, source, rospy.Time(),rospy.Duration(0.8))
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
        # self.jointlistener = tf_listener('world', '4th_link')
        self._tf_listener = tf_listener()
        self.joint_txn = self._tf_listener.listen('world', '4th_link')
        if self.joint_txn is not None:
            self.joint_rotations = self.joint_txn.transform.rotation
            self.newangles = euler_from_quaternion([self.joint_rotations.x, self.joint_rotations.y, self.joint_rotations.z, self.joint_rotations.w])
            self.roll = self.newangles[0]
            self.pitch = self.newangles[1]
            self.yaw = self.newangles[2]
            self.joint_pos = self.joint_txn.transform.translation

        # self.jointlocator = tf_listener('world', '4th_link')
        # self.joint_txn = self._tf_listener.listen()
        # if self.joint_txn is not None:

        # self.plantlocator = tf_listener('world', target_to_track)
        self.plant_txn = self._tf_listener.listen('world', target_to_track)
        if self.plant_txn is not None and self.joint_txn is not None:
            self.plant_pos = self.plant_txn.transform.translation

            self.x = self.plant_pos.x - self.joint_pos.x
            self.y = self.plant_pos.y - self.joint_pos.y
            self.z = self.plant_pos.z - self.joint_pos.z

    def __str__(self):
        self.whattoprint = ("just " + str(self.yaw_correction()*180/math.pi) + " degrees to yaw" + '\n' + \
                            "just " + str(self.pitch_correction()*180/math.pi) + " degrees to pitch" + '\n' + \
                            "just " + str(self.roll_correction()*180/math.pi) + " degrees to roll" + '\n' + \
                            "-----------------")
        return self.whattoprint

    def yaw_correction(self):
        self.go_yaw = math.atan2(self.y, self.x) - self.yaw
        if self.go_yaw > math.pi:
            self.go_yaw -= 2*math.pi
        if self.go_yaw < -math.pi:
            self.go_yaw += 2*math.pi
        return self.go_yaw

    def pitch_correction(self):
        self.horiz_dist = math.sqrt(self.x**2 + self.y**2)
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
    
    def pose_maker(self):
        if self.plant_txn is not None and self.joint_txn is not None:
            
            # self.desired_yaw = math.pi - math.atan2(self.y, self.x)
            
            # self.horiz_dist = math.sqrt(self.x**2 + self.y**2)
            # self.desired_pitch = math.pi - math.atan2(self.z, self.horiz_dist)
            # # self.desired_pitch *= -1 #Pitch measurements are weird

            
            # self.desired_roll = 0 #quick-fix for now

            # self.desired_quats = quaternion_from_euler(self.desired_roll, self.desired_pitch, self.desired_yaw)

            # self.quatx = self.desired_quats[0]
            # self.quaty = self.desired_quats[1]
            # self.quatz = self.desired_quats[2]
            # self.quatw = self.desired_quats[3]

            #NEW STRATEGY
            vec_mag = math.sqrt(self.x**2 + self.y**2 + self.z**2)
            theta = math.acos(self.x/vec_mag)

            self.quatw = math.cos(theta/2)
            self.quatx = 0
            self.quaty = (-self.z*math.sin(theta/2))/vec_mag
            self.quatz = ( self.y*math.sin(theta/2))/vec_mag

            return [self.x, self.y, self.z, self.quatx, self.quaty, self.quatz, self.quatw]
        else:
            return None

    def make_arrow1(self):
        arrow1 = marker_maker()
        # arrowlistener = tf_listener('world', '4th_link')
        txn = self._tf_listener.listen('world', '4th_link')
        # print("transform =", txn.transform)
        if txn is not None:
            arrow1.publish(txn.transform.translation, txn.transform.rotation)

def publish_plant_pose():
    sage_tracker = target_tracker('sagebrush')
    print(sage_tracker)
    sage_tracker.make_arrow1()
    pub = rospy.Publisher('target_angles', PoseStamped, queue_size=10)
    listsagepose = sage_tracker.pose_maker()
    if listsagepose is not None:
        sagepose = Pose()
        sagepose.position.x = listsagepose[0]
        sagepose.position.y = listsagepose[1]
        sagepose.position.z = listsagepose[2]
        sagepose.orientation.x = listsagepose[3]
        sagepose.orientation.y = listsagepose[4]
        sagepose.orientation.z = listsagepose[5]
        sagepose.orientation.w = listsagepose[6]
        pub.publish(pose=sagepose)


while not rospy.is_shutdown():
    
    rate = rospy.Rate(50)

    # make_arrow1()                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    publish_plant_pose()
    
    rate.sleep()
    