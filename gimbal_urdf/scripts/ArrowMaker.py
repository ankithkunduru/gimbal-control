#!/usr/bin/env python2.7
import rospy 
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from tf.transformations import *


def quaternion_multiply(Q0,Q1):
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
 
    """
    # Extract the values from Q0
    w0 = Q0.w
    x0 = Q0.x
    y0 = Q0.y
    z0 = Q0.z
     
    # Extract the values from Q1
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


rospy.init_node('ArrowMaker')
# tf_subscriber = rospy.Subscriber("/tf2_msgs", TFMessage, queue_size=2)


def vec_length(vector):
    sum = 0
    for term in vector:
        sum += term**2
    return np.sqrt(sum)


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
        self.marker.scale.x = 10
        self.marker.scale.y = 0.015
        self.marker.scale.z = 0.015
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
        self._tf_listener = tf_listener()
        
        self.joint_txn = self._tf_listener.listen('world', 'EE')
        self.plant_txn = self._tf_listener.listen('world', target_to_track)

        if self.plant_txn is not None and self.joint_txn is not None:
            self.plant_pos = self.plant_txn.transform.translation
            self.joint_pos = self.joint_txn.transform.translation


            self.x = self.plant_pos.x - self.joint_pos.x
            self.y = self.plant_pos.y - self.joint_pos.y
            self.z = self.plant_pos.z - self.joint_pos.z

    # def __str__(self):
    #     self.whattoprint = ("just " + str(self.yaw_correction()*180/math.pi) + " degrees to yaw" + '\n' + \
    #                         "just " + str(self.pitch_correction()*180/math.pi) + " degrees to pitch" + '\n' + \
    #                         "just " + str(self.roll_correction()*180/math.pi) + " degrees to roll" + '\n' + \
    #                         "-----------------")
    #     return self.whattoprint

    # def yaw_correction(self):
    #     self.go_yaw = math.atan2(self.y, self.x) - self.yaw
    #     if self.go_yaw > math.pi:
    #         self.go_yaw -= 2*math.pi
    #     if self.go_yaw < -math.pi:
    #         self.go_yaw += 2*math.pi
    #     return self.go_yaw

    # def pitch_correction(self):
    #     self.horiz_dist = math.sqrt(self.x**2 + self.y**2)
    #     self.pitch *= -1 #I don't know why but pitch measurements are weird
    #     self.go_pitch = math.atan2(self.z, self.horiz_dist) - self.pitch
    #     if self.go_pitch > math.pi:
    #         self.go_pitch -= 2*math.pi
    #     if self.go_pitch < -math.pi:
    #         self.go_pitch += 2*math.pi
    #     return self.go_pitch
        
    # def roll_correction(self):
    #     self.plant_rot = self.plant_txn.transform.rotation
    #     self.new_plant_angles = euler_from_quaternion([self.plant_rot.x, self.plant_rot.y, self.plant_rot.z, self.plant_rot.w])
    #     self.plant_roll = self.new_plant_angles[0]

    #     self.go_roll = self.plant_roll - self.roll
    #     return self.go_roll 
    

    def make_arrow1(self):
        arrow1 = marker_maker()
        txn = self._tf_listener.listen('world', 'EE')

        #quaternion rotation of 90 degrees along z axis
        qr90 = Quaternion()
        qr90.w = np.sqrt(2)/2
        qr90.x = 0
        qr90.y = 0
        qr90.z = np.sqrt(2)/2

        #new rotation of arrow (I'm rotating it 90 degrees along the z axis)
        arrowrot = quaternion_multiply(txn.transform.rotation, qr90)

        arrowpos = Vector3()
        arrowpos.x = txn.transform.translation.x
        arrowpos.y = txn.transform.translation.y + 0.08
        arrowpos.z = txn.transform.translation.z

        if txn is not None:
            arrow1.publish(arrowpos, arrowrot)
    
    
# ------------ IMPORTANT PART BELOW ------------------------
    
    
    def pose_maker(self): 
        
        self.txn = self._tf_listener.listen('world', 'EE')
        self.prev_x_rot = self.txn.transform.rotation.x
        self.prev_y_rot = self.txn.transform.rotation.y
        self.prev_z_rot = self.txn.transform.rotation.z
        
        if self.plant_txn is not None and self.joint_txn is not None:
            
            #Rotating the plant pose so that its currentdirection-axis faces in the same direction we want the gimbal to point
            self.currentdirection = [0,1,0]
            self.goaldirection = [self.x, self.y, self.z]

            self.length_goaldirection = vec_length(self.goaldirection)
            self.length_currentdirection = vec_length(self.currentdirection)

            self.crossedvectors = np.cross(self.currentdirection, self.goaldirection)
            self.quatx = self.crossedvectors[0]
            self.quaty = self.crossedvectors[1]
            self.quatz = self.crossedvectors[2]
            self.quatw = np.sqrt((self.length_currentdirection**2) * (self.length_goaldirection**2)) + np.dot(self.currentdirection, self.goaldirection)

            #normalizing quaternion
            self.quatmag = self.quatx**2 + self.quaty**2 + self.quatz**2 + self.quatw**2
            self.qx = self.quatx/self.quatmag
            self.qy = self.quaty/self.quatmag
            self.qz = self.quatz/self.quatmag
            self.qw = self.quatw/self.quatmag

            return [self.plant_pos.x, self.plant_pos.y, self.plant_pos.z, self.qx, self.qy, self.qz, self.qw]
        else:
            return None


def publish_plant_pose():
    sage_tracker = target_tracker('sagebrush')
    # print(sage_tracker)
    # sage_tracker.make_arrow1()
    pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)
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

def publish_directionarrow():
    sage_tracker = target_tracker('sagebrush')
    sage_tracker.make_arrow1()


if __name__=='__main__':
    print("running")

while not rospy.is_shutdown():
    rate = rospy.Rate(50)                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    publish_plant_pose() 
    publish_directionarrow()
    rate.sleep()
    