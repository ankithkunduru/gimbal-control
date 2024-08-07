#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
# to run, do "python3 IK.py"
# on melodic do python2 IK.py (ros uses python2 by defult)
import sys, os, random
import numpy as np

from transforms3d import *  #supersedes deprecated transformations.py

# from PyQt4.QtCore import *
# from PyQt4.QtGui import *

# from PyQt5.QtCore import *
# from PyQt5.QtGui import *
# from PyQt5.QtWidgets import *

# import matplotlib as plt
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
# from matplotlib.figure import Figure

from mpl_toolkits import mplot3d

import rospy 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_matrix, random_rotation_matrix
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math



class RigidBodyDynamics:  

    @staticmethod
    def SkewSymmetric(p):
        p_skew = np.array([[ 0      , -p[2][0],  p[1][0]], \
                           [ p[2][0],        0, -p[0][0]], \
                           [-p[1][0],  p[0][0],        0]])
        return p_skew

    @staticmethod
    def Vee(p_skew):
        p_vee = np.array([[p_skew[2][1]], \
                          [p_skew[0][2]], \
                          [p_skew[1][0]]])
        return p_vee;

    @staticmethod
    def Inverse(T):
        R_inv = np.transpose(T[0:3,0:3])
        p = np.array([[T[0][3]], \
                      [T[1][3]], \
                      [T[2][3]]])
        T_inv = np.concatenate((np.concatenate((R_inv, -R_inv.dot(p)), axis=1), \
                                np.array([[0, 0, 0, 1]])), axis=0)
        return T_inv

    @staticmethod
    def AdjointMap(T):
        R = T[0:3,0:3]
        p = np.array([[T[0][3]], \
                      [T[1][3]], \
                      [T[2][3]]])
        p_skew = RigidBodyDynamics.SkewSymmetric(p)
        Ad_T = np.concatenate((np.concatenate((R, np.zeros((3,3))), axis=1), \
                               np.concatenate((p_skew.dot(R), R), axis=1)), axis=0)
        return Ad_T

    @staticmethod
    def ExpMap(Screw, thetaScrew_deg):
        thetaScrew = np.deg2rad(thetaScrew_deg)
        if abs(thetaScrew) < 1e-3:
            e_ScrewMatrix_thetaScrew = np.eye(4)
        elif np.linalg.norm(Screw[0:3]) < 1e-6:
            V = np.eye(3);
            e_ScrewMatrix_thetaScrew = np.concatenate((np.concatenate((np.eye(3), V.dot(Screw[3:6]) * thetaScrew), axis=1), \
                                                       np.array([[0, 0, 0, 1]])), axis=0)
        else:
            omegaSkew = RigidBodyDynamics.SkewSymmetric(Screw) * thetaScrew
            theta = np.linalg.norm(Screw[0:3] * thetaScrew)
            e_omegaSkew = np.eye(3) + ( np.sin(theta)/theta ) * omegaSkew + ( (1-np.cos(theta))/(theta**2) ) * np.linalg.matrix_power(omegaSkew, 2)
            V = np.eye(3) + ( (1-np.cos(theta))/(theta**2) ) * omegaSkew + ( (theta-np.sin(theta))/(theta**3) ) * np.linalg.matrix_power(omegaSkew, 2)
            e_ScrewMatrix_thetaScrew = np.concatenate((np.concatenate((e_omegaSkew, V.dot(Screw[3:6] * thetaScrew)), axis=1), \
                                                       np.array([[0, 0, 0, 1]])), axis=0)
        return e_ScrewMatrix_thetaScrew

    @staticmethod
    def LogMap(T):
        R = T[0:3,0:3]
        p = np.array([[T[0][3]], \
                      [T[1][3]], \
                      [T[2][3]]])

        cos_theta = (np.trace(R)-1.0)/2
        if cos_theta > 0.999999:
            omega = np.array([[0], \
                              [0], \
                              [0]])

            V = np.eye(3);
        else:
            sin_theta = np.sqrt(1.0-cos_theta**2)
            # print("sin theta = ",sin_theta)
            theta = np.arccos(cos_theta)
            ln_R = (theta / (2*sin_theta)) * (R - np.transpose(R))
            omega = RigidBodyDynamics.Vee(ln_R)

            omegaSkew = ln_R
            V = np.eye(3) + ( (1-np.cos(theta))/(theta**2) ) * omegaSkew + ( (theta-np.sin(theta))/(theta**3) ) * np.linalg.matrix_power(omegaSkew, 2)

        t = np.linalg.inv(V).dot(p)
        
        Twist = np.array([omega[0], \
                          omega[1], \
                          omega[2], \
                          t[0], \
                          t[1], \
                          t[2]])
        return Twist


class AppForm():
    def __init__(self):
        self.scenario = 2

        if self.scenario == 0:
            self.input_joint_angles_sub = rospy.Subscriber("target_angles", JointState, self.j_a_cb)
            self.ee_joint_pub = rospy.Publisher("ee_pose", PoseStamped, queue_size=2)
        elif self.scenario == 2:
            self.pose_subscriber = rospy.Subscriber("target_pose", PoseStamped, self.callback)
            self.sim_joint_pub = rospy.Publisher("ik_joint_states", JointState, queue_size=2)
            self.arduino_pub = rospy.Publisher("input_topic", Point32, queue_size=2)
        
        self.pose_goal = None
        self.prev_pose_goal = None

        self.fire_event = False
        self.G_T_active = np.NAN

        self.val_theta_1 = 0
        self.val_theta_2 = 0
        self.val_theta_3 = 0
        self.val_theta_4 = 0
        self.val_theta_5 = 0
        self.val_theta_6 = 0
        self.val_vel_x = 0
        self.val_vel_y = 0
        self.val_vel_z = 0
        self.val_omega_x = 0
        self.val_omega_y = 0
        self.val_omega_z = 0

        self.val_pos_x = 0
        self.val_pos_y = 0
        self.val_pos_z = 0
        self.val_rot_x = 0
        self.val_rot_y = 0
        self.val_rot_z = 0

        self.timer_period = 0.10
        self.timer_value = 0


        # Space Frame / origin
        # translation part of origin
        self.S_p = np.array([[0], \
                             [0], \
                             [0]])
        # Identity as rotation or origin frame
        self.S_R = np.array([[1, 0, 0], \
                             [0, 1, 0], \
                             [0, 0, 1]])
        # 
        self.S_T = np.concatenate((np.concatenate((self.S_R, self.S_p), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
        
        self.translation_S = self.S_p
        self.rotation_S = quaternions.mat2quat(self.S_R)

    def j_a_cb(self, msg):
        # msg = JointState()
        if len(msg.position) == 3:
            self.val_theta_1 = msg.position[0]
            self.val_theta_2 = msg.position[1]
            self.val_theta_3 = msg.position[2]


    def callback(self, msg):
        self.pose_goal = msg.pose
        self.prev_pose_goal = Pose()
        if self.prev_pose_goal.position.x != self.pose_goal.position.x:
            self.prev_pose_goal = self.pose_goal
            self.fire_event = True

        self.val_pos_x = self.prev_pose_goal.position.x

        self.val_pos_y = self.prev_pose_goal.position.y

        self.val_pos_z = self.prev_pose_goal.position.z

        euler = euler_from_quaternion([self.prev_pose_goal.orientation.x,
                                       self.prev_pose_goal.orientation.y,
                                       self.prev_pose_goal.orientation.z,
                                       self.prev_pose_goal.orientation.w,])
        # print("euler = {}".format(euler))

        self.val_rot_x, self.val_rot_y, self.val_rot_z = euler

    def fk_ik_loop(self):
        if self.scenario == 0 or self.scenario == 1 or self.scenario == 2:
            # lengths calculated by trial and error
            L1 = 0.1
            L2 = 0.02
            L3 = 0.02

            # Distance from home to the end effector
            Home_T = np.concatenate((np.concatenate((np.eye(3), np.array([[(-L3)], [L2], [-L1]])), axis=1), \
                                     np.array([[0, 0, 0, 1]])), axis=0)
            # skew axis for joint 1 relative to home frame
            S1 = np.concatenate((np.array([[0], [0], [-1]]), \
                                 np.cross(-np.array([[0], [0], [-1]]) , np.array([[L3], [-L2], [L1]]), axis=0)), axis=0)
            # skew axis for joint 2 relative to home frame
            S2 = np.concatenate((np.array([[0], [1], [0]]), \
                                 np.cross(-np.array([[0], [1], [0]]) , np.array([[L3], [-L2], [0]]), axis=0)), axis=0)
            # skew axis for joint 3 relative to home frame
            S3 = np.concatenate((np.array([[-1], [0], [0]]), \
                                 np.cross(-np.array([[-1], [0], [0]]) , np.array([[L3], [0], [0]]), axis=0)), axis=0)

            e_S1Matrix_theta1 = RigidBodyDynamics.ExpMap(S1, self.val_theta_1)
            e_S2Matrix_theta2 = RigidBodyDynamics.ExpMap(S2, self.val_theta_2)
            e_S3Matrix_theta3 = RigidBodyDynamics.ExpMap(S3, self.val_theta_3)

            E_T = ((Home_T.dot(e_S1Matrix_theta1)).dot(e_S2Matrix_theta2)).dot(e_S3Matrix_theta3)  # Post-multiply
            out_pose = PoseStamped()
            out_pose.pose.position.x = E_T[0,-1]
            out_pose.pose.position.x = E_T[1,-1]
            out_pose.pose.position.x = E_T[2,-1]
            # out_rot = E_T[0:3,0:3]
            # R = random_rotation_matrix()
            # print("out_rot = ",R)
            out_quat = quaternion_from_matrix(E_T)
            # print("out quat = ",out_quat)
            out_pose.pose.orientation.x = out_quat[0]
            out_pose.pose.orientation.y = out_quat[1]
            out_pose.pose.orientation.z = out_quat[2]
            out_pose.pose.orientation.w = out_quat[3]
            out_pose.header.stamp = rospy.Time.now()
            out_pose.header.frame_id = 'EE'
            if self.scenario == 0:
                self.ee_joint_pub.publish(out_pose)


            # Intermediate frames
            # Joint 1
            Home_j1 = np.concatenate((np.concatenate((np.eye(3), np.array([[(0)], [0], [0]])), axis=1), \
                                      np.array([[0, 0, 0, 1]])), axis=0)
            
            #skew axis S1 defined relative to joint 1
            j1_S1 = np.concatenate((np.array([[0], [0], [-1]]), \
                                    np.cross(-np.array([[0], [0], [-1]]) , np.array([[-(0)], [0], [0]]), axis=0)), axis=0)
           
            e_j1S1Matrix_theta1 = RigidBodyDynamics.ExpMap(j1_S1, self.val_theta_1)

            j1_T = Home_j1.dot(e_j1S1Matrix_theta1)  # Post-multiply

            # Joint 2
            Home_j2 = np.concatenate((np.concatenate((np.eye(3), np.array([[0], [0], [-L1]])), axis=1), \
                                      np.array([[0, 0, 0, 1]])), axis=0)
            #skew axes S1 and S2 defined relative to joint 2
            j2_S1 = np.concatenate((np.array([[0], [0], [-1]]), \
                                    np.cross(-np.array([[0], [0], [-1]]) , np.array([[0], [0], [L1]]), axis=0)), axis=0)
            j2_S2 = np.concatenate((np.array([[0], [1], [0]]), \
                                    np.cross(-np.array([[0], [1], [0]]) , np.array([[0], [0], [0]]), axis=0)), axis=0)
            e_j2S1Matrix_theta1 = RigidBodyDynamics.ExpMap(j2_S1, self.val_theta_1)
            e_j2S2Matrix_theta2 = RigidBodyDynamics.ExpMap(j2_S2, self.val_theta_2)
  
            j2_T = (Home_j2.dot(e_j2S1Matrix_theta1)).dot(e_j2S2Matrix_theta2)  # Post-multiply
            
            # Joint 3
            Home_j3 = np.concatenate((np.concatenate((np.eye(3), np.array([[0], [L2], [-L1]])), axis=1), \
                                      np.array([[0, 0, 0, 1]])), axis=0)
            #skew axes S1, S2, and S3 defined relative to joint 3
            j3_S1 = np.concatenate((np.array([[0], [0], [-1]]), \
                                    np.cross(-np.array([[0], [0], [-1]]) , np.array([[0], [-L2], [L1]]), axis=0)), axis=0)
            j3_S2 = np.concatenate((np.array([[0], [1], [0]]), \
                                    np.cross(-np.array([[0], [1], [0]]) , np.array([[0], [-L2], [0]]), axis=0)), axis=0)
            j3_S3 = np.concatenate((np.array([[-1], [0], [0]]), \
                                    np.cross(-np.array([[-1], [0], [0]]) , np.array([[0], [0], [0]]), axis=0)), axis=0)     

            e_j3S1Matrix_theta1 = RigidBodyDynamics.ExpMap(j3_S1, self.val_theta_1)
            e_j3S2Matrix_theta2 = RigidBodyDynamics.ExpMap(j3_S2, self.val_theta_2)
            e_j3S3Matrix_theta3 = RigidBodyDynamics.ExpMap(j3_S3, self.val_theta_3)

            j3_T = ((Home_j3.dot(e_j3S1Matrix_theta1)).dot(e_j3S2Matrix_theta2)).dot(e_j3S3Matrix_theta3)  # Post-multiply


            # Jacobian
            J_3_E = S3
            J_2_E = RigidBodyDynamics.AdjointMap(RigidBodyDynamics.Inverse( e_S3Matrix_theta3 )).dot(S2)
            J_1_E = RigidBodyDynamics.AdjointMap(RigidBodyDynamics.Inverse( e_S2Matrix_theta2.dot(e_S3Matrix_theta3) )).dot(S1)
            J_E = np.concatenate((J_1_E, J_2_E, J_3_E), axis=1)

            J_E_angular = J_E[0:3,:] 
            J_E_linear = J_E[3:6,:]

            A_manip_angular = J_E_angular.dot(np.transpose(J_E_angular))
            A_rank_angular = np.linalg.matrix_rank(A_manip_angular)
            A_manip_linear = J_E_linear.dot(np.transpose(J_E_linear))
            A_rank_linear = np.linalg.matrix_rank(A_manip_linear)

            W_linear, V_linear = np.linalg.eig(A_manip_linear)
            idx_linear = W_linear.argsort()[::-1]   
            W = W_linear[idx_linear]
            V = V_linear[:,idx_linear]
            #for e in range(0,len(W_linear)):
            for e in range(0,A_rank_linear):
                eVec = V[:,e]
                eVal = W[e]
                axis_halflen = np.sqrt(eVal)
   
                
        
        if self.scenario == 1:
            # Use Jacobian
            J_S = RigidBodyDynamics.AdjointMap(E_T).dot(J_E)

            J_S_pinv = np.linalg.pinv(J_S)
            
            Twist_S = np.array([[np.deg2rad(self.val_omega_x)], \
                                [np.deg2rad(self.val_omega_y)], \
                                [np.deg2rad(self.val_omega_z)], \
                                [self.val_vel_x], \
                                [self.val_vel_y], \
                                [self.val_vel_z]])     
       
            Joints_vel = J_S_pinv.dot(Twist_S)

            # revolute
            self.val_theta_1 = self.val_theta_1 + np.rad2deg(Joints_vel[0]) * 0.1
            # if self.val_theta_1 > 180.0:
            #     self.val_theta_1 = 180.0
            # elif self.val_theta_1 < -180.0:
            #     self.val_theta_1 = -180.0

            # revolute
            self.val_theta_2 = self.val_theta_2 + np.rad2deg(Joints_vel[1]) * 0.1
            # if self.val_theta_2 > 180.0:
            #     self.val_theta_2 = 180.0
            # elif self.val_theta_2 < -180.0:
            #     self.val_theta_2 = -180.0

            # wrap: revolute
            self.val_theta_3 = self.val_theta_3 + np.rad2deg(Joints_vel[2]) * 0.1
            # if self.val_theta_3 > 180.0:
            #     self.val_theta_3 = 180.0
            # elif self.val_theta_3 < -180.0:
            #     self.val_theta_3 = -180.0



        if self.scenario == 2:
            # Goal transformation (Body Frame-based)
            G_p_zyx = np.array([[self.val_pos_x], \
                                [self.val_pos_y], \
                                [self.val_pos_z]])
            G_T_zyx_trans = np.concatenate((np.concatenate((np.eye(3), G_p_zyx), axis=1), \
                                            np.array([[0, 0, 0, 1]])), axis=0)
            
            angle_z = (self.val_rot_z)
            angle_y = (self.val_rot_y)
            angle_x = (self.val_rot_x)

            print("ip pos, x = {},y = {},z = {}".format(self.val_pos_x, self.val_pos_y, self.val_pos_z))
            print("ip rot, x = {},y = {},z = {}".format(self.val_rot_x, self.val_rot_y, self.val_rot_z))

            G_R_z = np.array([[np.cos(angle_z), -np.sin(angle_z), 0], \
                              [np.sin(angle_z),  np.cos(angle_z), 0], \
                              [0,                0,               1]])
            G_T_z = np.concatenate((np.concatenate((G_R_z, np.zeros((3, 1))), axis=1), \
                                    np.array([[0, 0, 0, 1]])), axis=0)
            G_R_y = np.array([[np.cos(angle_y),  0, np.sin(angle_y)], \
                              [0,                1,               0], \
                              [-np.sin(angle_y), 0, np.cos(angle_y)]])
            G_T_y = np.concatenate((np.concatenate((G_R_y, np.zeros((3, 1))), axis=1), \
                                    np.array([[0, 0, 0, 1]])), axis=0)
            G_R_x = np.array([[1,               0,                0], \
                              [0, np.cos(angle_x), -np.sin(angle_x)], \
                              [0, np.sin(angle_x),  np.cos(angle_x)]])
            G_T_x = np.concatenate((np.concatenate((G_R_x, np.zeros((3, 1))), axis=1), \
                                    np.array([[0, 0, 0, 1]])), axis=0)

            G_T_zyx = (((self.S_T.dot(G_T_zyx_trans)).dot(G_T_z)).dot(G_T_y)).dot(G_T_x)

            G_T = G_T_zyx
            G_p = G_T_zyx[0:3,3]
            G_R = G_T_zyx[0:3,0:3]
            
            # update goal for IK problem
            if self.fire_event:
                self.fire_event = False
 
                self.G_T_active = G_T

            if not np.isnan(self.G_T_active).any():
                # Solve IK problem at i-th iteration
                error_i = RigidBodyDynamics.Inverse(E_T).dot(self.G_T_active) 
                Twist_i = RigidBodyDynamics.LogMap(error_i)

                # Use Jacobian
                J_E_pinv = np.linalg.pinv(J_E) 
       
                Joints_vel = J_E_pinv.dot(Twist_i)

                # revolute
                self.val_theta_1 = self.val_theta_1 + np.rad2deg(Joints_vel[0]) * 0.1
                # if self.val_theta_1 > 360.0:
                #     self.val_theta_1 = 360.0
                # elif self.val_theta_1 < -360.0:
                #     self.val_theta_1 = -360.0

                # revolute
                self.val_theta_2 = self.val_theta_2 + np.rad2deg(Joints_vel[1]) * 0.1
                # if self.val_theta_2 > 170.0:
                #     self.val_theta_2 = 170.0
                # elif self.val_theta_2 < -170.0:
                #     self.val_theta_2 = -170.0

                # revolute
                self.val_theta_3 = self.val_theta_3 + np.rad2deg(Joints_vel[2]) * 0.1
                # if self.val_theta_3 > 180.0:
                #     self.val_theta_3 = 180.0
                # elif self.val_theta_3 < -180.0:
                #     self.val_theta_3 = -180.0

                print("t1 = {}, t2 = {},t3 = {},".format(self.val_theta_1, self.val_theta_2, self.val_theta_3)) #Outputs


                js = JointState()
                js.header = Header()
                js.header.stamp = rospy.Time.now()
                js.name = ['1st_to_base', '2nd_to_1st', '3rd_to_2nd']

                self.yawjoint = np.deg2rad(self.val_theta_1)
                self.rolljoint = np.deg2rad(self.val_theta_2) 
                self.pitchjoint = np.deg2rad(self.val_theta_3)

                js.position = [self.yawjoint, self.rolljoint, self.pitchjoint]
                js.velocity = [0.005, 0.005, 0.005]
                js.effort = [0.05, 0.05, 0.05]

                p32 = Point32()
                p32.x = self.rolljoint
                p32.y = self.pitchjoint
                p32.z = self.yawjoint

                self.sim_joint_pub.publish(js)
                self.arduino_pub.publish(p32)




def main():
    rospy.init_node("gimbal_ik_node",anonymous=True)
    form = AppForm()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        form.fk_ik_loop()
        rate.sleep()



if __name__ == "__main__":
    main()

