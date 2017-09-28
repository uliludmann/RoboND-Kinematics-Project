#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


#### create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:q8')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:alpha7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:a7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:d8')

########################
#project setup
########################


#### define used symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

#degrees to radians
dtr = pi/180

#### dh parameters for KUKA KR210
dh_params = {
alpha0: 0, a0: 0, d1: 0.75, q1: q1,
alpha1: -90 * dtr, a1: 0.35, d2: 0, q2: q2 - 90 * dtr,
alpha2: 0, a2: 1.25, d3: 0, q3: q3,
alpha3: -90 * dtr, a3: -0.054, d4:  1.5, q4: q4,
alpha4: 90 * dtr, a4: 0, d5: 0, q5: q5,
alpha5: -90 * dtr, a5: 0, d6: 0.193, q6: q6,
alpha6: 0, a6: 0, d7: 0.31, q7: 0 #d7 = middle of gripper.
        }

#DH Transformation function
def dh_transformation_step(alpha, a, d, q):
    """
    returns a DH Transformation matrix.
    """
    transformation_matrix = Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]
    ])
    return transformation_matrix

###helper function 
def extract_rotation_matrix(rotation_matrix):
    return rotation_matrix[0:3, 0:3]


### define elementary rotation functions
def rot_x(q):
    r_x = Matrix([[1, 0, 0, 0],
             [0, cos(q), -sin(q), 0],
             [0, sin(q), cos(q), 0],
                 [0, 0, 0, 1]])
    return r_x
  
def rot_y(q):
    r_y = Matrix([[cos(q), 0, sin(q), 0],
            [0, 1, 0, 0],
             [-sin(q), 0, cos(q), 0],
                 [0, 0, 0, 1]])
    return r_y
  
def rot_z(q):
    r_z = Matrix([[cos(q), -sin(q), 0, 0],
             [sin(q), cos(q), 0, 0],
             [0,0,1, 0],
            [0, 0, 0, 1]])
    return r_z

def get_thetas(x, y, z):
    #theta 1
    q1 = atan2(y, x)
    # project to x-z layer:
    y = y / sin(q1)
    x = x / cos(q1)
    # joint offset substraction
    x -= 0.35
    z -= 0.75
    # static lengths 
    l_cwc = 0.96
    l_ac = 1.25
    l_awc = sqrt(x ** 2 + z ** 2)
    beta = acos((l_cwc**2 + l_ac**2 - l_awc**2) / (2 * l_cwc * l_ac))
    q3 = 90 * dtr - beta
    h = atan2(z, x)
    alpha = acos((l_ac**2 + l_awc**2 - l_cwc**2) / (2 * l_ac * l_awc))
    q2 = 90 *dtr - h - alpha
    return q1.evalf(), q2.evalf(), q3.evalf()

###define RobotClass

class Robot():
    def __init__(self, dh_params):
        #on init do a forward kinematics operation.
        self.dh_params = dh_params
        self.forward_kinematics(dh_params)
        self.dh_params = dh_params
        self.R0_3 = self.T_01[0:3, 0:3] * self.T_12[0:3, 0:3] * self.T_23[0:3, 0:3]

    def get_thetas123(self, x, y, z):
        #theta 1
        q1 = atan2(y, x)
        # project to x-z layer:
        y = y / sin(q1)
        x = x / cos(q1)
        # joint offset substraction
        x -= 0.35
        z -= 0.75
        # static lengths 
        l_a = 1.50 #length link 4(0.96) + length_link5 (0.54)
        l_c = 1.25
        l_b = sqrt(x ** 2 + z ** 2)
        beta = acos((l_a**2 + l_c**2 - l_b**2) / (2 * l_a * l_c))
        q3 = 90 * dtr - beta
        h = atan2(z, x)
        alpha = acos((l_c**2 + l_b**2 - l_a**2) / (2 * l_c * l_b))
        q2 = 90 *dtr - h - (alpha + 0.036)
        return q1.evalf(), q2.evalf(), q3.evalf()

    def forward_kinematics(self, dh_params):
        self.T_01 = dh_transformation_step(alpha0, a0, d1, q1).subs(self.dh_params)
        self.T_12 = dh_transformation_step(alpha1, a1, d2, q2).subs(self.dh_params)
        self.T_23 = dh_transformation_step(alpha2, a2, d3, q3).subs(self.dh_params)
        self.T_34 = dh_transformation_step(alpha3, a3, d4, q4).subs(self.dh_params)
        self.T_45 = dh_transformation_step(alpha4, a4, d5, q5).subs(self.dh_params)
        self.T_56 = dh_transformation_step(alpha5, a5, d6, q6).subs(self.dh_params)
        self.T_6G = dh_transformation_step(alpha6, a6, d7, q7).subs(self.dh_params)
        self.R_corr = rot_z(180 * dtr) * rot_y(-90 * dtr)
        self.T_0G = self.T_01 * self.T_12 * self.T_23 * self.T_34 * self.T_45 * self.T_56 * self.T_6G * self.R_corr
        self.R0_3 = extract_rotation_matrix(self.T_01 * self.T_12 * self.T_23)

KR210 = Robot(dh_params)



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        T_0G = KR210.T_0G
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

    	    # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
    	    # Compensate for rotation discrepancy between DH parameters and Gazebo
            # -> R_corr  
    	    #
    	    #
    	    # Calculate joint angles using Geometric IK method
    	    #
    	    #
            l_ee = 0.054 + 0.193 + 0.15

            R_EE = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z


            nx, ny, nz = R_EE[0:4, 2][0:3]
            d = 0.303

            wx = px -  d * nx
            wy = py - d * ny
            wz = pz - d * nz

            R0_3 = (R0_1 * R1_2 * R2_3).subs({q1: theta1, q2: theta2, q3: theta3})

            theta1, theta2, theta3 = KR210.get_thetas123(wx, wy, wz)


            ###inverse orientation problem
            #R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
            R0_3 = extract_rotation_matrix(KR210.R0_3.subs({q1: theta1, q2: theta2, q3: theta3}))
            R3_6 = R0_3.inv('LU') * R_EE
    
            #check if rotation matrix print(R3_6.inv('LU')* R3_6)
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2]+ R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)


            forward_kin = T_0G.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
