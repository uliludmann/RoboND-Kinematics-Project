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

#### degrees to radians
dtr = pi/180

#### dh transformation function:

def dh_transformation_step(alpha_subi_1, a_subi_1, dsubi, thetai):
    """
    returns a DH Transformation matrix.
    """
    transformation_matrix = Matrix([
        [cos(thetai), -sin(thetai), 0, a_subi_1],
        [sin(thetai) * cos(alpha_subi_1), cos(thetai) * cos(alpha_subi_1), -sin(alpha_subi_1), -sin(alpha_subi_1) * dsubi],
        [sin(thetai) * sin(alpha_subi_1), cos(thetai) * sin(alpha_subi_1), cos(alpha_subi_1), cos(alpha_subi_1) * dsubi],
        [0, 0, 0, 1]
    ])
    return transformation_matrix


def extract_rotation_matrix(rotation_matrix):
    return rotation_matrix[0:3, 0:3]


### define rotation functions
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
    r_z = Matrix([[cos(q3), -sin(q3), 0, 0],
             [sin(q3), cos(q3), 0, 0],
             [0,0,1, 0],
            [0, 0, 0, 1]])
    return r_z



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:q8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:alpha7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:a7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:d8')

	#
	#   
	# Create Modified DH parameters x
	#
    dh_params = {
    alpha0: 0, a0: 0, d1: 0.75, q1: q1,
    alpha1: -90 * dtr, a1: 0.35, d2: 0, q2: q2 - 90 * dtr,
    alpha2: 0, a2: 1.25, d3: 0, q3: q3,
    alpha3: -90 * dtr, a3: -0.054, d4:  1.5, q4: q4,
    alpha4: 90 * dtr, a4: 0, d5: 0, q5: q5,
    alpha5: -90 * dtr, a5: 0, d6: 0.193, q6: q6,
    alpha6: 0, a6: 0, d7: 0.303, q7: 0
        }


	#            
	# Define Modified DH Transformation matrix x
	#
	#
	# Create individual transformation matrices x
    T_01 = dh_transformation_step(alpha0, a0, d1, q1).subs(dh_params)
    T_12 = dh_transformation_step(alpha1, a1, d2, q2).subs(dh_params)
    T_23 = dh_transformation_step(alpha2, a2, d3, q3).subs(dh_params)
    T_34 = dh_transformation_step(alpha3, a3, d4, q4).subs(dh_params)
    T_45 = dh_transformation_step(alpha4, a4, d5, q5).subs(dh_params)
    T_56 = dh_transformation_step(alpha5, a5, d6, q6).subs(dh_params)
    T_6G = dh_transformation_step(alpha6, a6, d7, q7).subs(dh_params)

    R_corr = rot_z(180 * dtr) * rot_y(-90 * dtr)

    T_0G = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6G * R_corr
	#
	#
	# Extract rotation matrices from the transformation matrices
    rot_matrices = []
    for matrix in [T_01, T_12, T_23, T_34, T_45, T_56, T_6G]:
        rot_matrices.append(extract_rotation_matrix(matrix))

    

	#
	#
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
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

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
