#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Fork: Derek Lau

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


# Define Rotation Matrices
def Rx(th):
	Rx = Matrix([[1,       0,        0],
		     [0, cos(th), -sin(th)], 
		     [0, sin(th), cos(th)]])
	return Rx


def Ry(th):
	Ry = Matrix([[cos(th),  0, sin(th)],
		     [      0,  1,       0], 
		     [-sin(th), 0, cos(th)]])
	return Ry


def Rz(th):
	Rz = Matrix([[cos(th), -sin(th), 0],
		     [sin(th),  cos(th), 0], 
		     [      0,        0, 1]])
	return Rz


# Define Modified DH Transformation matrix
def HTM(q, d, a, alpha):
	T = Matrix( [[cos(q), 			-sin(q), 		0, 		a],
		     [sin(q)*cos(alpha), 	cos(q)*cos(alpha), 	-sin(alpha), 	-d*sin(alpha)],
		     [sin(q)*sin(alpha), 	cos(q)*sin(alpha), 	cos(alpha), 	d*cos(alpha)],
		     [ 		   0,		0,			0,		1]] )
	return T


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	# Twist Angle
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	# Arm Link Length
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	# Arm Link Offset	
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	# Joint Angle 
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	# Create Modified DH parameters
	DHP = { alpha0: 0, 	a0: 0, 		d1: 0.75, 	q1: q1,
		alpha1: -pi/2, 	a1: 0.35, 	d2: 0,	 	q2: q2-pi/2,
		alpha2: 0, 	a2: 1.25, 	d3: 0,	 	q3: q3,
		alpha3: -pi/2, 	a3: 0.0536, 	d4: 1.5014, 	q4: q4,
		alpha4: pi/2, 	a4: 0, 		d5: 0, 		q5: q5,
		alpha5: -pi/2, 	a5: 0, 		d6: 0, 		q6: q6,
		alpha6: 0, 	a6: 0, 		d7: 0.303, 	q7: 0 }
			
	
	# Create individual transformation matrices
	T01 = HTM(q1, d1, a0, alpha0).subs(DHP)
	T12 = HTM(q2, d2, a1, alpha1).subs(DHP)
	T23 = HTM(q3, d3, a2, alpha2).subs(DHP)
	T34 = HTM(q4, d4, a3, alpha3).subs(DHP)
	T45 = HTM(q5, d5, a4, alpha4).subs(DHP)
	T56 = HTM(q6, d6, a5, alpha5).subs(DHP)
	T6G = HTM(q7, d7, a6, alpha6).subs(DHP)
		
	T0G = T01*T12*T23*T34*T45*T56*T6G


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

	    # Rotation Correction in Respect to RViz Parameters
	    RCorr = Rz(pi) * Ry(-pi/2)

	    # End Effector
	    REE = Rz(yaw) * Ry(pitch) * Rx(roll) * RCorr

	    # Calculate Wrist Center Position
	    nx = REE[0, 2]
	    ny = REE[1, 2]
	    nz = REE[2, 2]

	    # End Effector/Wrist Cartesian Coordinates 
	    # Wrist Offset = 0.303m (d7)
	    Wx = px - 0.303 * nx
	    Wy = py - 0.303 * ny
	    Wz = pz - 0.303 * nz
	

	    # Calculate joint angles using Geometric IK method
	    theta1 = atan2(Wy, Wx)
   	    a1 = 0.35
	    d1 = 0.75
	    A = 1.5
	    C = 1.25
	    rxy = sqrt(Wx**2 + Wy**2) - a1
	    rz = Wz - d1
	    r = sqrt(rxy**2 + rz**2)
	    angleA = acos((r**2 + C**2 - A**2)/(2*r*C))
	    theta2 = pi/2 - angleA - atan2(rz, rxy)
	    b = acos((A**2 + C**2 - r**2)/(2*A*C))
	    theta3 = pi/2 - b - 0.036 	#drop in link 4

	    # Extract rotation matrices from the transformation matrices
	    R03 = T01[0:3, 0:3]*T12[0:3, 0:3]*T23[0:3, 0:3]	
	    R03 = R03.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})
	    R36 = R03.T * REE
		
	    #theta4 = atan2(R36[2, 2], -R36[0, 2])
	    theta5 = atan2(sqrt(R36[0, 2]**2 + R36[2, 2]**2), R36[1, 2])
	    #theta6 = atan2(-R36[1, 1], R36[1, 0])
            if sin(theta5) < 0:
                theta4 = atan2(-R36[2,2], R36[0,2])
                theta6 = atan2(R36[1,1], -R36[1,0])
            else:
                theta4 = atan2(R36[2,2], -R36[0,2])
                theta6 = atan2(-R36[1,1], R36[1,0])

            ###

            # Populate response for the IK request
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
