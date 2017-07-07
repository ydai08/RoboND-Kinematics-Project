#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Completed by: Yang Dai

# import modules
import rospy
import tf
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Define DH param symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #thetas

# Modified DH params
s = {alpha0: 0,     a0: 0,      d1: 0.75,
     alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3: -0.054, d4: 1.5,
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303,  q7: 0}

# Define generic placeholder symbols
alpha, a, d, q = symbols('alpha, a, d, q')

# Define Modified DH Transformation matrix
def dh_transform(jointparam):
    # Generic DH homogenous transform
    T = Matrix([[cos(q),            -sin(q),            0,              a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),     cos(alpha)*d],
                [0,                 0,                  0,              1]])

    # Sub in each joint parameter set
    Tjoint = T.subs(jointparam)
    Tjoint = Tjoint.subs(s)
    return Tjoint

# Create individual transformation matrices
T0_1 = dh_transform({alpha: alpha0, a: a0, d: d1, q: q1})
T1_2 = dh_transform({alpha: alpha1, a: a1, d: d2, q: q2})
T2_3 = dh_transform({alpha: alpha2, a: a2, d: d3, q: q3})

T0_3 = T0_1 * T1_2 * T2_3

# Using gripper pose from sim
# Direct transform without using DH frames
# Define gripper pose symbols
sroll, spitch, syaw = symbols('sroll, spitch, syaw')

# Rotation matrices
r_z = Matrix([[cos(syaw),   -sin(syaw), 0],
              [sin(syaw),   cos(syaw),  0],
              [0,           0,          1]])
r_y = Matrix([[cos(spitch), 0,          sin(spitch)],
              [0,           1,          0],
              [-sin(spitch),0,          cos(spitch)]])
r_x = Matrix([[1,       0,              0],
              [0,       cos(sroll),     -sin(sroll)],
              [0,       sin(sroll),     cos(sroll)]])

# 3X3 Z-Y-X rotation
R_direct = r_z * r_y * r_x

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
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

            # Calculate joint angles using Geometric IK method
            # Calculate direct transform from frame 0 to gripper using gripper pose
            R0_G = R_direct.evalf(subs={sroll: roll, spitch: pitch, syaw: yaw})

            # Calculate wrist positions
            wx = (px - d7 * R0_G[0,0]).evalf(subs=s)
            wy = (py - d7 * R0_G[1,0]).evalf(subs=s)
            wz = (pz - d7 * R0_G[2,0]).evalf(subs=s)

            # Calculate thetas
            theta1 = atan2(wy, wx)

            f = ((((wx**2 + wy**2)**0.5 - a1)**2 + (wz - d1)**2)**0.5).evalf(subs=s)
            g = ((a3**2 + d4**2)**0.5).evalf(subs=s)
            h = a2.evalf(subs=s)

            # Law of cosines
            F = acos((f**2 - g**2 - h**2) / (-2 * g * h))

            # Law of sines
            G1 = asin(g * sin(F) / f)
            G2 = atan2(wz - d1, (wx**2 + wy**2)**0.5 - a1)
            G2 = G2.evalf(subs=s)

            theta2 = pi/2 - G1 - G2
            theta3 = -(F - pi/2)

            # Evaluate R0_3
            R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})[:3, :3]

            # Evaluate R3_6
            R3_6 = R0_3.T * R0_G

            # Roll
            theta4 = atan2(-R3_6[2,0], R3_6[0,0])

            # Pitch
            theta5 = -atan2((R3_6[0,0]**2 + R3_6[2,0]**2)**.5, R3_6[1,0])

            # Yaw
            theta6 = atan2(R3_6[1,1], R3_6[1,2])

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
