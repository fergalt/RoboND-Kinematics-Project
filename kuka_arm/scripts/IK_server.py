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
import time
import os.path
import pickle

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        r,p,y = symbols('r p y')
         
	    # Create Modified DH parameters
        s   =   {   alpha0: 0,         a0: 0,       d1: .75,
                    alpha1: -pi/2.,    a1: 0.35,    d2: 0,      q2: -pi/2.+q2,  
                    alpha2: 0,         a2: 1.25,    d3: 0,  
                    alpha3: -pi/2.,    a3: -.054,   d4: 1.5, 
                    alpha4: pi/2.,     a4:   0,     d5: 0,     
                    alpha5: -pi/2.,    a5:   0,     d6: 0,      
                    alpha6: 0,         a6:   0,     d7: .303,   q7: 0}         
	    # Define Modified DH Transformation matrix
        def DH_Matrix(alpha,a,d,q):
            return Matrix([[             cos(q),            -sin(q),            0,              a],
                            [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                            [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                            [                   0,                   0,            0,               1]])
	

	    ## Create individual transformation matrices
        T0_1 = DH_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = DH_Matrix(alpha1, a1, d2, q2).subs(s)    
        T2_3 = DH_Matrix(alpha2, a2, d3, q3).subs(s)    
        T3_4 = DH_Matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = DH_Matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = DH_Matrix(alpha5, a5, d6, q6).subs(s)
        T6_EE = DH_Matrix(alpha6,a6, d7, q7).subs(s)
        
        ###
        ## Composition of extracted rotation matrices for rotations 0:3
        ## As these are Symbolic representations, can be stored/loaded from disk
        if not os.path.exists("R0_3_sym.p"):
            R0_3_sym = simplify(T0_1[0:3, 0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3])
            pickle.dump(R0_3_sym, open("R0_3_sym.p", "wb"))
        
        else:
            R0_3_sym = pickle.load(open("R0_3_sym.p", "rb"))

        
        
        
        ###
        ## Symbolic end effector rotation matrix:
        ## As these are Symbolic representations, can be stored/loaded from disk
        if not os.path.exists("R_EE_sym.p"):
            R_x = Matrix([[ 1,             0,       0],
                          [ 0,        cos(r), -sin(r)],
                          [ 0,        sin(r),  cos(r)]])

            R_y = Matrix([[ cos(p),        0,  sin(p)],
                          [      0,        1,       0],
                          [-sin(p),        0,  cos(p)]])

            R_z = Matrix([[ cos(y),  -sin(y),       0],
                          [ sin(y),   cos(y),       0],
                          [ 0,              0,      1]])
             
            # Uncorrected End effector rotation              
            R_EE_sym = R_z * R_y * R_x
            
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_EE_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)                     
            R_EE_sym = simplify(R_EE_sym * R_EE_corr)
            pickle.dump(R_EE_sym, open("R_EE_sym.p", "wb"))
        
        else:
            R_EE_sym = pickle.load(open("R_EE_sym.p", "rb"))
            
            

        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            t0 = time.time()
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            
            #Create matrix for end effector position.
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            
            
            ### Your IK code here
            # Populate End Effector position and rotation data
            EE = Matrix([px,py,pz])
            R_EE = R_EE_sym.subs({'r': roll,'p':pitch,'y':yaw})


            # Calculate Wrist Centre
            # Step back from End Effector, along the gripper link's z axis, to get Wrist Centre
            WC = EE - s[d7] * R_EE[:,2]

            ### Calculate joint angles using Geometric IK method
            #
            # Theta1 is the horizontal rotation of the base joint, and can be
            # deduced by the XY component of the wrist centre 
            theta1 = atan2(WC[1],WC[0])
            
            # Get wrist centre distance from origin, on XY plane
            WC_xy_mag = sqrt(WC[0]*WC[0]+WC[1]*WC[1])
            
            # Thus, the positions of WC and Joint2 are known, as are the lengths of the links between them.
            # This means that Theta2 and Theta3 can be determined by analysing the triangle created by
            # Joint 2, Joint 3, and the Wrist Centre:
            
            # Distance between Joint 3 and Wrist Centre
            side_A = sqrt((s[a3] * s[a3]) + (s[d4] * s[d4]))
            # Distance between Wrist Centre and Joint 2
            side_B = sqrt((WC_xy_mag - s[a1]) * (WC_xy_mag - s[a1]) + (WC[2] - s[d1]) * (WC[2] - s[d1]))
            # Distance between Joints 2 and 3
            side_C = s[a2]
            
            # As all sides are known, apply Cosine rules to determine angles
            angle_a = acos((side_B * side_B + side_C * side_C - side_A * side_A) / (2 * side_B * side_C))
            angle_b = acos((side_A * side_A + side_C * side_C - side_B * side_B) / (2 * side_A * side_C))    

            # Account for the sag on link 3
            angle_sag = atan2(s[a3],s[d4])
              
            # Use angles to determine Thetas 2 & 3
            theta2 = pi/2 - angle_a - atan2(WC[2] - s[d1], WC_xy_mag - s[a1])
            theta3 = pi/2 + angle_sag - angle_b

         
            # Evaluate  Rotation 0:3, using Thetas 1-3
            R0_3 = R0_3_sym.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            
            # Get rotation of spherical wrist joints by applying end effector rotation matrix to inverse of R0_3
            R3_6 = R0_3.transpose() * R_EE

            
            ##Extract euler angles
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            if sin(theta5)  < 0:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])            
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])            
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            t13 = time.time()
            ###
            #print "---Pose---"
            #print x
            #print t13-t0

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
