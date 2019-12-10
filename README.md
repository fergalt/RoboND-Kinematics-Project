# RoboND-Kinematics-Project

In this project, we were tasked to write code to perform Inverse Kinematics of a six degree of freedom robotic arm, meaning given a list of end-effector poses, 
the objective was to calculate joint angles for the Kuka KR210. 

The tools used to achieve this were ROS, Gazebo, RViz, URDF, and MoveIt.

The first goal was to setup your environment properly, followed by an evaluation of the forward kinematics of the Kuka KR210 to learn more about the robot's geometry and derive DH parameters. 

Once the DH parameters were obtained, a Kinematic Analysis of the robot was performed and equations derived for individual joint angles. 

In addition, the actual Inverse Kinematics code was written inside of [IK_server.py](kuka_arm/scripts/IK_server.py). 

A complete writeup, detailing derivation of the DH parameters and the kinematic Analysis can be found here: [Write Up](write_up_kinematics_updated.pdf)

Installation and setup instructions for the project can be found in [README_Setup_Instructions.md](README_Setup_Instructions.md)
