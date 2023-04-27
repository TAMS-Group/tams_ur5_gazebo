#!/usr/bin/env python
#
# Basic "grasp a coke can" demo for the UR5 robot with Robotiq 3-finger gripper.
# Spawning a Gazebo SDF model needs the complete contents of the .sdf file:
# https://answers.gazebosim.org//question/5553/how-does-one-use-gazebospawn_sdf_model/
#

import sys
import os
import copy
from math import pi, tau, dist, fabs, sin, cos
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list



# takes an array of UR5 joint-angles and creates a JointState message
# that can be used in moveit_commander. Expected joint order is
# elbow shoulder lift pan wrist 1 2 3 (aka alphabetical).
#
def make_arm_pose( joint_angles ):
    # note: rostopic pub /joint_states has elbow lift pan wrist 1 2 3 :-(
    # 
    js = JointState()
    js.name = [ "ur5_elbow_joint", "ur5_shoulder_lift_joint", "ur5_shoulder_pan_joint", \
                "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint" ]
    js.position = joint_angles
    return js


# takes the finger joint angles (proximal, medial, distal) for the 
# Robotiq 3-finger hand and replicates those angles on all three fingers.
# Finger abduction (palm) joints are hardcoded to "basic" mode (near zero).
#
def make_basic_gripper_pose( mf1, mf2, mf3 ):
    js = JointState()
    js.name = [ "s_model_finger_1_joint_1", "s_model_finger_1_joint_2", "s_model_finger_1_joint_3", \
                "s_model_finger_2_joint_1", "s_model_finger_2_joint_2", "s_model_finger_2_joint_3", \
                "s_model_finger_middle_joint_1", "s_model_finger_middle_joint_2", "s_model_finger_middle_joint_3", \
                "s_model_palm_finger_1_joint", "s_model_palm_finger_2_joint" ]
    js.position = [ mf1, mf2, mf3, mf1, mf2, mf3, mf1, mf2, mf3, -0.016, 0.016 ]
    return js


# some predefined arm poses for the scripted demo.
# Joint ordering is elbow shoulder lift pan wrist 1 2 3

# candle pose (same as SRDF "home" pose)
arm_home_pose = make_arm_pose( [0.0, -1.571, 0.0, -1.571, 0.0, 0.0] )

# gripper horizontal over middle of the table
arm_pose_1 = make_arm_pose( [ -2.43, -2.24, -0.87, -1.73, -0.88, -3.00 ] )

# (open) gripper just aligned with the coke can at (1, 0.8, 0.79) ...
arm_pose_2 = make_arm_pose( [ -1.83, -2.68, -1.123, -1.87, -1.13, -3.03 ] )

# lifting the coke can ~10 cm
# arm_pose_3 = make_arm_pose( [ -1.959, -2.430, -1.072, -1.862, -1.014, 0.3] )
arm_pose_3 = make_arm_pose( [ -2.34, -2.33, -1.75, -1.70, -1.5, -3.10 ] )

# above the dropping position near (1, 0.6, 0.79 )
# arm_pose_4 = make_arm_pose( [ -1.982, -2.463, -1.606, -1.811, -1.547, 0.1] )
arm_pose_4 = make_arm_pose( [ -1.86, -2.47, -1.68, -2.04, -1.68, -3.05] )

# coke can placing position
# arm_pose_5 = make_arm_pose( [ -1.841, -2.768, -1.586, -1.646, -1.527, 0.1] )
arm_pose_5 = make_arm_pose( [ -1.80, -2.69, -1.67, -1.88, -1.68, -3.05 ] )

# two predefined gripper poses for the scripted demo.
# gripper basic open pose
gripper_open_pose = make_basic_gripper_pose( 0.10, 0.02, -0.10 )

# gripper coke can grasp 
# coke_can_grasp_pose = make_basic_gripper_pose( 0.56, 0.60, -0.52 )# used in the video, but fails often
coke_can_grasp_pose = make_basic_gripper_pose( 0.59, 0.65, -0.52 )  # slightly higher grasp force ...


# from Moveit tutorial. Not used here.
# 
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


# Moving the gripper to positions it can't reach results in pretty
# high motor torques - and correspondingly large Gazebo oscillations.
# To avoid this, read the current joint-angles and write them back
# as current joint-goals.
# 
def gripper_freeze(): # command current joint-states back as the goal
    rospy.logerr( "gripper_freeze: IMPLEMENT ME!!!" )
    return


## First initialize `moveit_commander`_ and a `rospy`_ node:
print( "UR5+Robotiq coke-can grasp demo..." )
moveit_commander.roscpp_initialize( sys.argv ) # don't ask why, it's needed
rospy.init_node( 'grab_coke_can_demo', log_level=rospy.INFO)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

print( "... got the Moveit Robot: " + str(robot) )
print( "... got the RobotCommander and planning scene interface..." )
print( "... planning groups are: " + str( robot.get_group_names() ))

arm_group_name = "arm"
arm_move_group = moveit_commander.MoveGroupCommander( arm_group_name )
arm_move_group.set_max_velocity_scaling_factor( 1.0 )
arm_move_group.set_max_acceleration_scaling_factor( 1.0 )
# note: same ordering for these methods: shoulder pan lift elbow wrist 1 2 3
# while /joint_states are sorted alphabetically..."
print( "... arm group joints: " + str(arm_move_group.get_active_joints()) );
print( "... arm current positions: " + str( arm_move_group.get_current_joint_values() ));

gripper_group_name = "gripper"
gripper_move_group = moveit_commander.MoveGroupCommander( gripper_group_name )
gripper_move_group.set_max_velocity_scaling_factor( 1.0 )
gripper_move_group.set_max_acceleration_scaling_factor( 1.0 )

display_trajectory_publisher = rospy.Publisher(
   "/move_group/display_planned_path",
   moveit_msgs.msg.DisplayTrajectory,
   queue_size=20,
)

print( "... got MoveGroupCommander's for arm and gripper..." )
print( "... planning frames and default end-effectors:" )
print( str( arm_move_group.get_planning_frame() )) 
print( str( gripper_move_group.get_planning_frame() ))
print( robot.get_current_state() )


print( "... moving to the 'home' pose..." )
result = arm_move_group.plan( arm_home_pose ) 
# print( str( result ))
arm_move_group.go( arm_home_pose, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

# spawn the Coke can object at fixed position (1.0, 0.8, 0.78), aka
# on top of the experiment table.
# 
print( "... spawning a coke can object at (1.0, 0.8, 0.78)..." )
initial_pose = Pose()
initial_pose.position.x = 0.96
initial_pose.position.y = 0.80
initial_pose.position.z = 0.78

# by default, Gazebo SDF models are found in /home/<username>/.gazebo/<modelname>/model.sdf
f = open( os.path.expanduser( '~' )  + '/.gazebo/models/coke_can/model.sdf', 'r' )
sdfmodel = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("coke_can", sdfmodel, "", initial_pose, "world")  # namespace "coke_can_2" at (1,0.8,0.777)


print( "... moving to a position above the coke can..." )
# plan() takes named position, Pose, or joints list/array # takes named position, Pose, or joints list/array
# returns a tuple (success, trajectory_message, planning_time, error_code) 
result = arm_move_group.plan( arm_pose_1 ) 
# print( str( result ))
arm_move_group.go( arm_pose_1, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

print( "... opening gripper" )
gripper_move_group.go( gripper_open_pose, wait=False )
rospy.sleep( rospy.Duration( 1.0 ))

print( "... moving to grasp pose around the coke can..." )
arm_move_group.go( arm_pose_2, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

print( "... closing fingers into 'basic_closed' position..." )
gripper_move_group.go( coke_can_grasp_pose, wait=False )
rospy.sleep( rospy.Duration( 3.0 ))

# sys.exit( 1 )

print( "... freezing gripper in current finger position..." )
gripper_freeze_pose = gripper_move_group.get_current_joint_values() 
# gripper_move_group.go( gripper_freeze_pose, wait=False )
# rospy.sleep( rospy.Duration( 1.0 ))

print( "... lifting arm a bit..." )
arm_move_group.go( arm_pose_3, wait=True )
rospy.sleep( rospy.Duration( 1.0 ))

print( "... moving arm to above-target position..." )
arm_move_group.go( arm_pose_4, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

print( "... lowering arm to object place position..." )
arm_move_group.go( arm_pose_5, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

print( "... opening gripper..." )
gripper_move_group.go( gripper_open_pose, wait=True )
rospy.sleep( rospy.Duration( 1.0 ))

print( "... lifting arm..." )
arm_move_group.go( arm_pose_4, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

print( "... retracting arm..." )
arm_move_group.go( arm_pose_1, wait=True )
arm_move_group.stop()
rospy.sleep( rospy.Duration( 1.0 ))

print( "... ok. Exiting now." )
sys.exit( 0 )

