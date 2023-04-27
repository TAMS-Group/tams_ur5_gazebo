#!/usr/bin/env python
#
# simple python script for "movej" style robot position control.
# Hardcoded for our "tams_ur5" setup, creates and publishes a
# 1-point trajectory to the FJTAS goal topic.
# 
# Usage: 
# rosrun tams_ur5_gazebo movej 'to'|'by' <j1> <j2> <j3> <j4> <j5> <j6>
# rosrun tams_ur5_gazebo movej <named-target>    ('candle' 'home' 'above-table' ...)
#
import sys
import rospy
import numpy as np

import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg

# import visualization_msgs.msg
# import geometry_msgs.msg

joint_names = [ 'ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint',
                'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint' ]
joint_angles = np.zeros( len(joint_names ))
joint_name_map = {}
for i in range(len(joint_names)):
    joint_name_map[ joint_names[i] ] = i

# known poses similar to predefined poses in Moveit SRDF...
#
known_poses = {}
# known_poses[ 'candle' ] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # wall-collision
known_poses[ 'home' ]           = [0.0, -1.5708, 0.0, -1.5708, 0, 0]
known_poses[ 'folded' ]         = [-1.5708, -2.8591, 2.4708,  -2.7179, 1.5708, 0.0]
known_poses[ 'extended' ]       = [-1.5708, -3.1413, 0.0,  0.0, 1.5708, 0.0]

known_poses[ 'blackjack_idle' ] = [-2.563, -1.7708, -2.0847, -0.856, 1.571, -0.993] 
known_poses[ 'pour_default' ]   = [ 0.0, -2.356, -2.356, 1.571, 0.0, 0.0 ]
known_poses[ 'pour_default_2' ] = [ 0.0, -0.66, 2.217, -1.57, 0.0, 3.14 ]

known_poses[ 'start_grab_pose' ]= [ 1.76, -0.428, -1.392, -1.321, 1.381, 1.571 ]
known_poses[ 'gpd_starting_pose' ]= [ -1.571, -1.8764, -1.106, -1.710, 1.571, 0.038]
known_poses[ 'ft_calib_start_pose'] = [-1.571, -1.571, -1.571, 0, 1.571, 0] 

known_poses[ 'present_gripper'] = [-1.571, -3.151, 0.0, 1.571, 1.571, 0.0] 
known_poses[ 'clothes_hanger']  = [-1.571, -2.098, -0.516, -2.087, 1.571, -3.141]
known_poses[ 'clothes_reach']   = [-0.777, -2.529, 0.0, -2.177, 1.571, -3.141 ]

# gripper joint names
gripper_joint_names = [ \
        's_model_finger_1_joint_1', 's_model_finger_1_joint_2', 's_model_finger_1_joint_3', \
        's_model_finger_2_joint_1', 's_model_finger_2_joint_2', 's_model_finger_2_joint_3', \
        's_model_finger_middle_joint_1', 's_model_finger_middle_joint_2', 's_model_finger_middle_joint_3', \
        's_model_palm_finger_1_joint', 's_model_palm_finger_2_joint' ]


# known gripper poses according to Moveit SRDF...
#
gripper_poses =  {}
gripper_poses[ 'closed' ]       = [ 0.924, 0.0, -0.977,  0.924, 0.0, -0.977,  0.924, 0.0, -0.977,  -0.141,  0.141 ]
gripper_poses[ 'open'   ]       = [ 0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  0.928, 0.0, -0.052,   0.192, -0.192 ]
gripper_poses[ 'basic_closed' ] = [ 1.221, 1.57, -0.96,  1.221, 1.57, -0.96,  1.221, 1.57, -0.96,  -0.016,  0.016 ]
gripper_poses[ 'basic_open'   ] = [ 0.107, 0.0, -0.113,  0.107, 0.0, -0.113,  0.107, 0.0, -0.113,  -0.016,  0.016 ]

gripper_poses[ 'pinch_closed' ] = [ 0.932, 0.0, -0.986,  0.932, 0.0, -0.986,  0.932, 0.0, -0.986,  -0.156,  0.156 ]
gripper_poses[ 'pinch_open'   ] = [ 0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  -0.156,  0.156 ]
gripper_poses[ 'wide_closed'  ] = [ 1.221, 1.57, -0.96,  1.221, 1.57, -0.96,  1.221, 1.57, -0.96,   0.177, -0.177 ]
gripper_poses[ 'wide_open'    ] = [ 0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  0.049, 0.0, -0.052,   0.177, -0.177 ]

gripper_poses[ 'scissor_closed']= [ 0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  -0.178,  0.178 ]
gripper_poses[ 'scissor_open' ]=  [ 0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  0.049, 0.0, -0.052,   0.192, -0.192 ]
gripper_poses[ 'wide_pinch_closed'  ] = [ 0.924, 0.0, -0.977,  0.924, 0.0, -0.977,  0.924, 0.0, -0.77,  -0.141,  0.141 ]
gripper_poses[ 'wide_pinch_open'    ] = [ 0.049, 0.0, -0.052,  0.049, 0.0, -0.052,  0.049, 0.0, -0.052, 0.192,  -0.192 ]






def usage():
    print( "Usage:" )
    print( "rosrun tams_ur5_gazebo movej 'to'|'by' <j1> <j2> <j3> <j4> <j5> <j6>" )
    print( "rosrun tams_ur5_gazebo movej <named-target>    ('home' 'folded' ...)" )
    print( "Examples:" )
    print( "rosrun tams_ur5_gazebo movej help" )
    print( "rosrun tams_ur5_gazebo movej folded" )
    print( "rosrun tams_ur5_gazebo movej home" )
    print( "rosrun tams_ur5_gazebo movej to 0.0 _ _ _ _ 0.3" )
    print( "rosrun tams_ur5_gazebo movej by 1.571 0 . . -0.8" )
    print( "where joint values are in radians." )
    print( "Known positions:" )
    print( [ value + " " for value in known_poses.keys() ] )
    print( "Tip: don't use '*' as placeholder within bash scripts." )
    sys.exit( 1 )

def float_or_zero( value ):
    placeholders = [".", "*", "_", "#"]
    if value in placeholders: 
        return 0.0
    else: 
        return float( value )

def float_or_same( value, previous ):
    placeholders = [".", "*", "_", "#"]
    if value in placeholders: 
        return previous
    else: 
        return float( value )




# Start of "main" script...
# For some reaon, the moveit_command wants sys.argv... whatever...
# 
if __name__ == "__main__":

    if len(sys.argv) < 2: usage()

    cmd = sys.argv[1]
    print( "" + str( [ sys.argv[i] for i in range(len(sys.argv)) ]) )
   
    rospy.init_node( "movej" )

    traj_topic = '/pos_joint_traj_controller/command'
    traj_publisher = rospy.Publisher( traj_topic, trajectory_msgs.msg.JointTrajectory, latch=False, queue_size=1 )
    gripper_topic = '/gripper_controller/command'
    gripper_publisher = rospy.Publisher( gripper_topic, trajectory_msgs.msg.JointTrajectory, latch=False, queue_size=1 )
    rospy.sleep( 0.3 )

    ready = False
    while not ready:
        try:
            matched = 0
            msg = rospy.wait_for_message( "/joint_states", sensor_msgs.msg.JointState, timeout=rospy.Duration(0.2) )
            for i in range(len(msg.name)):
                if msg.name[i] in joint_name_map.keys():
                    joint_angles[ joint_name_map[ msg.name[i]]] = msg.position[i]
                    matched += 1
            if matched == len( joint_names ): # ready!
                rospy.loginfo( "... got joint angles: " + str( joint_angles ))
            break
        except: 
            rospy.logwarn_throttle( 5.0, "... waiting for /joint_states with UR5 data..." )

    tp = trajectory_msgs.msg.JointTrajectoryPoint()

    traj = trajectory_msgs.msg.JointTrajectory()
    traj.header.stamp    = rospy.Time.now()
    traj.header.frame_id = "world"
    traj.joint_names = joint_names

    if cmd == "help":
        usage()

    elif cmd == "to":
        tp.positions = [ float_or_same( sys.argv[i+2], joint_angles[i] ) for i in range(len(joint_names)) ]
        tp.velocities = np.zeros( len(joint_names) )
        tp.accelerations = np.zeros( len(joint_names) )
        deltat = np.max( [ abs( tp.positions[i] - joint_angles[i] ) for i in range(len(joint_names)) ] )
        tp.time_from_start = rospy.Duration( deltat )
        traj.points.append( tp )

    elif cmd == "by":
        tp.positions = [ (float_or_zero( sys.argv[i+2] ) + joint_angles[i] ) for i in range(len(joint_names)) ]
        tp.velocities = np.zeros( len(joint_names) )
        tp.accelerations = np.zeros( len(joint_names) )
        deltat = np.max( [ abs( tp.positions[i] - joint_angles[i] ) for i in range(len(joint_names)) ] )
        tp.time_from_start = rospy.Duration( deltat )
        traj.points.append( tp )

    elif cmd == "gripper":
        print( "Implement me!!!" )
        if not sys.argv[2] in gripper_poses.keys():
            rospy.logerr( "Unknown gripper pose '" + str( sys.argv[2] ) + "', ignored." )
            sys.exit( 1 )
 
        tp.positions = gripper_poses[ sys.argv[2] ]
        tp.velocities = np.zeros( len(gripper_joint_names) )
        tp.accelerations = np.zeros( len(gripper_joint_names) )
        tp.time_from_start = rospy.Duration( 2.0 )
        traj.joint_names = gripper_joint_names
        traj.points.append( tp )
        print( "... gripper to: " + str( tp.positions ) + " duration " + str( tp.time_from_start) )
        gripper_publisher.publish( traj )
        rospy.sleep( 0.5 )
        sys.exit( 0 )

    elif cmd in known_poses.keys():
        tp.positions = known_poses[ cmd ]
        tp.velocities = np.zeros( len(joint_names) )
        tp.accelerations = np.zeros( len(joint_names) )
        deltat = np.max( [ abs( tp.positions[i] - joint_angles[i] ) for i in range(len(joint_names)) ] )
        tp.time_from_start = rospy.Duration( deltat )
        traj.points.append( tp )

    else:
        rospy.logerr( "Unknown command '" + sys.argv[1] + "', ignored." )
        sys.exit( 1 )

    print( "... moving to: " + str( tp.positions ) + " duration " + str( tp.time_from_start) )
    traj_publisher.publish( traj )
    print( "... trajectory published..." )
    rospy.sleep( 0.5 )
    sys.exit( 0 )
  

# goal_topic = '/pos_joint_traj_controller/follow_joint_trajectory/goal'
# goal_publisher = rospy.Publisher( goal_topic, control_msgs.msg.FollowJointTrajectoryGoal, latch=False, queue_size=1 )

# roscpp_initialize(sys.argv) # needed in case you want to play with MoveitCommander

