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
import math
import numpy as np

import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg

# import visualization_msgs.msg
# import geometry_msgs.msg

joint_names = [ \
  'world_mallet5_joint', \
  'mallet5_stick_1_joint', 'mallet5_stick_2_joint', 'mallet5_stick_3_joint', 'mallet5_stick_4_joint', \
  # 'gabi_stick_1_joint', 'gabi_stick_2_joint', 'gabi_stick_3_joint', 'gabi_stick_4_joint', \
  # 'olga_stick_1_joint', 'olga_stick_2_joint', 'olga_stick_3_joint', 'olga_stick_4_joint' \
]
joint_angles = np.zeros( len(joint_names ))
joint_name_map = {}
for i in range(len(joint_names)):
    joint_name_map[ joint_names[i] ] = i

# known poses similar to predefined poses in Moveit SRDF...
#
known_poses = {}
known_poses[ 'home' ]           = np.zeros( len(joint_names))


def usage():
    print( "Usage:" )
    print( "rosrun tams_ur5_gazebo mallet <angle> | home" )
    sys.exit( 1 )



# Start of "main" script...
# For some reaon, the moveit_command wants sys.argv... whatever...
# 
if __name__ == "__main__":

    if len(sys.argv) < 2: usage()

    cmd = sys.argv[1]
    print( "" + str( [ sys.argv[i] for i in range(len(sys.argv)) ]) )
   
    rospy.init_node( "mallet_mallet_movej" )

    traj_topic = '/mallet_traj_controller/command'
    traj_publisher = rospy.Publisher( traj_topic, trajectory_msgs.msg.JointTrajectory, latch=False, queue_size=1 )
    rospy.sleep( 0.3 )

    tp = trajectory_msgs.msg.JointTrajectoryPoint()

    traj = trajectory_msgs.msg.JointTrajectory()
    traj.header.stamp    = rospy.Time.now()
    traj.header.frame_id = "world"
    traj.joint_names = joint_names

    if cmd == "help":
        usage()

    elif cmd == "home":
        tp.positions = known_poses[ 'home' ]
        tp.velocities = np.zeros( len(joint_names) )
        tp.accelerations = np.zeros( len(joint_names) )
        tp.time_from_start = rospy.Duration( 0.2 )
        traj.points.append( tp )

    elif cmd == "wave": # "mallet.py wave <amplitude> <hertz>"
        amplitude = float( sys.argv[2] )
        frequency = float( sys.argv[3] ) 
        # phase 1: create sinwave positions for "actuator" joint
        for i in range( 200 ):
            tp = trajectory_msgs.msg.JointTrajectoryPoint()
            tp.positions = [ amplitude*math.sin( frequency*i/200*math.pi ), 0.0, 0.0, 0.0, 0.0 ]
            tp.velocities = [ 0.0, 0.0, 0.0, 0.0, 0.0 ]
            tp.accelerations = [ 0.0, 0.0, 0.0, 0.0, 0.0 ]
            tp.time_from_start = rospy.Duration( 0.2 + 1.0*i/200 )
            traj.points.append( tp )
        # phase 2: fix velocities for "actuator" joint
        for i in range( 1, 200 ):
            pos  = traj.points[i].positions[0]
            posi = traj.points[i-1].positions[0]
            traj.points[i].velocities[0] = (pos - posi) / 200.0
        # phase 3: same for actuator accelerations
        for i in range( 2, 200 ):
            vel  = traj.points[i].velocities[0]
            veli = traj.points[i-1].velocities[0]
            traj.points[i].accelerations[0] = (vel - veli) / 200.0

    else: 
        angle = float( sys.argv[1] )
        tp.positions = np.ones( len(joint_names) ) * angle 
        tp.velocities = np.zeros( len(joint_names) )
        tp.accelerations = np.zeros( len(joint_names) )
        tp.time_from_start = rospy.Duration( 0.2 )
        traj.points.append( tp )

    print( "... moving to: " + str( tp.positions ) + " duration " + str( tp.time_from_start) )
    traj_publisher.publish( traj )
    print( "... trajectory published..." )
    rospy.sleep( 0.5 )
    sys.exit( 0 )
  
