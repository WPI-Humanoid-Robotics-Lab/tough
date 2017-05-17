#!/usr/bin/env python
import copy
import time
import rospy
import sys
import tf
import yaml

from math import ceil
from numpy import append, array, linspace, zeros

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage

def sendTrajectory(head_waypoints):
    msg = HeadTrajectoryRosMessage()
    msg.unique_id = -1
    # for each set of joint states
    for y in head_waypoints:
        # first value is time duration
        time = float(y[0])
        # subsequent values are desired joint commands
        commands = array([ float(x) for x in y[1].split() ])
        msg = appendTrajectoryPoint(msg, time, commands)
    rospy.loginfo('publishing %s trajectory' % "head")
    headTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(head_trajectory, time, rollPitchYaw):
    roll = rollPitchYaw[0]
    pitch = rollPitchYaw[1]
    yaw = rollPitchYaw[2]
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    point = copy.deepcopy(SO3TrajectoryPointRosMessage())
    point.time = time
    point.orientation = copy.deepcopy(Quaternion())
    point.orientation.x = quat[0]
    point.orientation.y = quat[1]
    point.orientation.z = quat[2]
    point.orientation.w = quat[3]
    point.angular_velocity = copy.deepcopy(Vector3())
    point.angular_velocity.x = 0
    point.angular_velocity.y = 0
    point.angular_velocity.z = 0
    head_trajectory.taskspace_trajectory_points.append(point)
    return head_trajectory

if __name__ == '__main__':
    # first make sure the input arguments are correct
    if len(sys.argv) != 3:
        print "usage: head_traj_yaml.py YAML_FILE TRAJECTORY_NAME"
        print "  where TRAJECTORY is a dictionary defined in YAML_FILE"
        sys.exit(1)
    traj_yaml = yaml.load(file(sys.argv[1], 'r'))
    traj_name = sys.argv[2]
    if not traj_name in traj_yaml:
        print "unable to find trajectory %s in %s" % (traj_name, sys.argv[1])
        sys.exit(1)
    traj = traj_yaml[traj_name]

    try:
        rospy.init_node('ihmc_head_control')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        headTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/head_trajectory".format(ROBOT_NAME), HeadTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz

        # make sure the simulation is running otherwise wait
        if headTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subsciber...')
            while headTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendTrajectory(traj)

    except rospy.ROSInterruptException:
        pass
