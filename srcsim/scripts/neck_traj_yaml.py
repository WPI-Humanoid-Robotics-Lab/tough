#!/usr/bin/env python
import copy
import time
import rospy
import sys
import yaml

from math import ceil
from numpy import append, array, linspace, zeros

from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

def sendTrajectory(joint_waypoints):
    msg = NeckTrajectoryRosMessage()
    msg.unique_id = -1
    # for each set of joint states
    for y in joint_waypoints:
        # first value is time duration
        time = float(y[0])
        # subsequent values are desired joint positions
        commandPosition = array([ float(x) for x in y[1].split() ])
        msg = appendTrajectoryPoint(msg, time, commandPosition)
    rospy.loginfo('publishing neck trajectory')
    neckTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(neck_trajectory, time, positions):
    if not neck_trajectory.joint_trajectory_messages:
        neck_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        neck_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return neck_trajectory

if __name__ == '__main__':
    # first make sure the input arguments are correct
    if len(sys.argv) != 3:
        print "usage: neck_traj_yaml.py YAML_FILE TRAJECTORY_NAME"
        print "  where TRAJECTORY is a dictionary defined in YAML_FILE"
        sys.exit(1)
    traj_yaml = yaml.load(file(sys.argv[1], 'r'))
    traj_name = sys.argv[2]
    if not traj_name in traj_yaml:
        print "unable to find trajectory %s in %s" % (traj_name, sys.argv[1])
        sys.exit(1)
    traj = traj_yaml[traj_name]

    try:
        rospy.init_node('ihmc_neck_control_yaml')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        neckTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/neck_trajectory".format(ROBOT_NAME), NeckTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz

        # make sure the simulation is running otherwise wait
        if neckTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subsciber...')
            while neckTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendTrajectory(traj)

    except rospy.ROSInterruptException:
        pass
