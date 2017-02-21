#!/usr/bin/env python
import copy
import time
import rospy
import sys
import yaml

from math import ceil
from numpy import append, array, linspace, zeros

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

def sendTrajectory(joint_waypoints):
    msgs = {}
    msgs["left"] = copy.deepcopy(ArmTrajectoryRosMessage())
    msgs["right"] = copy.deepcopy(ArmTrajectoryRosMessage())
    msgs["left"].robot_side = ArmTrajectoryRosMessage.LEFT
    msgs["right"].robot_side = ArmTrajectoryRosMessage.RIGHT
    # for each set of joint states
    for y in joint_waypoints:
        # first value is time duration
        time = float(y[0])
        side = str(y[1]).lower()
        # subsequent values are desired joint positions
        commandPosition = array([ float(x) for x in y[2].split() ])
        if side in msgs.keys():
            msgs[side] = appendTrajectoryPoint(msgs[side], time, commandPosition)
            msgs[side].unique_id = -1
    for side in msgs.keys():
        if msgs[side].unique_id == -1:
            rospy.loginfo('publishing %s trajectory' % side)
            armTrajectoryPublisher.publish(msgs[side])

def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

if __name__ == '__main__':
    # first make sure the input arguments are correct
    if len(sys.argv) != 3:
        print "usage: arm_traj_yaml.py YAML_FILE TRAJECTORY_NAME"
        print "  where TRAJECTORY is a dictionary defined in YAML_FILE"
        sys.exit(1)
    traj_yaml = yaml.load(file(sys.argv[1], 'r'))
    traj_name = sys.argv[2]
    if not traj_name in traj_yaml:
        print "unable to find trajectory %s in %s" % (traj_name, sys.argv[1])
        sys.exit(1)
    traj = traj_yaml[traj_name]

    try:
        rospy.init_node('ihmc_arm_control_yaml')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=2)

        rate = rospy.Rate(10) # 10hz

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subsciber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendTrajectory(traj)

    except rospy.ROSInterruptException:
        pass
