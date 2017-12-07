#!/usr/bin/env python

#
# A simple Python script showing the use of IHMC footstep,
# whole body, and arm messages. Uncomment the method in main
# that you want to run.
#


import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

#from ihmc_msgs.msg import WholeBodyTrajectoryPacketMessage
#from ihmc_msgs.msg import ArmJointTrajectoryPacketMessage
#from ihmc_msgs.msg import JointTrajectoryPointMessage

LEFT = 0
RIGHT = 1

LEFT_HOME =  [-0.1680786907672882, -1.213896632194519, 0.6511754393577576, -1.5278431177139282, 0.0, 0.0, 0.0]
RIGHT_HOME = [-0.19623228907585144, 1.1947178840637207, 0.6948927044868469, 1.629650354385376, 0.0, 0.0, 0.0]
ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
TEST_ELBOW = [0.0, 0.0, -1.57, 0.7, 0.0, 0.0, 0.0]
TEST_ELBOW2 = [0.0, 0.0, -1.57, 1.6, 0.0, 0.0, 0.0]

def wholeBody():
    #The message data structures
    msg = WholeBodyTrajectoryPacketMessage()

    #Pelvis and chest position
    msg.pelvis_world_position.append(Vector3(0.0, 0.0, 0.65))
    msg.pelvis_linear_velocity.append(Vector3(0.0, 0.0, 0.0))
    msg.pelvis_angular_velocity.append(Vector3(0.0, 0.0, 0.0))
    msg.pelvis_world_orientation.append(Quaternion(0.0, 0.0, 0.0, 1.0))

    msg.chest_world_orientation.append(Quaternion(0.0, 0.0, 0.0, 1.0))
    msg.chest_angular_velocity.append(Vector3(0.0, 0.0, 0.0))

    #Arm trajectories
    msg.left_arm_trajectory = ArmJointTrajectoryPacketMessage()
    msg.right_arm_trajectory = ArmJointTrajectoryPacketMessage()
    msg.left_arm_trajectory.robot_side = ArmJointTrajectoryPacketMessage.LEFT
    msg.right_arm_trajectory.robot_side = ArmJointTrajectoryPacketMessage.RIGHT

    msg.left_arm_trajectory.trajectory_points = [createTrajectoryPoint(2.0, ZERO_VECTOR)]
    msg.right_arm_trajectory.trajectory_points = [createTrajectoryPoint(2.0, ZERO_VECTOR)]

    #Misc
    msg.time_at_waypoint = [1.0]
    msg.num_waypoints = 1
    msg.num_joints_per_arm = 7
    msg.unique_id = 1

    #Publish the message
    print 'publishing the whole body message'
    wholeBodyTrajectoryPublisher.publish(msg)

def sendRightArmTrajectory():
    msg = ArmJointTrajectoryPacketMessage()

    trajectoryPoints = [createTrajectoryPoint(2.0, TEST_ELBOW),
                        createTrajectoryPoint(4.0, TEST_ELBOW2),
                        createTrajectoryPoint(6.0, TEST_ELBOW),
                        createTrajectoryPoint(8.0, TEST_ELBOW2),
                        createTrajectoryPoint(10.0, TEST_ELBOW),
                        createTrajectoryPoint(12.0, RIGHT_HOME)]


    msg.robot_side = ArmJointTrajectoryPacketMessage.RIGHT
    msg.trajectory_points = trajectoryPoints

    print 'publishing right trajectory'
    armTrajectoryPublisher.publish(msg)

def sendLeftArmTrajectory():
    msg = ArmJointTrajectoryPacketMessage()

    trajectoryPoints = [createTrajectoryPoint(2.0, ZERO_VECTOR),
                        createTrajectoryPoint(4.0, LEFT_HOME)]

    msg.robot_side = ArmJointTrajectoryPacketMessage.LEFT
    msg.trajectory_points = trajectoryPoints

    print 'publishing left trajectory'
    armTrajectoryPublisher.publish(msg)

def createTrajectoryPoint(time, positions):
    point = JointTrajectoryPointMessage()
    point.time = time
    point.positions = positions
    point.velocities = ZERO_VECTOR
    return point

def stepInPlace():
    msg = FootstepDataListMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5

    msg.footstep_data_list.append(createFootStepInPlace(LEFT))

    footStepListPublisher.publish(msg)
    print 'walking in place...'
    waitForFootsteps(len(msg.footstep_data_list))

def oneStep():
    msg = FootstepDataListMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5

    # walk forward starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.0, 0.0]))

    # walk back starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.0, 0.0]))

    footStepListPublisher.publish(msg)
    print 'one stepping...'
    waitForFootsteps(len(msg.footstep_data_list))

def boxStep():
    msg = FootstepDataListRosMessage()
    msg.unique_id = -1
    msg.default_transfer_time = 1.5
    msg.default_swing_time = 1.5

    # walk forward starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.8, 0.0, 0.0]))

    # walk left starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.8, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.8, 0.6, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, 0.6, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.8, 0.8, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, 0.8, 0.0]))

    # walk back starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.8, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.8, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.8, 0.0]))

    # walk right starting RIGHT
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.6, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.6, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.3, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.0, 0.0]))

    footStepListPublisher.publish(msg)
    print 'box stepping...'
    waitForFootsteps(len(msg.footstep_data_list))

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = 'leftFoot'
    else:
        foot_frame = 'rightFoot'

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    print 'finished set of steps'

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('valkyrie_demo')

        #Set up in the right topics to publish and subscribe
        footStepStatusSubscriber = rospy.Subscriber('/ihmc_ros/valkyrie/output/footstep_status', FootstepStatusRosMessage, recievedFootStepStatus)
        footStepListPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)
#        armTrajectoryPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/arm_joint_trajectory', ArmJointTrajectoryPacketMessage, queue_size=1)
#        wholeBodyTrajectoryPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/whole_body_trajectory', WholeBodyTrajectoryPacketMessage, queue_size=1)
 
        #Set up TF so we can place footsteps relative to the world frame       
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if footStepListPublisher.get_num_connections() == 0:
            print 'waiting for subsciber...'
            while footStepListPublisher.get_num_connections() == 0:
                rate.sleep()

        # Uncomment the right block(s) that you want to demo
        # if not rospy.is_shutdown():
        #     stepInPlace()
        # time.sleep(1)

        # if not rospy.is_shutdown():
        #     oneStep()
        # time.sleep(1)

        while not rospy.is_shutdown():
            boxStep()
        time.sleep(1)

        # if not rospy.is_shutdown():
        #     sendLeftArmTrajectory()
        # time.sleep(1)       

        # if not rospy.is_shutdown():
        #     sendRightArmTrajectory()
        # time.sleep(1)

        #if not rospy.is_shutdown():
        #    wholeBody()
        #time.sleep(1)

    except rospy.ROSInterruptException:
        pass
