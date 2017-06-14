#!/usr/bin/env python
from __future__ import print_function, division

import argparse
import rospy
from tf import transformations
import tf2_ros
import numpy
from StringIO import StringIO

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

"""
REGEX: You can use this hasty regex to do most of the work of reformatting a message from 
/ihmc_ros/valkyrie/control/footstep_list to this format

FIND: \s*-\s*origin: \d+\s*robot_side: (\d)\s*location:\s*x: ([\d.e+-]+)\s*y: ([\d.e+-]+)\s*z: ([\d.e+-]+)\s*orientation: \s*x: ([\d.e+-]+)\s*y: ([\d.e+-]+)\s*z: ([\d.e+-]+)\s*w: ([\d.e+-]+)[^-]*unique_id: \d+
REPLACE: ($1, ($2, $3, $4), ($5, $6, $7, $8)), \n
"""

footstep_sets = {
    -90: (
        # pelvis-to-world tf (rosrun tf tf_echo /pelvis /world)
        ([0.103, -0.146, -0.994], [0.000, -0.000, 1.000, 0.002]),
        # footsteps (from rostopic echo /ihmc_ros/valkyrie/control/footstep_list, then reformatted using the regex)
        [
            # (foot, position (xyz), quaternion (xyzw)
            (0, (0.125, -0.275, 0.0982290995701), (0.0, 0.0, 1.0, 6.12323399574e-17)),
            (1, (0.025, 0.025, 0.0982304462606), (0.0, 0.0, -0.995184726672, 0.0980171403296)),
            (0, (0.075, -0.325, 0.0982290995701), (0.0, 0.0, -0.903989293123, 0.42755509343)),
            (1, (-0.075, -0.075, 0.0982304462606), (0.0, 0.0, -0.831469612303, 0.55557023302)),
            (0, (0.275, -0.025, 0.0982290995701), (0.0, 0.0, -0.707106781187, 0.707106781187)),
            (1, (0.0190632732585, -0.0171800019412, 0.0982304462606), (0.0, 0.0, -0.705664962114, 0.708545666309))
        ]
    ),
    90: (
        ([-0.060, 0.049, -0.985], [0.000, 0.000, 0.010, 1.000]),
        [
            (0, (0.025, 0.075, 0.0900992894832), (0.0, 0.0, 0.0, 1.0)),
            (1, (-0.075, -0.225, 0.0901028669253), (0.0, 0.0, 0.0980171403296, 0.995184726672)),
            (0, (-0.125, 0.125, 0.0900992894832), (0.0, 0.0, 0.42755509343, 0.903989293123)),
            (1, (0.225, -0.025, 0.0901028669253), (0.0, 0.0, 0.595699304492, 0.803207531481)),
            (0, (-0.125, -0.125, 0.0900992894832), (0.0, 0.0, 0.671558954847, 0.740951125355)),
            (1, (0.147420042966, -0.153386633716, 0.0901028669253), (0.0, 0.0, 0.683880261288, 0.729594262738))
        ]
    )
}


class ArgumentParserError(Exception): pass


class ThrowingArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        raise ArgumentParserError(message)


def footstep_marker(i, step):
    """

    :param int i:
    :param FootstepDataRosMessage step:
    :return Marker:
    """
    m = Marker()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = '/world'

    m.id = i
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = step.location.x
    m.pose.position.y = step.location.y
    m.pose.position.z = step.location.z
    m.pose.orientation.x = step.orientation.x
    m.pose.orientation.y = step.orientation.y
    m.pose.orientation.z = step.orientation.z
    m.pose.orientation.w = step.orientation.w

    m.scale.x = 0.27
    m.scale.y = 0.18
    m.scale.z = 0.015

    if step.robot_side == RIGHT:
        m.color.r = 0
        m.color.g = 1
    else:
        m.color.r = 1
        m.color.g = 0

    m.color.a = 0.6

    return m


def empty_marker(i):
    """

    :param int i:
    :param FootstepDataRosMessage step:
    :return Marker:
    """
    m = Marker()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = '/world'

    m.id = i
    m.action = Marker.DELETE

    return m


def make_steps(step_set):
    msg = FootstepDataListRosMessage()
    msg.default_transfer_time = 1.0
    msg.default_swing_time = 1.0

    reference_pose, steps = step_set

    msg.footstep_data_list = [make_footstep(reference_pose, *step) for step in steps]

    msg.unique_id = 1
    return msg


# Creates footstep offset from the current foot position. The offset is in foot frame.
def make_footstep(reference_pose, side, offset_posn, offset_quat):
    step = FootstepDataRosMessage()
    step.robot_side = side

    old_world_to_point = numpy.dot(transformations.translation_matrix(offset_posn),
                                   transformations.quaternion_matrix(offset_quat))
    pelvis_to_old_world = numpy.dot(transformations.translation_matrix(reference_pose[0]),
                                    transformations.quaternion_matrix(reference_pose[1]))

    pelvis_tf_msg = tfBuffer.lookup_transform('world', 'pelvis', rospy.Time())
    q = pelvis_tf_msg.transform.rotation
    t = pelvis_tf_msg.transform.translation
    new_world_to_pelvis = numpy.dot(transformations.translation_matrix((t.x, t.y, t.z)),
                                    transformations.quaternion_matrix((q.x, q.y, q.z, q.w)))

    new_world_to_point = new_world_to_pelvis.dot(pelvis_to_old_world).dot(old_world_to_point)

    step.location.x, step.location.y, step.location.z = transformations.translation_from_matrix(new_world_to_point)
    step.orientation.x, step.orientation.y, step.orientation.z, step.orientation.w = \
        transformations.quaternion_from_matrix(new_world_to_point)

    foot_COP_tf_msg = tfBuffer.lookup_transform('world', 'leftCOP_Frame', rospy.Time())

    # Ensure z is always at foot height
    step.location.z = foot_COP_tf_msg.transform.translation.z

    return step

if __name__ == '__main__':
    try:
        parser = ThrowingArgumentParser(description='Rotate valkyrie using predefined footsteps that are known collision-free.')
        parser.add_argument('angle', type=int, choices=footstep_sets.keys(),
                            help="The angle to rotate relative to pelvis")

        rospy.init_node('walk_rotate_safe')

        log_pub = rospy.Publisher('/field/log', String, queue_size=10)
        def log_msg(val):
            val = rospy.get_name() + ": " + val
            msg = String(val)
            log_pub.publish(msg)
            rospy.loginfo(val)

        # Wait a reasonable amount of time for log_pub to connect
        wait_until = rospy.Time.now() + rospy.Duration(0.5)
        while log_pub.get_num_connections() == 0 and rospy.Time.now() < wait_until:
            rospy.sleep(0.1)

        try:
            args = parser.parse_args()
        except ArgumentParserError as e:
            f = StringIO()
            parser.print_usage(f)
            log_msg(f.getvalue() + e.message)
            argparse.ArgumentParser.error(parser, e.message)

        footStepListPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)

        # Set up TF so we can place footsteps relative to the world frame
        # these go into the global scope
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        vis_pub = rospy.Publisher('/footstep_planner/footsteps_array', MarkerArray, queue_size=10)

        rospy.sleep(1)

        msg = make_steps(footstep_sets[args.angle])

        ma = MarkerArray()
        ma.markers = [footstep_marker(i, step) for i, step in enumerate(msg.footstep_data_list)]
        ma.markers.extend([empty_marker(i) for i in range(len(ma.markers), 100)])
        vis_pub.publish(ma)

        log_msg("Rotating " + args.angle + " using hard coded step list")
        footStepListPublisher.publish(msg)

        log_msg("Node finished, footsteps may still be executing")
    except rospy.ROSInterruptException:
        pass
