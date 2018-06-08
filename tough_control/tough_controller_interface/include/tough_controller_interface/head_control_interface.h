#ifndef VAL_HEAD_NAVIGATION_H
#define VAL_HEAD_NAVIGATION_H

#include <ros/ros.h>
#include <ihmc_msgs/HeadTrajectoryRosMessage.h>
#include <ihmc_msgs/NeckTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tough_common/robot_state.h>
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_controller_interface.h"

/**
 * @brief The HeadTrajectory class provides ability to move the head of valkyrie. Current implementation provides the ability to move the head to a set roll, pitch, and yaw.
 */
class HeadControlInterface: public ToughControllerInterface {

private:
    int NUM_NECK_JOINTS;

    ros::Publisher headTrajPublisher;
    ros::Publisher neckTrajPublisher;
    void appendNeckTrajectoryPoint(ihmc_msgs::NeckTrajectoryRosMessage &msg, float time, std::vector<float> pos);

//protected:
//    ros::NodeHandle nh_;
//    static int id_;
//    RobotStateInformer *state_informer_;
//    RobotDescription *rd_;


public:

  /**
   * @brief The HeadTrajectory class provides ability to move the head of valkyrie. Current implementation provides the ability to move the head to a set roll, pitch, and yaw.
   */
    HeadControlInterface(ros::NodeHandle nh);
    ~HeadControlInterface();

    /**
     * @brief moveHead Moves the robot head to the given roll, pitch, and yaw. All orientations are expressed in pelvis frame.
     * @param roll The roll in degrees.
     * @param pitch The pitch in degrees.
     * @param yaw The yaw in degrees.
     * @param time The time it takes to move to the given orientation. Default is 1.0
     */
    void moveHead(float roll, float pitch, float yaw, const float time = 1.0f);

    /**
     * @brief moveHead Moves the robot head by the given quaternion. All orientations are expressed in pelvis frame.
     * @param quaternion The quaternion representing the rotation of the head.
     * @param time The time it takes to move to the given orientation. Default is 1.0
     */
    void moveHead(const geometry_msgs::Quaternion &quaternion, const float time = 1.0f);

    /**
     * @brief moveHead          Moves the robot head through a series of trajectory roll, pitch, and yaw angles. All orientations are expressed in pelvis frame.
     * @param trajectory_points The RPY angles for the head to move through in its trajectory.
     * @param time              The time it takes to move to the given orientation. Default is 1.0
     */
    void moveHead(const std::vector<std::vector<float> > &trajectory_points, const float time = 1.0f);

    /**
     * @brief getNumNeckJoints Gives back the number of neck joints for Valkyrie R5
     * @return The number of neck joints.
     */
    int getNumNeckJoints() const;

    /**
     * @brief moveNeckJoints  Move the joints of the neck (lowerNeckPitch, neckYaw, upperNeckPitch) through a series of trajectory points, spaced equally in the time specified.
     * @param neck_pose       The angles of the joints of the neck as a series of trajectory points to pass through.
     * @param time            The total time of the trajectories (each trajectory is spaced equally in time).
     */
    void moveNeckJoints(const std::vector<std::vector<float> > &neck_pose, const float time);
};

#endif // VAL_HEAD_NAVIGATION_H
