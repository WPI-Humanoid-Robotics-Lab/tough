#ifndef HEAD_CONTROL_INTERFACE_H
#define HEAD_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <ihmc_msgs/HeadTrajectoryRosMessage.h>
#include <ihmc_msgs/NeckTrajectoryRosMessage.h>
#include <ihmc_msgs/NeckDesiredAccelerationsRosMessage.h>
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
#include "tough_controller_interface/tough_control_interface.h"

/**
 * @brief The HeadTrajectory class provides ability to move the head of valkyrie. Current implementation provides the
 * ability to move the head to a set roll, pitch, and yaw.
 */
class HeadControlInterface : public ToughControlInterface
{
private:
  int NUM_NECK_JOINTS;

  ros::Publisher headTrajPublisher;
  ros::Publisher neckTrajPublisher;
  ros::Publisher neckAccnPublisher;
  void appendNeckTrajectoryPoint(ihmc_msgs::NeckTrajectoryRosMessage& msg, float time, std::vector<float> pos);

  // protected:
  //    ros::NodeHandle nh_;
  //    static int id_;
  //    RobotStateInformer *state_informer_;
  //    RobotDescription *rd_;

public:
  /**
   * @brief The HeadTrajectory class provides ability to move the head of valkyrie. Current implementation provides the
   * ability to move the head to a set roll, pitch, and yaw.
   */
  HeadControlInterface(ros::NodeHandle nh);
  ~HeadControlInterface();

  /**
   * @brief moveHead Moves the robot head to the given roll, pitch, and yaw. All orientations are expressed in pelvis
   * frame.
   * @param roll The roll in radians.
   * @param pitch The pitch in radians.
   * @param yaw The yaw in radians.
   * @param time The time it takes to move to the given orientation. Default is 4.0
   */
  void moveHead(const float roll, const float pitch, const float yaw, const float time = 4.0f);

  /**
   * @brief moveHead Moves the robot head by the given quaternion. All orientations are expressed in pelvis frame.
   * @param quaternion The quaternion representing the rotation of the head.
   * @param time The time it takes to move to the given orientation. Default is 4.0
   */
  void moveHead(const geometry_msgs::Quaternion& quaternion, const float time = 4.0f);

  /**
   * @brief moveHead          Moves the robot head through a series of trajectory roll, pitch, and yaw angles. All
   * orientations are expressed in pelvis frame.
   * @param trajectory_points The RPY angles for the head to move through in its trajectory.
   * @param time              The time it takes to move to the given orientation. Default is 4.0
   */
  void moveHead(const std::vector<std::vector<float> >& trajectory_points, const float time = 4.0f);

  /**
   * @brief moveHeadWithAcceleration: moves Head with the given accelerations. DOES NOT CONTROL THE MOTION.
   *
   * This message gives the user the option to bypass IHMC feedback controllers for the arm joints 
   * by sending desired arm joint accelerations.
   * One needs experience in control when activating the bypass as it can result in unexpected 
   * behaviors for unreasonable accelerations.
   
   * 
   * @return true 
   * @return false 
   */
  bool moveHeadWithAcceleration(const std::vector<double>& neck_accelerations);

  /**
   * @brief generates the NeckDesiredAccelerationsRosMessage. DOES NOT EXECUTE ANY MOTION.
   * 
   * @param neck_accelerations      Vector of the accelerations to be executed.
   * @param msg                     [output]
   * @return true                   If the message is generated
   * @return false                  If the message is not generated
   */
  bool generateMessage(const std::vector<double>& neck_accelerations,
                       ihmc_msgs::NeckDesiredAccelerationsRosMessage& msg);

  /**
   * @brief getNumNeckJoints Gives back the number of neck joints for Valkyrie R5
   * @return The number of neck joints.
   */
  int getNumNeckJoints() const;

  /**
   * @brief moveNeckJoints  Move the joints of the neck (lowerNeckPitch, neckYaw, upperNeckPitch) through a series of
   * trajectory points, spaced equally in the time specified.
   * @param neck_pose       The angles of the joints of the neck as a series of trajectory points to pass through.
   * @param time            The total time of the trajectories (each trajectory is spaced equally in time).
   */
  bool moveNeckJoints(const std::vector<std::vector<float> >& neck_pose, const float time = 4.0f);

  /**
   * @brief Get the current positions of all joints of head.
   * Ordering is based on the order in the JointNames vector. The order for the Joints' Names, Numbers,
   * Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param joints            [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @return true             when successful
   * @return false
   */
  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  /**
   * @brief Get the Current Pose of the head frame with respect to the fixedFrame frame
   *
   * @param pose              [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param fixedFrame        Reference frame for the state query
   * @return true             When successful
   * @return false
   */
  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                                 std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;
};

#endif  // HEAD_CONTROL_INTERFACE_H
