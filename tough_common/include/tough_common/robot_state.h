#ifndef VAL_ROBOT_STATE_H
#define VAL_ROBOT_STATE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <tf/transform_listener.h>
#include <mutex>
#include <geometry_msgs/Pose2D.h>
#include "tough_common/robot_description.h"

struct RobotState
{
  std::string name;
  float position;
  float velocity;
  float effort;
};

class RobotStateInformer
{
private:
  // private constructor to disable user from creating objects
  RobotStateInformer(ros::NodeHandle nh);
  static RobotStateInformer* currentObject_;

  RobotDescription* rd_;

  ros::NodeHandle nh_;
  ros::Subscriber jointStateSub_;
  tf::TransformListener listener_;

  std::map<std::string, RobotState> currentState_;
  std::mutex currentStateMutex_;
  std::string robotName_;

  void jointStateCB(const sensor_msgs::JointStatePtr msg);

  void inline parseParameter(const std::string& paramName, std::string& parameter)
  {
    if (paramName == "left_arm_joint_names" || paramName == "left_arm")
    {
      parameter.assign(TOUGH_COMMON_NAMES::TOPIC_PREFIX + robotName_ + "/" +
                       TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM);
    }
    else if (paramName == "right_arm_joint_names" || paramName == "right_arm")
    {
      parameter.assign(TOUGH_COMMON_NAMES::TOPIC_PREFIX + robotName_ + "/" +
                       TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM);
    }
    else
    {
      parameter.assign(paramName);
    }
  }

public:
  static RobotStateInformer* getRobotStateInformer(ros::NodeHandle nh);
  ~RobotStateInformer();

  // disable assign and copy. This is required for singleton pattern
  RobotStateInformer(RobotStateInformer const&) = delete;
  void operator=(RobotStateInformer const&) = delete;

  void getJointStateMessage(sensor_msgs::JointState& jointState);

  void getJointPositions(std::vector<double>& positions);
  bool getJointPositions(const std::string& paramName, std::vector<double>& positions);

  void getJointVelocities(std::vector<double>& velocities);
  bool getJointVelocities(const std::string& paramName, std::vector<double>& velocities);

  void getJointEfforts(std::vector<double>& efforts);
  bool getJointEfforts(const std::string& paramName, std::vector<double>& efforts);

  double getJointPosition(const std::string& jointName);
  double getJointVelocity(const std::string& jointName);
  double getJointEffort(const std::string& jointName);

  void getJointNames(std::vector<std::string>& jointNames);

  bool getCurrentPose(const std::string& frameName, geometry_msgs::Pose& pose,
                      const std::string& baseFrame = TOUGH_COMMON_NAMES::WORLD_TF);

  bool getTransform(const std::string& frameName, tf::StampedTransform& transform,
                    const std::string& baseFrame = TOUGH_COMMON_NAMES::WORLD_TF);

  bool transformQuaternion(const geometry_msgs::QuaternionStamped& qt_in, geometry_msgs::QuaternionStamped& qt_out,
                           const std::string target_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  bool transformQuaternion(const geometry_msgs::Quaternion& qt_in, geometry_msgs::Quaternion& qt_out,
                           const std::string& from_frame, const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  bool transformPoint(const geometry_msgs::PointStamped& pt_in, geometry_msgs::PointStamped& pt_out,
                      const std::string target_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  bool transformPoint(const geometry_msgs::Point& pt_in, geometry_msgs::Point& pt_out, const std::string& from_frame,
                      const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  bool transformPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out, const std::string& from_frame,
                     const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  bool transformPose(const geometry_msgs::Pose2D& pose_in, geometry_msgs::Pose2D& pose_out,
                     const std::string& from_frame, const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  bool transformVector(const geometry_msgs::Vector3& vec_in, geometry_msgs::Vector3& vec_out,
                       const std::string& from_frame, const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  bool transformVector(const geometry_msgs::Vector3Stamped& vec_in, geometry_msgs::Vector3Stamped& vec_out,
                       const std::string target_frame = TOUGH_COMMON_NAMES::WORLD_TF);
};

#endif  // VAL_ROBOT_STATE_H
