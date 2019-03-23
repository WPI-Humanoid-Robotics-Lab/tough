#ifndef CHEST_CONTROL_INTERFACE_H
#define CHEST_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <ihmc_msgs/GoHomeRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_control_interface.h"

class ChestControlInterface : public ToughControlInterface
{
private:
  ros::Publisher chestTrajPublisher_;
  ros::Publisher homePositionPublisher_;
  std::vector<std::string> chestJointNames_;
  std::vector<int> chestJointNumbers_;

public:
  ChestControlInterface(ros::NodeHandle nh);
  ~ChestControlInterface();
  void controlChest(const float roll, const float pitch, const float yaw, const float time = 1.0f,
                    int execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE);
  void controlChest(const geometry_msgs::Quaternion quat, const float time = 1.0f,
                    int execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE);
  void executeMessage(const ihmc_msgs::ChestTrajectoryRosMessage& msg);
  void generateMessage(const geometry_msgs::Quaternion& quat, const float time, const int execution_mode,
                       ihmc_msgs::ChestTrajectoryRosMessage& msg);
  void generateMessage(const std::vector<ihmc_msgs::SO3TrajectoryPointRosMessage>& chest_trajectory,
                       const int execution_mode, ihmc_msgs::ChestTrajectoryRosMessage& msg);
  void setupFrameAndMode(ihmc_msgs::ChestTrajectoryRosMessage& msg,
                         const int mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE,
                         const int frame_hash = TOUGH_COMMON_NAMES::PELVIS_ZUP_FRAME_HASH);
  void appendChestTrajectoryPoint(const geometry_msgs::Quaternion q_in, ihmc_msgs::ChestTrajectoryRosMessage& msg,
                                  const double time = 2.0f);
  void getChestOrientation(geometry_msgs::Quaternion& orientation);
  void resetPose(float time = 2.0f);

  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                                 std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;
};

#endif  // CHEST_CONTROL_INTERFACE_H
