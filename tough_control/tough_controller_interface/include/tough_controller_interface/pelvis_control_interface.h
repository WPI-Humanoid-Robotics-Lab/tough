#ifndef VAL_PELVIS_NAVIGATION_H
#define VAL_PELVIS_NAVIGATION_H

#include <ihmc_msgs/PelvisHeightTrajectoryRosMessage.h>
#include <ihmc_msgs/GoHomeRosMessage.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_control_interface.h"

class PelvisControlInterface : public ToughControlInterface
{
private:
  ros::Publisher pelvisHeightPublisher_;
  ros::Publisher homePositionPublisher_;

public:
  PelvisControlInterface(ros::NodeHandle nh);
  ~PelvisControlInterface();
  void controlPelvisHeight(float height, float duration = 2.0f);
  void publishPelvisMessage(const ihmc_msgs::PelvisHeightTrajectoryRosMessage& msg) const;
  void resetPose(float time = 0.0f);

  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                                 std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;
};

#endif
