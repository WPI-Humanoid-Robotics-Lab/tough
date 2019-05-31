#pragma once

#include <ros/ros.h>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>

class ToughControlCommon
{
private:
  ros::NodeHandle nh_;

  ros::Publisher stop_traj_pub_;
  ArmControlInterface armTraj;
  ChestControlInterface chestTraj;
  PelvisControlInterface pelvisTraj;

public:

  /**
   * @brief The class ToughControlCommon provides methods to execute some basic tasks for whole robot.
   * 
   * @param nh                    - Node handle 
   */
  ToughControlCommon(ros::NodeHandle nh);
  ~ToughControlCommon();

  /**
   * @brief Stops all the executing trajectories on the robot.
   * 
   */
  void stopAllTrajectories(void);

  /**
   * @brief Reset the robot to its Default Pose.
   * 
   */
  void resetRobot();
};
