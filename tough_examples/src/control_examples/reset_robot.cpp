#include <ros/ros.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reset_robot");
  ros::NodeHandle nh;

  // initializing objects
  ArmControlInterface armTraj(nh);
  ChestControlInterface chestTraj(nh);
  PelvisControlInterface pelvisTraj(nh);

  pelvisTraj.controlPelvisHeight(0.75);
  chestTraj.resetPose(1.0);
  armTraj.moveToDefaultPose(RobotSide::LEFT);
  armTraj.moveToDefaultPose(RobotSide::RIGHT);
  ros::Duration(1).sleep();
}
