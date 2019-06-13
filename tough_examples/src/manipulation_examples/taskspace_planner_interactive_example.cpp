#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include "tough_common/tough_common_names.h"
#include "tough_moveit_planners/taskspace_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"

WholebodyControlInterface* wb_controller;
ChestControlInterface* chest_controller;
ArmControlInterface* arm_controller;
TaskspacePlanner* planner;
RobotDescription* rd_;

bool movetoPose(const std::string planning_group, geometry_msgs::PoseStamped& goal)
{
  moveit_msgs::RobotTrajectory trajectory_msg;

  if (planner->getTrajectory(goal, planning_group, trajectory_msg))
  {
    wb_controller->executeTrajectory(trajectory_msg);
    return true;
  }
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  wb_controller = new WholebodyControlInterface(nh);
  planner = new TaskspacePlanner(nh);
  chest_controller = new ChestControlInterface(nh);
  arm_controller = new ArmControlInterface(nh);
  rd_ = RobotDescription::getRobotDescription(nh);

  std::string planning_group;

  bool RUN_STATUS = true;

  planner->waitForMoveGroupInitialization();

  while (RUN_STATUS)
  {
    std::cout << "\n\n\t************************\n"
                 "\tTaskspace Arm Planning\n"
                 "\t  1. Move to a point using 7DOF planning\n"
                 "\t  2. Move to a pose using 7DOF planning\n"
                 "\t  3. Move to a point using 10DOF planning\n"
                 "\t  4. Move to a pose using 10DOF planning\n"
                 "\t  5. Move arm to default pose\n"
                 "\t  6. Move arm to zero pose\n"
                 "\t  Select a motion or q to quit:";
    int choice, inputSide;
    std::cin >> choice;
    switch (choice)
    {
      case 1:
      {
        std::cout << "Enter <side 0=Left, 1=Right> <x> <y> <z> \n(pose in pelvis frame):";
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = rd_->getPelvisFrame();
        goal.pose.orientation.w = 1.0;  // read from current state
        std::cin >> inputSide >> goal.pose.position.x >> goal.pose.position.y >> goal.pose.position.z;

        if (inputSide == 0)
        {
          planning_group.assign(TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP);
        }
        else
        {
          planning_group.assign(TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP);
        }
        movetoPose(planning_group, goal);
        break;
      }
      case 2:
      {
        std::cout << "Enter <side 0=Left, 1=Right> <x> <y> <z> <quat_x> <quat_y> <quat_z> <quat_w> \n"
                     "(pose in pelvis frame):";
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = rd_->getPelvisFrame();
        std::cin >> inputSide >> goal.pose.position.x >> goal.pose.position.y >> goal.pose.position.z >>
            goal.pose.orientation.x >> goal.pose.orientation.y >> goal.pose.orientation.z >> goal.pose.orientation.w;

        if (inputSide == 0)
        {
          planning_group.assign(TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP);
        }
        else
        {
          planning_group.assign(TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP);
        }
        movetoPose(planning_group, goal);
        break;
      }
      case 3:
      {
        std::cout << "Enter <side 0=Left, 1=Right> <x> <y> <z> \n(pose in pelvis frame):";
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = rd_->getPelvisFrame();
        goal.pose.orientation.w = 1.0;  // read from current state
        std::cin >> inputSide >> goal.pose.position.x >> goal.pose.position.y >> goal.pose.position.z;

        if (inputSide == 0)
        {
          planning_group.assign(TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP);
        }
        else
        {
          planning_group.assign(TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP);
        }
        movetoPose(planning_group, goal);
        break;
      }
      case 4:
      {
        std::cout << "Enter <side 0=Left, 1=Right> <x> <y> <z> <quat_x> <quat_y> <quat_z> <quat_w> \n"
                     "(pose in pelvis frame):";
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = rd_->getPelvisFrame();
        std::cin >> inputSide >> goal.pose.position.x >> goal.pose.position.y >> goal.pose.position.z >>
            goal.pose.orientation.x >> goal.pose.orientation.y >> goal.pose.orientation.z >> goal.pose.orientation.w;

        if (inputSide == 0)
        {
          planning_group.assign(TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP);
        }
        else
        {
          planning_group.assign(TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP);
        }
        movetoPose(planning_group, goal);
        break;
      }
      case 5:
      {
        std::cout << "Enter <side> :";
        std::cin >> inputSide;
        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        arm_controller->moveToDefaultPose(side, 3.0);
        chest_controller->resetPose();
        break;
      }
      case 6:
      {
        std::cout << "Enter <side> :";
        std::cin >> inputSide;
        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        arm_controller->moveToZeroPose(side, 3.0);
        break;
      }
      default:
      {
        RUN_STATUS = false;
        break;
      }
        ros::Duration(3.0).sleep();
    }
  }

  delete wb_controller;  
  delete planner;
  delete chest_controller;
  delete arm_controller;  
  return 0;
}