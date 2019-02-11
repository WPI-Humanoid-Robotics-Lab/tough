#ifndef TASKSPACE_PLANNER_H
#define TASKSPACE_PLANNER_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>
#include <tough_common/tough_common_names.h>
#include <tough_common/robot_description.h>

class TaskspacePlanner
{
public:
  TaskspacePlanner(ros::NodeHandle& nh, std::string urdf_param = "");
  ~TaskspacePlanner();

  bool getTrajectory(const geometry_msgs::PoseStamped pose, std::string planning_group,
                     moveit_msgs::RobotTrajectory& output_robot_traj_msg);

  double getPositionTolerance() const;
  void setPositionTolerance(const double position_tolerance);

  double getAngleTolerance() const;
  void setAngleTolerance(const double tolerance_angle);

  std::string getPlugin() const;
  void setPlugin(const std::string& plugin_param);

  void loadPlugin(const std::string& planner_plugin_name);
  double getPlanningTime() const;
  void setPlanningTime(const double planning_time);

  int getNumPlanningAttempts() const;
  void setNumPlanningAttempts(const int num_planning_attempts);

private:
  ros::NodeHandle nh_;
  ros::Publisher display_publisher_;

  double position_tolerance_;
  double angle_tolerance_;

  void displayInRviz(const moveit_msgs::MotionPlanResponse& response_msg);
  void loadPlanners();

  // Planner parameters
  double planning_time_;
  int num_planning_attempts_;

  std::string plugin_param_;
  std::string planner_plugin_name;

  planning_interface::PlannerManagerPtr planner_instance;
  planning_scene::PlanningScenePtr planning_scene_;
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::MotionPlanResponse res_;

  // Moveit parameters
  moveit_msgs::DisplayTrajectory display_trajectory_;
  robot_model::RobotModelPtr robot_model_;

  // Trajectory Parameters
  std::shared_ptr<robot_model::RobotState> moveit_robot_state_;
  trajectory_processing::IterativeParabolicTimeParameterization timeParameterizer;

  // tough
  RobotStateInformer* state_informer_;
  RobotDescription* rd_;
};

#endif  // TASKSPACE_PLANNER_H
