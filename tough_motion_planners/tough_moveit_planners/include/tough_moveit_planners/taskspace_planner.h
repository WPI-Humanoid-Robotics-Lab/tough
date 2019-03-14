#ifndef TASKSPACE_PLANNER_H
#define TASKSPACE_PLANNER_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <thread>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

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

  double getTrajFromCartPoints(const std::vector<geometry_msgs::Pose>& pose_vec, const std::string& planning_group,
                               moveit_msgs::RobotTrajectory& robot_traj, const bool avoid_collisions = true);

  bool solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                std::vector<double>& result);

  bool solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                trajectory_msgs::JointTrajectory& result, float time = 2.0f);
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

  const std::vector<std::string> planning_groups_ = { TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP,
                                                      TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP,
                                                      TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP,
                                                      TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP };

  void displayInRviz(const moveit_msgs::MotionPlanResponse& response_msg);
  void loadPlanners();

  // Trac IK solvers
  std::map<std::string, TRAC_IK::TRAC_IK*> ik_solvers_;
  std::map<std::string, KDL::Chain*> kdl_chains_;
  std::map<std::string, std::pair<KDL::JntArray, KDL::JntArray>> kdl_joint_limits_;
  std::vector<std::string> joint_names_in_traj_;
  bool updateKDLChains();

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

  // Cartesian Planner
  void initializeMoveGroupsForCartesianPath(void);
  bool is_move_group_initializing_ = true;

  std::thread thread_for_move_group_init_;

  double computeCartesianPath(moveit::planning_interface::MoveGroupInterface& move_group,
                              const std::vector<geometry_msgs::Pose>& pose_vec,
                              moveit_msgs::RobotTrajectory& robot_traj, const bool avoid_collisions);

  moveit::planning_interface::MoveGroupInterface::Options* move_group_option_left_7_dof_;
  moveit::planning_interface::MoveGroupInterface::Options* move_group_option_left_10_dof_;
  moveit::planning_interface::MoveGroupInterface::Options* move_group_option_right_7_dof_;
  moveit::planning_interface::MoveGroupInterface::Options* move_group_option_right_10_dof_;

  moveit::planning_interface::MoveGroupInterface* move_group_left_7_dof_;
  moveit::planning_interface::MoveGroupInterface* move_group_left_10_dof_;
  moveit::planning_interface::MoveGroupInterface* move_group_right_7_dof_;
  moveit::planning_interface::MoveGroupInterface* move_group_right_10_dof_;

  // tough
  RobotStateInformer* state_informer_;
  RobotDescription* rd_;

  void vectorToKDLJntArray(std::vector<double>& vec, KDL::JntArray& kdl_array);
  void KDLJntArrayToVector(KDL::JntArray& kdl_array, std::vector<double>& vec);
  void poseToKDLFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame);
};

#endif  // TASKSPACE_PLANNER_H
