#ifndef TASKSPACE_PLANNER_H
#define TASKSPACE_PLANNER_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <thread>
#include <map>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
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
  /**
   * @brief This class sets up the planners for the task-space planning for the robot
   *
   * @param nh              nodehandle to which subscribers and publishers are attached.
   * @param urdf_param      URDF parameter for the ros parameter server.
   */
  TaskspacePlanner(ros::NodeHandle& nh, std::string urdf_param = "");
  ~TaskspacePlanner();

  /**
   * @brief Plans the trajectory for the planning_group to reach the given pose
   * 
   * @param pose                        Pose to be achieved by the robot
   * @param planning_group              planning group for the trajectory planning, it can be for left or right side for 7 or 10 dof
   * @param output_robot_traj_msg       [output]
   * @return true                       When Successful
   * @return false 
   */
  bool getTrajectory(const geometry_msgs::PoseStamped pose, std::string planning_group,
                     moveit_msgs::RobotTrajectory& output_robot_traj_msg);

  /**
   * @brief Plans the trajector for the robot to follow a vector of poses
   *
   * @param pose_vec                    Vector of Poses to be followed by the robot
   * @param planning_group              planning group for the trajectory planning, it can be for left or right side for 7 or 10 dof
   * @param robot_traj                  [output]
   * @param avoid_collisions            true if robot has to avoid collision, false if not.
   * 
   * @return double                     returns the fraction of desired trajectory planned.
   */
  double getTrajFromCartPoints(const std::vector<geometry_msgs::Pose>& pose_vec, const std::string& planning_group,
                               moveit_msgs::RobotTrajectory& robot_traj, const bool avoid_collisions = true);

  /**
   * @brief Plans the trajector for the robot to follow a vector of poses
   *
   * @param pose_array                  geometry_msgs::PoseArray object consisting of Poses to be followed by the robot
   * @param planning_group              planning group for the trajectory planning, it can be for left or right side for 7 or 10 dof
   * @param robot_traj                  [output]
   * @param avoid_collisions            true if robot has to avoid collision, false if not.
   * 
   * @return double                     returns the fraction of desired trajectory planned.
   */
  double getTrajFromCartPoints(const geometry_msgs::PoseArray& pose_array, const std::string& planning_group,
                               moveit_msgs::RobotTrajectory& robot_traj, const bool avoid_collisions = true);

  /**
   * @brief Solves and provides result for the IK for desired end_effector_pose
   *
   * @param planning_group              planning group for the trajectory planning, it can be for left or right side for 7 or 10 dof
   * @param end_effector_pose           Desired end effecotor pose to be achieved
   * @param result                      Resultant vector of joint angles for the planning group included joints
   * @return true                       When Successful
   * @return false
   */
  bool solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                std::vector<double>& result);

  /**
   * @brief Solves and provides result for the IK for desired end_effector_pose
   *
   * @param planning_group              planning group for the trajectory planning, it can be for left or right side for 7 or 10 dof
   * @param end_effector_pose           Desired end effecotor pose to be achieved
   * @param result                      Resultant vector of joint angles for the planning group included joints
   * @param time                        Time for execution of the trajectory
   * @return true
   * @return false
   */
  bool solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                trajectory_msgs::JointTrajectory& result, float time = 2.0f);
  
  /**
   * @brief Get the Position Tolerance
   * 
   * @return double                     The set position tolerance
   */
  double getPositionTolerance() const;

  /**
   * @brief Set the Position Tolerance 
   * 
   * @param position_tolerance          Desired position tolerance
   */
  void setPositionTolerance(const double position_tolerance);

  /**
   * @brief Get the Angle Tolerance 
   * 
   * @return double                     The set Angle Tolerance
   */
  double getAngleTolerance() const;

  /**
   * @brief Set the Angle Tolerance 
   * 
   * @param tolerance_angle             Desired Angle Tolerance
   */
  void setAngleTolerance(const double tolerance_angle);

  /**
   * @brief Get the Plugin Parameter
   * 
   * @return std::string                The set Plugin Parameter
   */
  std::string getPlugin() const;

  /**
   * @brief Set the Plugin parameter
   * 
   * @param plugin_param                Desired plugin parameter
   */
  void setPlugin(const std::string& plugin_param);

  /**
   * @brief Loads and initializes the plugins
   * 
   * @param planner_plugin_name         Plugin to be initialized
   */
  void loadPlugin(const std::string& planner_plugin_name);

  /**
   * @brief Get the Planning Time 
   * 
   * @return double                     The set planning time
   */
  double getPlanningTime() const;

  /**
   * @brief Set the Planning Time 
   * 
   * @param planning_time               Desired planning time 
   */
  void setPlanningTime(const double planning_time);

  /**
   * @brief Get the Num Planning Attempts 
   * 
   * @return int                        The set planning attempts
   */
  int getNumPlanningAttempts() const;

  /**
   * @brief Set the Num Planning Attempts 
   * 
   * @param num_planning_attempts       Desired planning attempts
   */
  void setNumPlanningAttempts(const int num_planning_attempts);

  /**
   * @brief Waits for the move group initialization to finish
   * 
   */
  void waitForMoveGroupInitialization();

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
  trajectory_processing::IterativeSplineParameterization timeParameterizer;
  
  // Cartesian Planner
  void initializeMoveGroupsForCartesianPath(void);
  bool is_move_group_initializing_ = true;

  std::thread thread_for_move_group_init_;

  double computeCartesianPathFromVector(moveit::planning_interface::MoveGroupInterface& move_group,
                              const std::vector<geometry_msgs::Pose>& pose_vec,
                              moveit_msgs::RobotTrajectory& robot_traj, const bool avoid_collisions);

  std::map<std::string, moveit::planning_interface::MoveGroupInterface::Options> move_group_interface_option_map_;
  std::map<std::string, moveit::planning_interface::MoveGroupInterface> move_group_interface_map_;

  // tough
  RobotStateInformer* state_informer_;
  RobotDescription* rd_;

  void vectorToKDLJntArray(std::vector<double>& vec, KDL::JntArray& kdl_array);
  void KDLJntArrayToVector(KDL::JntArray& kdl_array, std::vector<double>& vec);
  void poseToKDLFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame);
};

#endif  // TASKSPACE_PLANNER_H
