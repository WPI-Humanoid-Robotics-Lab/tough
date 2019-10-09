#ifndef TOUGH_KINEMATICS_H
#define TOUGH_KINEMATICS_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <tough_common/tough_common_names.h>
#include <tough_common/robot_description.h>
#include <tough_common/robot_state.h>

class ToughKinematics
{
private:
  ros::NodeHandle nh_;

  std::map<std::string, TRAC_IK::TRAC_IK*> ik_solvers_;
  std::map<std::string, KDL::Chain*> kdl_chains_;
  std::map<std::string, std::pair<KDL::JntArray, KDL::JntArray>> kdl_joint_limits_;
  std::map<std::pair<std::string,std::string>, std::string> links_group_name_map_;
  
  std::vector<std::string> joint_names_in_traj_;

  float planning_time_=1.0;

  // IK solver
  bool addExistingKDLChains();

  /**
   * @brief 
   * 
   * @param chain_start_parent    Parent of the first actuating link
   * @param chain_end             Last link of the chain
   * @param group_name            Group name of the chain 
   * @return true 
   * @return false 
   */
  bool addChainToMap(const std::string& chain_start_parent, const std::string& chain_end, const std::string& group_name);

  void vectorToKDLJntArray(std::vector<double>& vec, KDL::JntArray& kdl_array);
  void KDLJntArrayToVector(KDL::JntArray& kdl_array, std::vector<double>& vec);
  void poseToKDLFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame);

  int get_IK_joint_angles(const std::string& planning_group, std::vector<double>& initial_joint_angles, const geometry_msgs::PoseStamped& end_effector_pose, std::vector<double>& result);

  // tough
  RobotStateInformer* state_informer_;
  RobotDescription* rd_;

public:
  ToughKinematics(ros::NodeHandle& nh, std::string urdf_param = "");
  ~ToughKinematics();

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
   * @brief Solves and provides result for the IK from custom chain for desired end_effector_pose
   *            ** THE CUSTOM CHAIN MUST BE FIRST ADDED USING add_custom_chain **
   *
   * @param chain_start                 link name for the start of the chain
   * @param chain_end                   link name for the end of the chain
   * @param end_effector_pose           Desired end effecotor pose to be achieved
   * @param result                      Resultant vector of joint angles for the planning group included joints
   * @param time                        Time for execution of the trajectory
   * @return true
   */
  bool solve_ik(const std::string& chain_start, const std::string& chain_end,
                             const geometry_msgs::PoseStamped& end_effector_pose,
                             trajectory_msgs::JointTrajectory& result, float time = 2.0f);

  /**
   * @brief Add custom chain for the IK solving.
   * 
   * @param chain_start                 link name for the start of the chain
   * @param chain_end                   link name for the end of the chain
   * @return true 
   * @return false 
   */
  bool add_custom_chain(const std::string& chain_start, const std::string& chain_end);

  /**
   * @brief Get the planning time object
   * 
   * @return double                     Current time set for the planning.
   */
  double get_planning_time();

  /**
   * @brief Set the planning time object
   * 
   * @param time                        Time to set for the planning.
   */
  void set_planning_time(const double time);
};

#endif // TOUGH_KINEMATICS_H