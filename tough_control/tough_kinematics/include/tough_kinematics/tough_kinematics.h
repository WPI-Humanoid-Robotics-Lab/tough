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
  std::vector<std::string> joint_names_in_traj_;

  float planning_time_;

  // IK solver
  bool updateKDLChains();
  void vectorToKDLJntArray(std::vector<double>& vec, KDL::JntArray& kdl_array);
  void KDLJntArrayToVector(KDL::JntArray& kdl_array, std::vector<double>& vec);
  void poseToKDLFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame);
  
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
    
};
