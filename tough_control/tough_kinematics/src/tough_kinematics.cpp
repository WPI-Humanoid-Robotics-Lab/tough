#include <tough_kinematics/tough_kinematics.h>

ToughKinematics::ToughKinematics(ros::NodeHandle& nh, std::string urdf_param) : nh_(nh)
{
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh);

  addExistingKDLChains();
}

ToughKinematics::~ToughKinematics()
{
}

bool ToughKinematics::addExistingKDLChains()
{
  bool valid;
  for (std::string planning_group : TOUGH_COMMON_NAMES::PLANNING_GROUPS)
  {
    std::string chain_start_parent = rd_->getParentFrameForMoveGroups(planning_group);
    std::string chain_end = rd_->getFrameNamesInMoveGroup(planning_group).back();

    valid = addChainToMap(chain_start_parent, chain_end, planning_group);
  }
  return valid;
}

bool ToughKinematics::addChainToMap(const std::string& chain_start_parent, const std::string& chain_end, const std::string& planning_group)
{
  bool valid;
  ik_solvers_[planning_group] = new TRAC_IK::TRAC_IK(chain_start_parent,
                                                     chain_end,
                                                     rd_->getURDFParameter(), this->planning_time_);

  // get the KDL chain for current planning group and set its upper and lower joint limits
  KDL::Chain* chain = new KDL::Chain();
  valid = ik_solvers_[planning_group]->getKDLChain(*chain);

  links_group_name_map_.insert({std::make_pair(chain_start_parent, chain_end), planning_group});

  kdl_chains_.insert({planning_group, chain});
  KDL::JntArray ll, ul;
  valid = valid && ik_solvers_[planning_group]->getKDLLimits(ll, ul);
  kdl_joint_limits_.insert({planning_group, std::make_pair(ll, ul) });
  return valid;
}

bool ToughKinematics::solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                                trajectory_msgs::JointTrajectory& result_traj, float time)
{
  bool success = false;

  std::vector<double> joint_angles;
  success = solve_ik(planning_group, end_effector_pose, joint_angles);

  result_traj.header = std_msgs::Header();
  result_traj.header.frame_id = end_effector_pose.header.frame_id;
  // result_traj.header.seq = rand()%100;

  result_traj.joint_names.clear();
  
  result_traj.points.resize(1);
  result_traj.points.resize(1);
  result_traj.points.front().positions = joint_angles;
  result_traj.points.front().velocities.resize(joint_angles.size());
  result_traj.points.front().effort.resize(joint_angles.size());
  result_traj.points.front().accelerations.resize(joint_angles.size());
  result_traj.points.front().time_from_start = ros::Duration(traj_exec_time_);


  KDL::Chain* current_chain = kdl_chains_[planning_group];
  for (size_t i = 0; i < current_chain->getNrOfJoints(); i++)
  {
    result_traj.joint_names.push_back(current_chain->getSegment(i).getJoint().getName());
    // ROS_INFO("Joint Name: %s, Joint angle: %.2f", current_chain->getSegment(i).getJoint().getName().c_str(),
            //  result_traj.points.front().positions.at(i));
  }
  return success;
}

bool ToughKinematics::solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                                std::vector<double>& result)
{
  joint_names_in_traj_.clear();
    std::vector<double> initial_position;
    
    if(kdl_chains_.find(planning_group) != kdl_chains_.end())
    {
      KDL::Chain* current_chain = kdl_chains_[planning_group];
      double current_joint_angle;
      std::string current_joint_name;
      for (size_t i = 0; i < current_chain->getNrOfJoints(); i++)
      {
        current_joint_name = current_chain->getSegment(i).getJoint().getName();
        current_joint_angle = state_informer_->getJointPosition(current_joint_name);
        initial_position.push_back(current_joint_angle);
      }

      int success = get_IK_joint_angles(planning_group, initial_position, end_effector_pose, result);
      return success >= 0;
    }
    else
    {
      ROS_ERROR("Planning Group %s not found in kdl chain map.", planning_group);
      return false;
    }
}

bool ToughKinematics::add_custom_chain(const std::string& chain_start, const std::string& chain_end)
{
  std::string chain_start_parent = rd_->getParentFrameForJointName(chain_start);
  std::string custom_chain_name = chain_start_parent + "_" + chain_end;
  links_group_name_map_.insert({ std::make_pair(chain_start_parent, chain_end), custom_chain_name });
  return addChainToMap(chain_start_parent, chain_end, custom_chain_name);
}

int ToughKinematics::get_IK_joint_angles(const std::string& planning_group, std::vector<double>& initial_position, const geometry_msgs::PoseStamped& end_effector_pose, std::vector<double>& result)
{
  KDL::JntArray kdl_initial_position, kdl_result;
  vectorToKDLJntArray(initial_position, kdl_initial_position);
  KDL::Frame kdl_end_effector_pose;
  geometry_msgs::Pose goal_pose;
  std::string first_frame = kdl_chains_[planning_group]->getSegment(0).getName();
  state_informer_->transformPose(end_effector_pose.pose, goal_pose, end_effector_pose.header.frame_id.data(),
                                 rd_->getParentFrameForJointName(first_frame));
  poseToKDLFrame(goal_pose, kdl_end_effector_pose);
  if(ik_solvers_.find(planning_group) != ik_solvers_.end())
  {
    int success = ik_solvers_[planning_group]->CartToJnt(kdl_initial_position, kdl_end_effector_pose, kdl_result);
    KDLJntArrayToVector(kdl_result, result);
    return success;
  }
  else
  {
    ROS_ERROR("Planning group %s not initialized for ik solvers", planning_group);
  }
}

bool ToughKinematics::solve_ik(const std::string& chain_start, const std::string& chain_end,
                             const geometry_msgs::PoseStamped& end_effector_pose,
                             trajectory_msgs::JointTrajectory& result, float time)
{
  std::string chain_start_parent = rd_->getParentFrameForJointName(chain_start);
  if(links_group_name_map_.find(std::make_pair(chain_start_parent, chain_end)) == links_group_name_map_.end())
  {
    add_custom_chain(chain_start, chain_end);
  }
  std::string group_name = links_group_name_map_[std::make_pair(chain_start_parent, chain_end)];
  return solve_ik(group_name, end_effector_pose, result);
}

void ToughKinematics::vectorToKDLJntArray(std::vector<double>& vec, KDL::JntArray& kdl_array)
{
  double* ptr = &vec[0];
  Eigen::Map<Eigen::VectorXd> data(ptr, vec.size());
  kdl_array.data = data;
}

void ToughKinematics::KDLJntArrayToVector(KDL::JntArray& kdl_array, std::vector<double>& vec)
{
  // use emplace
  vec = std::vector<double>(kdl_array.data.data(), kdl_array.data.data() + kdl_array.data.size());
}

void ToughKinematics::poseToKDLFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame)
{
  frame.M = KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  frame.p = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);
}

double ToughKinematics::get_planning_time()
{
  return planning_time_;
}

void ToughKinematics::set_planning_time(const double time)
{
  planning_time_ = time;
}

double ToughKinematics::get_traj_exec_time()
{
  return traj_exec_time_;
}

void ToughKinematics::set_traj_exec_time(const double time)
{
  traj_exec_time_ = time;
}