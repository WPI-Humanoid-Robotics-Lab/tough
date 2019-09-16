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
  for (size_t i = 0; i < PLANNING_GROUPS.size(); i++)
  {
    // get the current planning group
    std::string planning_group = PLANNING_GROUPS.at(i);

    valid = addChainToMap(rd_->getParentFrameForMoveGroups(planning_group),
                          rd_->getFrameNamesInMoveGroup(planning_group).back(), planning_group);

    // create IK solver for the current planning group
    //   ik_solvers_[planning_group] = new TRAC_IK::TRAC_IK(rd_->getParentFrameForMoveGroups(planning_group),
    //                                                      rd_->getFrameNamesInMoveGroup(planning_group).back(),
    //                                                      rd_->getURDFParameter(), this->planning_time_);

    //   // get the KDL chain for current planning group and set its upper and lower joint limits
    //   KDL::Chain* chain = new KDL::Chain();
    // valid = ik_solvers_[planning_group]->getKDLChain(*chain);

    //   kdl_chains_[planning_group] = chain;
    //   KDL::JntArray ll, ul;
    //   valid = valid && ik_solvers_[planning_group]->getKDLLimits(ll, ul);
    //   kdl_joint_limits_[planning_group] = std::make_pair(ll, ul);
    //   if (!valid)
    //   {
    //     return false;
    //   }
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

  kdl_chains_[planning_group] = chain;
  KDL::JntArray ll, ul;
  valid = valid && ik_solvers_[planning_group]->getKDLLimits(ll, ul);
  kdl_joint_limits_[planning_group] = std::make_pair(ll, ul);
  return valid;
}

bool ToughKinematics::solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                                trajectory_msgs::JointTrajectory& result_traj, float time)
{
  bool success = false;

  std::vector<double> joint_angles;
  success = solve_ik(planning_group, end_effector_pose, joint_angles);

  result_traj.header = std_msgs::Header();
  result_traj.points.resize(1);
  result_traj.joint_names.clear();
  result_traj.points.resize(1);
  result_traj.points.front().positions = joint_angles;
  result_traj.points.front().velocities.resize(joint_angles.size());
  result_traj.points.front().effort.resize(joint_angles.size());
  result_traj.points.front().accelerations.resize(joint_angles.size());
  result_traj.points.front().time_from_start = ros::Duration(time);

  KDL::Chain* current_chain = kdl_chains_[planning_group];
  for (size_t i = 0; i < current_chain->getNrOfJoints(); i++)
  {
    result_traj.joint_names.push_back(current_chain->getSegment(i).getJoint().getName());
    ROS_INFO("Joint Name: %s, Joint angle: %.2f", current_chain->getSegment(i).getJoint().getName().c_str(),
             result_traj.points.front().positions.at(i));
  }
  return success;
}

bool ToughKinematics::solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                                std::vector<double>& result)
{
  joint_names_in_traj_.clear();
    std::vector<double> initial_position_arms, initial_position_chest, initial_position;
    std::vector<std::string> joint_names_in_group;
    std::string prefix = TOUGH_COMMON_NAMES::TOPIC_PREFIX + rd_->getRobotName() + "/";

    // std::string first_frame = rd_->getFrameNamesInMoveGroup(planning_group).front();
    // if (planning_group==TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP || planning_group==TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP)
    // {
    //   state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::CHEST_JOINT_NAMES_PARAM, initial_position_chest);
    // }
    // if (planning_group==TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP || planning_group == TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP)
    // {
    //   state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM,
    //                                      initial_position_arms);
    // }
    // else if (planning_group==TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP || planning_group==TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP)
    // {
    //   state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM,
    //                                      initial_position_arms);
    // }
    // // state_informer_->getJointPositions()
    // initial_position = initial_position_chest;
    // initial_position.insert(initial_position.end(), initial_position_arms.begin(),
    //                               initial_position_arms.end());
    
    KDL::Chain* current_chain = kdl_chains_[planning_group];
    double current_joint_angle;
    std::string current_joint_name;
    for (size_t i = 0; i < current_chain->getNrOfJoints(); i++)
    {
      current_joint_name = current_chain->getSegment(i).getJoint().getName();
      // joint_names_in_group.push_back(current_chain->getSegment(i).getJoint().getName());
      // ROS_INFO("Joint Name: %s, Joint angle: %.2f", current_chain->getSegment(i).getJoint().getName().c_str(),
      //         result_traj.points.front().positions.at(i));
      current_joint_angle = state_informer_->getJointPosition(current_joint_name);
      initial_position.push_back(current_joint_angle);
    }
    int success = get_IK_joint_angles(planning_group, initial_position, end_effector_pose, result);
    return success >= 0;
}

bool ToughKinematics::add_custom_chain(const std::string& chain_start, const std::string& chain_end)
{
  std::string custom_chain_name = chain_name_prefix + std::to_string(custom_chain_number_);
  custom_chain_start_.push_back(chain_start);
  custom_chain_end_.push_back(chain_end);
  return addChainToMap(chain_start, chain_end, custom_chain_name);
}

int ToughKinematics::get_IK_joint_angles(const std::string& planning_group, std::vector<double>& initial_position, const geometry_msgs::PoseStamped& end_effector_pose, std::vector<double>& result)
{
  KDL::JntArray kdl_initial_position, kdl_result;
  vectorToKDLJntArray(initial_position, kdl_initial_position);
  KDL::Frame kdl_end_effector_pose;
  geometry_msgs::Pose goal_pose;
  std::string first_frame = kdl_chains_[planning_group]->getSegment(0).getJoint().getName();
  state_informer_->transformPose(end_effector_pose.pose, goal_pose, end_effector_pose.header.frame_id.data(),
                                 rd_->getParentFrameForJointName(first_frame));
  poseToKDLFrame(goal_pose, kdl_end_effector_pose);
  int success = ik_solvers_[planning_group]->CartToJnt(kdl_initial_position, kdl_end_effector_pose, kdl_result);
  KDLJntArrayToVector(kdl_result, result);
  return success;
}

bool ToughKinematics::solve_custom_chain_ik(const std::string& chain_start, const std::string& chain_end,
                             const geometry_msgs::PoseStamped& end_effector_pose,
                             trajectory_msgs::JointTrajectory& result, float time)
{
  int index_of_chain = get_index_of_chain(chain_start, chain_end);
  if(index_of_chain == -1)
    return false;
  std::string custom_chain_name = chain_name_prefix + std::to_string(index_of_chain);
  return solve_ik(custom_chain_name, end_effector_pose, result);
}

int ToughKinematics::get_index_of_chain(const std::string& chain_start, const std::string& chain_end)
{
  std::vector<std::string>::iterator chain_start_iterator, chain_end_iterator;
  for (int i = 0; i < custom_chain_start_.size(); ++i)
    if (custom_chain_start_.at(i) != chain_start && custom_chain_end_.at(i) != chain_end)
      return i;
  return -1;
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

