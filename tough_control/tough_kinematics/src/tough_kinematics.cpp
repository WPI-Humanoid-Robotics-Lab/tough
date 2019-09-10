#include <tough_kinematics/tough_kinematics.h>

ToughKinematics::ToughKinematics(ros::NodeHandle& nh, std::string urdf_param) : nh_(nh)
{
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh);

  updateKDLChains();
}

ToughKinematics::~ToughKinematics()
{
}

bool ToughKinematics::updateKDLChains()
{
  bool valid;
  for (size_t i = 0; i < TOUGH_COMMON_NAMES::PLANNING_GROUPS.size(); i++)
  {
    // get the current planning group
    std::string planning_group = TOUGH_COMMON_NAMES::PLANNING_GROUPS.at(i);

    // create IK solver for the current planning group
    ik_solvers_[planning_group] = new TRAC_IK::TRAC_IK(rd_->getParentFrameForMoveGroups(planning_group),
                                                       rd_->getFrameNamesInMoveGroup(planning_group).back(),
                                                       rd_->getURDFParameter(), this->planning_time_);

    // get the KDL chain for current planning group and set its upper and lower joint limits
    KDL::Chain* chain = new KDL::Chain();
    valid = ik_solvers_[planning_group]->getKDLChain(*chain);

    kdl_chains_[planning_group] = chain;
    KDL::JntArray ll, ul;
    valid = valid && ik_solvers_[planning_group]->getKDLLimits(ll, ul);
    kdl_joint_limits_[planning_group] = std::make_pair(ll, ul);
    if (!valid)
    {
      return false;
    }
  }
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
    std::string prefix = TOUGH_COMMON_NAMES::TOPIC_PREFIX + rd_->getRobotName() + "/";
    
    std::string first_frame = rd_->getFrameNamesInMoveGroup(planning_group).front();
    if (planning_group==TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP || planning_group==TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP)
    {
      state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::CHEST_JOINT_NAMES_PARAM, initial_position_chest);
    }
    if (planning_group==TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP || planning_group == TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP)
    {
      state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM,
                                         initial_position_arms);
    }
    else if (planning_group==TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP || planning_group==TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP)
    {
      state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM,
                                         initial_position_arms);
    }
    
    initial_position = initial_position_chest;
    initial_position.insert(initial_position.end(), initial_position_arms.begin(),
                                  initial_position_arms.end());
    KDL::JntArray kdl_initial_position, kdl_result;
    vectorToKDLJntArray(initial_position, kdl_initial_position);
    KDL::Frame kdl_end_effector_pose;

    geometry_msgs::Pose goal_pose;
    state_informer_->transformPose(end_effector_pose.pose, goal_pose, end_effector_pose.header.frame_id.data(),
                                   rd_->getParentFrameForMoveGroups(planning_group));
    poseToKDLFrame(goal_pose, kdl_end_effector_pose);
    int success = ik_solvers_[planning_group]->CartToJnt(kdl_initial_position, kdl_end_effector_pose, kdl_result);
    KDLJntArrayToVector(kdl_result, result);
    return success >= 0;
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

