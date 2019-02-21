#include <tough_moveit_planners/taskspace_planner.h>

TaskspacePlanner::TaskspacePlanner(ros::NodeHandle& nh, std::string urdf_param) : nh_(nh)
{
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh);

  if (urdf_param == "")
  {
    urdf_param.assign(rd_->getURDFParameter());
  }

  robot_model_loader::RobotModelLoader robot_model_loader(urdf_param);
  robot_model_ = robot_model_loader.getModel();

  display_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>(TOUGH_COMMON_NAMES::TRAJECTORY_DISPLAY_TOPIC, 1, true);

  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  position_tolerance_ = 0.01;
  angle_tolerance_ = 0.01;
  planning_time_ = 5.0;
  num_planning_attempts_ = 5;
  moveit_robot_state_ = std::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(robot_model_));

  plugin_param_ = TOUGH_COMMON_NAMES::PLANNING_PLUGIN_PARAM;
  loadPlanners();
  updateKDLChains();
}

TaskspacePlanner::~TaskspacePlanner()
{
  planner_instance.reset();
  for (size_t i = 0; i < planning_groups_.size(); i++)
  {
    if (ik_solvers_.count(planning_groups_.at(i)) > 0)
      delete ik_solvers_[planning_groups_.at(i)];
    if (kdl_chains_.count(planning_groups_.at(i)) > 0)
      delete kdl_chains_[planning_groups_.at(i)];
  }
}

bool TaskspacePlanner::getTrajectory(const geometry_msgs::PoseStamped pose_msg, std::string planning_group,
                                     moveit_msgs::RobotTrajectory& output_robot_traj_msg)
{
  planning_interface::MotionPlanRequest req;
  moveit_msgs::MotionPlanResponse response_msg;

  state_informer_->getJointStateMessage(req.start_state.joint_state);

  req.group_name = planning_group;

  // configured planning groups are all upper case. right arm begins with R and left arm begins with L
  std::string ee_frame = planning_group.front() == 'R' ? rd_->getRightEEFrame() : rd_->getLeftEEFrame();

  // ROS_INFO("End effector name : %s", ee_frame.c_str());
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(ee_frame, pose_msg, position_tolerance_, angle_tolerance_);
  req.goal_constraints.push_back(pose_goal);

  req.allowed_planning_time = planning_time_;
  req.num_planning_attempts = num_planning_attempts_;

  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene_, req, res_.error_code_);
  context->solve(res_);
  if (res_.error_code_.val != res_.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return false;
  }

  res_.getMessage(response_msg);

  //     Visualize the trajectory
  displayInRviz(response_msg);

  // Robot state is required for generating RobotTrajectory object
  // RobotTrajectoryObject is required for generating creating IterativeParabolicTimeParameterization
  // IterativeParabolicTimeParameterization is required for generating timestamps for each trajectory point
  // Procedure:   create RobotTrajectory object using the RobotModelPtr and planning group name
  //              create RobotState object using the same RobotModelPtr
  //              After planner generates a new trajectory, update the robot state
  //              update the RobotTrajectory
  //              call computeTimeStamps
  //              convert RobotTrajectory back to moveit_msgs_RobotTrajectory

  robot_trajectory::RobotTrajectory robot_traj = robot_trajectory::RobotTrajectory(robot_model_, planning_group);
  moveit_robot_state_->update();
  robot_traj.setRobotTrajectoryMsg(*moveit_robot_state_, response_msg.trajectory.joint_trajectory);
  timeParameterizer.computeTimeStamps(robot_traj);

  robot_traj.getRobotTrajectoryMsg(response_msg.trajectory);
  output_robot_traj_msg = response_msg.trajectory;
  return true;
}

double TaskspacePlanner::getPositionTolerance() const
{
  return position_tolerance_;
}

void TaskspacePlanner::setPositionTolerance(const double tolerance_pose)
{
  position_tolerance_ = tolerance_pose;
}

double TaskspacePlanner::getAngleTolerance() const
{
  return angle_tolerance_;
}

void TaskspacePlanner::setAngleTolerance(const double tolerance_angle)
{
  angle_tolerance_ = tolerance_angle;
}

std::string TaskspacePlanner::getPlugin() const
{
  return plugin_param_;
}

/// @todo: fix this allow changing theplanner
void TaskspacePlanner::setPlugin(const std::string& plugin_param)
{
  plugin_param_ = plugin_param;
  loadPlugin(plugin_param_);
}

double TaskspacePlanner::getPlanningTime() const
{
  return planning_time_;
}
void TaskspacePlanner::setPlanningTime(const double planning_time)
{
  planning_time_ = planning_time;
}

int TaskspacePlanner::getNumPlanningAttempts() const
{
  return num_planning_attempts_;
}
void TaskspacePlanner::setNumPlanningAttempts(const int num_planning_attempts)
{
  num_planning_attempts_ = num_planning_attempts;
}

void TaskspacePlanner::displayInRviz(const moveit_msgs::MotionPlanResponse& response_msg)
{
  ROS_INFO("Visualizing the trajectory");
  display_trajectory_.trajectory_start = response_msg.trajectory_start;
  display_trajectory_.trajectory.push_back(response_msg.trajectory);
  display_publisher_.publish(display_trajectory_);
}

void TaskspacePlanner::loadPlanners()
{
  ROS_INFO("Loading Planners");

  if (!nh_.getParam(plugin_param_, planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");

  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning"
                                                                                                              "_interfa"
                                                                                                              "ce::"
                                                                                                              "PlannerM"
                                                                                                              "anage"
                                                                                                              "r"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  loadPlugin(planner_plugin_name);
}

void TaskspacePlanner::loadPlugin(const std::string& planner_plugin_name)
{
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));

    if (!planner_instance->initialize(robot_model_, nh_.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
}

bool TaskspacePlanner::updateKDLChains()
{
  std::vector<std::pair<std::string, std::string> > base_frame_EE_pair = {
    { rd_->getPelvisFrame(), rd_->getRightEEFrame() },
    { "", rd_->getRightEEFrame() },
    { rd_->getPelvisFrame(), rd_->getLeftEEFrame() },
    { "", rd_->getLeftEEFrame() }
  };

  bool valid;
  for (size_t i = 0; i < planning_groups_.size(); i++)
  {
    // get the current planning group
    std::string planning_group = planning_groups_.at(i);

    // create IK solver for the current planning group
    ik_solvers_[planning_group] = new TRAC_IK::TRAC_IK(base_frame_EE_pair.at(i).first, base_frame_EE_pair.at(i).second,
                                                       rd_->getURDFParameter(), this->planning_time_);

    // get the KDL chain for current planning group and set its upper and lower joint limits
    KDL::Chain* chain = new KDL::Chain();
    valid = ik_solvers_[planning_group]->getKDLChain(*chain);
    if (chain->getNrOfJoints() == 10)
    {
      // i = 0 and i = 2 are 10 DOF chains. (i + 1) are 7 DOF chains of the same side as i. In the following line we set
      // base_frame_EE_pair.at(i + 1).first as the 3rd element in 10 DOF chain to get the 7 DOF chain. This is required
      // as the current arm frames in the parameter are not sufficient for KDL. it expects parent frame but the
      // parameter stores child frame
      base_frame_EE_pair.at(i + 1).first = chain->getSegment(2).getName();
    }
    kdl_chains_[planning_group] = chain;
    KDL::JntArray ll, ul;
    valid = valid && ik_solvers_[planning_group]->getKDLLimits(ll, ul);
    kdl_joint_limits_[planning_group] = std::make_pair(ll, ul);
    if (!valid)
    {
      return valid;
    }
  }
  return valid;
}

bool TaskspacePlanner::solve_ik(const std::string& planning_group, const geometry_msgs::PoseStamped& end_effector_pose,
                                std::vector<double>& result)
{
  int index = std::distance(planning_groups_.begin(),
                            std::find(planning_groups_.begin(), planning_groups_.end(), planning_group));
  if (index < planning_groups_.size())
  {
    std::vector<double> initial_position_arms, initial_position_chest;
    std::string prefix = TOUGH_COMMON_NAMES::TOPIC_PREFIX + rd_->getRobotName() + "/";
    // 10 DOF joints
    if (index == 0 || index == 2)
    {
      state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::CHEST_JOINT_NAMES_PARAM, initial_position_chest);
    }
    if (planning_group.at(0) == 'R')
    {
      state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM,
                                         initial_position_arms);
    }
    else
    {
      state_informer_->getJointPositions(prefix + TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM,
                                         initial_position_arms);
    }

    initial_position_chest.insert(initial_position_chest.end(), initial_position_arms.begin(),
                                  initial_position_arms.end());
    KDL::JntArray kdl_initial_position, kdl_result;
    vectorToKDLJntArray(initial_position_chest, kdl_initial_position);
    KDL::Frame kdl_end_effector_pose;

    geometry_msgs::Pose goal_pose;
    state_informer_->transformPose(end_effector_pose.pose, goal_pose, end_effector_pose.header.frame_id.data(),
                                   rd_->getPelvisFrame());
    poseToKDLFrame(goal_pose, kdl_end_effector_pose);
    int success = ik_solvers_[planning_group]->CartToJnt(kdl_initial_position, kdl_end_effector_pose, kdl_result);
    KDLJntArrayToVector(kdl_result, result);
    return success >= 0;
  }

  ROS_ERROR("This function allows planning for only the following groups :");
  for (int i = 0; i < planning_groups_.size(); i++)
  {
    ROS_ERROR("\t\t%s", planning_groups_.at(i).c_str());
  }
  return false;
}

void TaskspacePlanner::vectorToKDLJntArray(std::vector<double>& vec, KDL::JntArray& kdl_array)
{
  double* ptr = &vec[0];
  Eigen::Map<Eigen::VectorXd> data(ptr, vec.size());
  kdl_array.data = data;
}

void TaskspacePlanner::KDLJntArrayToVector(KDL::JntArray& kdl_array, std::vector<double>& vec)
{
  // use emplace
  vec = std::vector<double>(kdl_array.data.data(), kdl_array.data.data() + kdl_array.data.size());
}

void TaskspacePlanner::poseToKDLFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame)
{
  frame.M = KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  frame.p = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);
}
