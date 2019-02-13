#include "tough_common/robot_state.h"

RobotStateInformer* RobotStateInformer::currentObject_ = nullptr;

/* Singleton implementation */
RobotStateInformer* RobotStateInformer::getRobotStateInformer(ros::NodeHandle nh)
{
  // check if an object of this class already exists, if not create one
  if (RobotStateInformer::currentObject_ == nullptr)
  {
    static RobotStateInformer obj(nh);
    currentObject_ = &obj;
  }
  return currentObject_;
}

RobotStateInformer::RobotStateInformer(ros::NodeHandle nh) : nh_(nh)
{
  rd_ = RobotDescription::getRobotDescription(nh_);
  nh.getParam(TOUGH_COMMON_NAMES::ROBOT_NAME_PARAM, robotName_);

  jointStateSub_ = nh_.subscribe(TOUGH_COMMON_NAMES::TOPIC_PREFIX + robotName_ +
                                     TOUGH_COMMON_NAMES::OUTPUT_TOPIC_PREFIX + TOUGH_COMMON_NAMES::JOINT_STATES_TOPIC,
                                 1, &RobotStateInformer::jointStateCB, this);
  ros::Duration(0.2).sleep();
}

RobotStateInformer::~RobotStateInformer()
{
  jointStateSub_.shutdown();
}

void RobotStateInformer::getJointStateMessage(sensor_msgs::JointState& jointState)
{
  jointState.name.clear();
  jointState.position.clear();
  jointState.velocity.clear();
  jointState.effort.clear();
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  for (auto it = currentState_.begin(); it != currentState_.end(); ++it)
  {
    jointState.name.push_back(it->first);
    jointState.position.push_back(it->second.position);
    jointState.velocity.push_back(it->second.velocity);
    jointState.effort.push_back(it->second.effort);
  }
  jointState.header = std_msgs::Header();
}

void RobotStateInformer::jointStateCB(const sensor_msgs::JointStatePtr msg)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    RobotState state;
    state.name = msg->name[i];
    state.position = msg->position[i];
    state.velocity = msg->velocity[i];
    state.effort = msg->effort[i];
    currentState_[msg->name[i]] = state;
  }
}

void RobotStateInformer::getJointPositions(std::vector<double>& positions)
{
  positions.clear();
  std::lock_guard<std::mutex> guard(currentStateMutex_);

  for (auto it = currentState_.begin(); it != currentState_.end(); ++it)
  {
    positions.push_back(it->second.position);
  }
}

bool RobotStateInformer::getJointPositions(const std::string& paramName, std::vector<double>& positions)
{
  positions.clear();
  std::vector<std::string> jointNames;
  std::string parameter;
  parseParameter(paramName, parameter);

  std::lock_guard<std::mutex> guard(currentStateMutex_);
  if (nh_.getParam(parameter, jointNames))
  {
    for (auto joint : jointNames)
    {
      positions.push_back((currentState_[joint]).position);
    }
    return true;
  }
  return false;
}

void RobotStateInformer::getJointVelocities(std::vector<double>& velocities)
{
  velocities.clear();
  std::lock_guard<std::mutex> guard(currentStateMutex_);

  for (auto it = currentState_.begin(); it != currentState_.end(); ++it)
  {
    velocities.push_back(it->second.velocity);
  }
}

bool RobotStateInformer::getJointVelocities(const std::string& paramName, std::vector<double>& velocities)
{
  velocities.clear();
  std::vector<std::string> jointNames;
  std::string parameter;
  parseParameter(paramName, parameter);

  std::lock_guard<std::mutex> guard(currentStateMutex_);
  if (nh_.getParam(parameter, jointNames))
  {
    for (auto joint : jointNames)
    {
      velocities.push_back((currentState_[joint]).velocity);
    }
    return true;
  }
  return false;
}

void RobotStateInformer::getJointEfforts(std::vector<double>& efforts)
{
  efforts.clear();
  std::lock_guard<std::mutex> guard(currentStateMutex_);

  for (auto it = currentState_.begin(); it != currentState_.end(); ++it)
  {
    efforts.push_back(it->second.effort);
  }
}

bool RobotStateInformer::getJointEfforts(const std::string& paramName, std::vector<double>& efforts)
{
  efforts.clear();
  std::vector<std::string> jointNames;
  std::string parameter;
  parseParameter(paramName, parameter);

  std::lock_guard<std::mutex> guard(currentStateMutex_);
  if (nh_.getParam(parameter, jointNames))
  {
    for (auto joint : jointNames)
    {
      efforts.push_back((currentState_[joint]).effort);
    }
    return true;
  }
  return false;
}

double RobotStateInformer::getJointPosition(const std::string& jointName)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  return (currentState_[jointName]).position;
}

double RobotStateInformer::getJointVelocity(const std::string& jointName)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  return (currentState_[jointName]).velocity;
}

double RobotStateInformer::getJointEffort(const std::string& jointName)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  return (currentState_[jointName]).effort;
}

void RobotStateInformer::getJointNames(std::vector<std::string>& jointNames)
{
  jointNames.clear();
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  for (auto i : currentState_)
  {
    jointNames.push_back(i.first);
  }
}

bool RobotStateInformer::getCurrentPose(const std::string& frameName, geometry_msgs::Pose& pose,
                                        const std::string& baseFrame)
{
  tf::StampedTransform origin;

  try
  {
    listener_.waitForTransform(baseFrame, frameName, ros::Time(0), ros::Duration(2));
    listener_.lookupTransform(baseFrame, frameName, ros::Time(0), origin);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }

  tf::pointTFToMsg(origin.getOrigin(), pose.position);
  tf::quaternionTFToMsg(origin.getRotation(), pose.orientation);

  return true;
}

bool RobotStateInformer::getTransform(const std::string& frameName, tf::StampedTransform& transform,
                                      const std::string& baseFrame)
{
  try
  {
    listener_.waitForTransform(baseFrame, frameName, ros::Time(0), ros::Duration(2));
    listener_.lookupTransform(baseFrame, frameName, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  return true;
}

bool RobotStateInformer::transformQuaternion(const geometry_msgs::QuaternionStamped& qt_in,
                                             geometry_msgs::QuaternionStamped& qt_out, const std::string target_frame)
{
  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformQuaternion(target_frame, qt_in, qt_out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  return true;
}

bool RobotStateInformer::transformQuaternion(const geometry_msgs::Quaternion& qt_in, geometry_msgs::Quaternion& qt_out,
                                             const std::string& from_frame, const std::string& to_frame)
{
  geometry_msgs::QuaternionStamped in, out;
  in.quaternion = qt_in;
  in.header.frame_id = from_frame;
  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformQuaternion(to_frame, in, out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  qt_out = out.quaternion;
  return true;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::PointStamped& pt_in, geometry_msgs::PointStamped& pt_out,
                                        const std::string target_frame)
{
  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformPoint(target_frame, pt_in, pt_out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  return true;
}

bool RobotStateInformer::transformPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out,
                                       const std::string& from_frame, const std::string& to_frame)
{
  geometry_msgs::PoseStamped in, out;
  in.header.frame_id = from_frame;
  in.header.stamp = ros::Time(0);
  in.pose = pose_in;
  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformPose(to_frame, in, out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }

  pose_out = out.pose;
  return true;
}

bool RobotStateInformer::transformPose(const geometry_msgs::Pose2D& pose_in, geometry_msgs::Pose2D& pose_out,
                                       const std::string& from_frame, const std::string& to_frame)
{
  geometry_msgs::PoseStamped in, out;
  in.header.frame_id = from_frame;
  in.header.stamp = ros::Time(0);
  in.pose.position.x = pose_in.x;
  in.pose.position.y = pose_in.y;
  in.pose.position.z = 0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose_in.theta), in.pose.orientation);

  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformPose(to_frame, in, out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }

  pose_out.x = out.pose.position.x;
  pose_out.y = out.pose.position.y;
  pose_out.theta = tf::getYaw(out.pose.orientation);

  return true;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::Point& pt_in, geometry_msgs::Point& pt_out,
                                        const std::string& from_frame, const std::string& to_frame)
{
  geometry_msgs::PointStamped stmp_pt_in, stmp_pt_out;
  stmp_pt_in.header.frame_id = from_frame;
  stmp_pt_in.point = pt_in;

  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformPoint(to_frame, stmp_pt_in, stmp_pt_out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  pt_out = stmp_pt_out.point;
  return true;
}

bool RobotStateInformer::transformVector(const geometry_msgs::Vector3Stamped& vec_in,
                                         geometry_msgs::Vector3Stamped& vec_out, const std::string target_frame)
{
  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformVector(target_frame, vec_in, vec_out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  return true;
}

bool RobotStateInformer::transformVector(const geometry_msgs::Vector3& vec_in, geometry_msgs::Vector3& vec_out,
                                         const std::string& from_frame, const std::string& to_frame)
{
  geometry_msgs::Vector3Stamped in, out;
  in.vector = vec_in;
  in.header.frame_id = from_frame;
  try
  {
    listener_.waitForTransform(rd_->getPelvisFrame(), rd_->getWorldFrame(), ros::Time(0), ros::Duration(2));
    listener_.transformVector(to_frame, in, out);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::spinOnce();
    return false;
  }
  vec_out = out.vector;
  return true;
}
