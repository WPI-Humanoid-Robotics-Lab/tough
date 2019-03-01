#include "tough_common/robot_state.h"

using namespace TOUGH_COMMON_NAMES;
RobotStateInformer *RobotStateInformer::currentObject_ = nullptr;

/* Singleton implementation */
RobotStateInformer *RobotStateInformer::getRobotStateInformer(ros::NodeHandle nh)
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
  nh.getParam(ROBOT_NAME_PARAM, robotName_);

  std::string prefix = TOPIC_PREFIX + robotName_ + OUTPUT_TOPIC_PREFIX;

  jointStateSub_ = nh_.subscribe(prefix + JOINT_STATES_TOPIC, 1, &RobotStateInformer::jointStateCB, this);
  pelvisIMUSub_ = nh_.subscribe(prefix + PELVIS_IMU_TOPIC, 1, &RobotStateInformer::pelvisImuCB, this);
  centerOfMassSub_ = nh_.subscribe(prefix + CENTER_OF_MASS_TOPIC, 1, &RobotStateInformer::centerOfMassCB, this);
  capturePointSub_ = nh_.subscribe(prefix + CAPTURE_POINT_TOPIC, 1, &RobotStateInformer::capturPointCB, this);
  isInDoubleSupportSub_ =
      nh_.subscribe(prefix + DOUBLE_SUPPORT_STATUS_TOPIC, 1, &RobotStateInformer::doubleSupportStatusCB, this);
  leftFootForceSensorSub_ =
      nh_.subscribe(prefix + LEFT_FOOT_FORCE_SENSOR_TOPIC, 1, &RobotStateInformer::leftFootForceSensorCB, this);
  rightFootForceSensorSub_ =
      nh_.subscribe(prefix + RIGHT_FOOT_FORCE_SENSOR_TOPIC, 1, &RobotStateInformer::rightFootForceSensorCB, this);
  leftWristForceSensorSub_ =
      nh_.subscribe(prefix + LEFT_WRIST_FORCE_SENSOR_TOPIC, 1, &RobotStateInformer::leftWristForceSensorCB, this);
  rightWristForceSensorSub_ =
      nh_.subscribe(prefix + RIGHT_WRIST_FORCE_SENSOR_TOPIC, 1, &RobotStateInformer::rightWristForceSensorCB, this);

  ros::Duration(0.2).sleep();
}

RobotStateInformer::~RobotStateInformer()
{
  jointStateSub_.shutdown();
}

void RobotStateInformer::getJointStateMessage(sensor_msgs::JointState &jointState)
{
  jointState = *currentStatePtr_;
}

void RobotStateInformer::jointStateCB(const sensor_msgs::JointStateConstPtr msg)
{
  currentStatePtr_ = msg;
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

void RobotStateInformer::pelvisImuCB(const sensor_msgs::ImuConstPtr msg)
{
  pelvisImuValue_ = msg;
}
void RobotStateInformer::centerOfMassCB(const geometry_msgs::Point32ConstPtr msg)
{
  centerOfMassValue_ = msg;
}
void RobotStateInformer::capturPointCB(const ihmc_msgs::Point2dRosMessageConstPtr msg)
{
  capturePointValue_ = msg;
}

void RobotStateInformer::doubleSupportStatusCB(const std_msgs::Bool &msg)
{
  doubleSupportStatus_ = msg.data;
}
void RobotStateInformer::leftFootForceSensorCB(const geometry_msgs::WrenchStampedConstPtr msg)
{
  footWrenches_[LEFT] = msg;
}
void RobotStateInformer::rightFootForceSensorCB(const geometry_msgs::WrenchStampedConstPtr msg)
{
  footWrenches_[RIGHT] = msg;
}
void RobotStateInformer::leftWristForceSensorCB(const geometry_msgs::WrenchStampedConstPtr msg)
{
  wristWrenches_[LEFT] = msg;
}
void RobotStateInformer::rightWristForceSensorCB(const geometry_msgs::WrenchStampedConstPtr msg)
{
  wristWrenches_[RIGHT] = msg;
}

void RobotStateInformer::getFootWrenches(std::map<RobotSide, geometry_msgs::Wrench> &wrenches)
{
  wrenches[LEFT] = footWrenches_[LEFT]->wrench;
  wrenches[RIGHT] = footWrenches_[RIGHT]->wrench;
}
void RobotStateInformer::getWristWrenches(std::map<RobotSide, geometry_msgs::Wrench> &wrenches)
{
  wrenches[LEFT] = wristWrenches_[LEFT]->wrench;
  wrenches[RIGHT] = wristWrenches_[RIGHT]->wrench;
}

void RobotStateInformer::getFootWrench(const RobotSide side, geometry_msgs::Wrench &wrench)
{
  wrench = footWrenches_[side]->wrench;
}
void RobotStateInformer::getWristWrench(const RobotSide side, geometry_msgs::Wrench &wrench)
{
  wrench = wristWrenches_[side]->wrench;
}

void RobotStateInformer::getFootForce(const RobotSide side, geometry_msgs::Vector3 &force)
{
  force = footWrenches_[side]->wrench.force;
}
void RobotStateInformer::getFootTorque(const RobotSide side, geometry_msgs::Vector3 &torque)
{
  torque = footWrenches_[side]->wrench.torque;
}

void RobotStateInformer::getWristForce(const RobotSide side, geometry_msgs::Vector3 &force)
{
  force = wristWrenches_[side]->wrench.force;
}
void RobotStateInformer::getWristTorque(const RobotSide side, geometry_msgs::Vector3 &torque)
{
  torque = wristWrenches_[side]->wrench.torque;
}

bool RobotStateInformer::isRobotInDoubleSupport()
{
  return doubleSupportStatus_;
}

void RobotStateInformer::getCapturePoint(geometry_msgs::Point &point)
{
  point.x = capturePointValue_->x;
  point.y = capturePointValue_->y;
  point.z = 0.0;
}

void RobotStateInformer::getCenterOfMass(geometry_msgs::Point &point)
{
  point.x = centerOfMassValue_->x;
  point.y = centerOfMassValue_->y;
  point.z = centerOfMassValue_->z;
}

void RobotStateInformer::getPelvisIMUReading(sensor_msgs::Imu &msg)
{
  msg = *pelvisImuValue_;
}

void RobotStateInformer::populateStateMap()
{
  for (size_t i = 0; i < currentStatePtr_->name.size(); ++i)
  {
    RobotState state;
    state.name = currentStatePtr_->name[i];
    state.position = currentStatePtr_->position[i];
    state.velocity = currentStatePtr_->velocity[i];
    state.effort = currentStatePtr_->effort[i];
    currentState_[currentStatePtr_->name[i]] = state;
  }
}
void RobotStateInformer::getJointPositions(std::vector<double> &positions)
{
  positions = currentStatePtr_->position;
}

bool RobotStateInformer::getJointPositions(const std::string &paramName, std::vector<double> &positions)
{
  positions.clear();
  std::vector<std::string> jointNames;
  std::string parameter;
  parseParameter(paramName, parameter);

  if (nh_.getParam(parameter, jointNames))
  {
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    populateStateMap();
    for (auto joint : jointNames)
    {
      positions.push_back((currentState_[joint]).position);
    }
    return true;
  }
  return false;
}

void RobotStateInformer::getJointVelocities(std::vector<double> &velocities)
{
  velocities = currentStatePtr_->velocity;
}

bool RobotStateInformer::getJointVelocities(const std::string &paramName, std::vector<double> &velocities)
{
  velocities.clear();
  std::vector<std::string> jointNames;
  std::string parameter;
  parseParameter(paramName, parameter);

  if (nh_.getParam(parameter, jointNames))
  {
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    populateStateMap();
    for (auto joint : jointNames)
    {
      velocities.push_back((currentState_[joint]).velocity);
    }
    return true;
  }
  return false;
}

void RobotStateInformer::getJointEfforts(std::vector<double> &efforts)
{
  efforts = currentStatePtr_->effort;
}

bool RobotStateInformer::getJointEfforts(const std::string &paramName, std::vector<double> &efforts)
{
  efforts.clear();
  std::vector<std::string> jointNames;
  std::string parameter;
  parseParameter(paramName, parameter);

  if (nh_.getParam(parameter, jointNames))
  {
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    populateStateMap();
    for (auto joint : jointNames)
    {
      efforts.push_back((currentState_[joint]).effort);
    }
    return true;
  }
  return false;
}

double RobotStateInformer::getJointPosition(const std::string &jointName)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  for (int i = 0; i < currentStatePtr_->name.size(); i++)
  {
    if (currentStatePtr_->name.at(i) == jointName)
    {
      return currentStatePtr_->position.at(i);
    }
  }
}

double RobotStateInformer::getJointVelocity(const std::string &jointName)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  for (int i = 0; i < currentStatePtr_->name.size(); i++)
  {
    if (currentStatePtr_->name.at(i) == jointName)
    {
      return currentStatePtr_->velocity.at(i);
    }
  }
}

double RobotStateInformer::getJointEffort(const std::string &jointName)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  for (int i = 0; i < currentStatePtr_->name.size(); i++)
  {
    if (currentStatePtr_->name.at(i) == jointName)
    {
      return currentStatePtr_->effort.at(i);
    }
  }
}

void RobotStateInformer::getJointNames(std::vector<std::string> &jointNames)
{
  jointNames = currentStatePtr_->name;
}

bool RobotStateInformer::getCurrentPose(const std::string &frameName, geometry_msgs::Pose &pose,
                                        const std::string &baseFrame)
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

bool RobotStateInformer::getTransform(const std::string &frameName, tf::StampedTransform &transform,
                                      const std::string &baseFrame)
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

bool RobotStateInformer::transformQuaternion(const geometry_msgs::QuaternionStamped &qt_in,
                                             geometry_msgs::QuaternionStamped &qt_out, const std::string target_frame)
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

bool RobotStateInformer::transformQuaternion(const geometry_msgs::Quaternion &qt_in, geometry_msgs::Quaternion &qt_out,
                                             const std::string &from_frame, const std::string &to_frame)
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

bool RobotStateInformer::transformPoint(const geometry_msgs::PointStamped &pt_in, geometry_msgs::PointStamped &pt_out,
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

bool RobotStateInformer::transformPose(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out,
                                       const std::string &from_frame, const std::string &to_frame)
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

bool RobotStateInformer::transformPose(const geometry_msgs::Pose2D &pose_in, geometry_msgs::Pose2D &pose_out,
                                       const std::string &from_frame, const std::string &to_frame)
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

bool RobotStateInformer::transformPoint(const geometry_msgs::Point &pt_in, geometry_msgs::Point &pt_out,
                                        const std::string &from_frame, const std::string &to_frame)
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

bool RobotStateInformer::transformVector(const geometry_msgs::Vector3Stamped &vec_in,
                                         geometry_msgs::Vector3Stamped &vec_out, const std::string target_frame)
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

bool RobotStateInformer::transformVector(const geometry_msgs::Vector3 &vec_in, geometry_msgs::Vector3 &vec_out,
                                         const std::string &from_frame, const std::string &to_frame)
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
