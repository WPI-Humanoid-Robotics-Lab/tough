#include "tough_common/robot_state.h"

using namespace TOUGH_COMMON_NAMES;
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

RobotStateInformer::RobotStateInformer(ros::NodeHandle nh) : nh_(nh), spinner(2)
{
  rd_ = RobotDescription::getRobotDescription(nh_);
  spinner.start();

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

  initializeClassMembers();
}

RobotStateInformer::~RobotStateInformer()
{
  jointStateSub_.shutdown();
  pelvisIMUSub_.shutdown();
  centerOfMassSub_.shutdown();
  capturePointSub_.shutdown();
  isInDoubleSupportSub_.shutdown();
  leftFootForceSensorSub_.shutdown();
  rightFootForceSensorSub_.shutdown();
  leftWristForceSensorSub_.shutdown();
  rightWristForceSensorSub_.shutdown();
  spinner.stop();
}

void RobotStateInformer::initializeClassMembers()
{
  currentStatePtr_ = sensor_msgs::JointState::Ptr(new sensor_msgs::JointState());
  pelvisImuValue_ = sensor_msgs::Imu::Ptr(new sensor_msgs::Imu());
  centerOfMassValue_ = geometry_msgs::Point32::Ptr(new geometry_msgs::Point32());
  capturePointValue_ = ihmc_msgs::Point2dRosMessage::Ptr(new ihmc_msgs::Point2dRosMessage());
  doubleSupportStatus_ = true;
  footWrenches_[LEFT] = geometry_msgs::WrenchStamped::Ptr(new geometry_msgs::WrenchStamped());
  footWrenches_[RIGHT] = geometry_msgs::WrenchStamped::Ptr(new geometry_msgs::WrenchStamped());
  wristWrenches_[LEFT] = geometry_msgs::WrenchStamped::Ptr(new geometry_msgs::WrenchStamped());
  wristWrenches_[RIGHT] = geometry_msgs::WrenchStamped::Ptr(new geometry_msgs::WrenchStamped());
}
void RobotStateInformer::jointStateCB(const sensor_msgs::JointState::Ptr msg)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  currentStatePtr_ = msg;
}

void RobotStateInformer::pelvisImuCB(const sensor_msgs::Imu::Ptr msg)
{
  pelvisImuValue_ = msg;
}
void RobotStateInformer::centerOfMassCB(const geometry_msgs::Point32::Ptr msg)
{
  centerOfMassValue_ = msg;
}
void RobotStateInformer::capturPointCB(const ihmc_msgs::Point2dRosMessage::Ptr msg)
{
  capturePointValue_ = msg;
}

void RobotStateInformer::doubleSupportStatusCB(const std_msgs::Bool& msg)
{
  doubleSupportStatus_ = msg.data;
}
void RobotStateInformer::leftFootForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg)
{
  footWrenches_[LEFT] = msg;
}
void RobotStateInformer::rightFootForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg)
{
  footWrenches_[RIGHT] = msg;
}
void RobotStateInformer::leftWristForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg)
{
  wristWrenches_[LEFT] = msg;
}
void RobotStateInformer::rightWristForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg)
{
  wristWrenches_[RIGHT] = msg;
}

void RobotStateInformer::getJointStateMessage(sensor_msgs::JointState& jointState)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  jointState = *currentStatePtr_;
}

void RobotStateInformer::getFootWrenches(std::map<RobotSide, geometry_msgs::Wrench>& wrenches)
{
  wrenches[LEFT] = footWrenches_[LEFT]->wrench;
  wrenches[RIGHT] = footWrenches_[RIGHT]->wrench;
}
void RobotStateInformer::getWristWrenches(std::map<RobotSide, geometry_msgs::Wrench>& wrenches)
{
  wrenches[LEFT] = wristWrenches_[LEFT]->wrench;
  wrenches[RIGHT] = wristWrenches_[RIGHT]->wrench;
}

void RobotStateInformer::getFootWrench(const RobotSide side, geometry_msgs::Wrench& wrench)
{
  wrench = footWrenches_[side]->wrench;
}
void RobotStateInformer::getWristWrench(const RobotSide side, geometry_msgs::Wrench& wrench)
{
  wrench = wristWrenches_[side]->wrench;
}

void RobotStateInformer::getFootForce(const RobotSide side, geometry_msgs::Vector3& force)
{
  force = footWrenches_[side]->wrench.force;
}
void RobotStateInformer::getFootTorque(const RobotSide side, geometry_msgs::Vector3& torque)
{
  torque = footWrenches_[side]->wrench.torque;
}

void RobotStateInformer::getWristForce(const RobotSide side, geometry_msgs::Vector3& force)
{
  force = wristWrenches_[side]->wrench.force;
}
void RobotStateInformer::getWristTorque(const RobotSide side, geometry_msgs::Vector3& torque)
{
  torque = wristWrenches_[side]->wrench.torque;
}

bool RobotStateInformer::isRobotInDoubleSupport()
{
  return doubleSupportStatus_;
}

void RobotStateInformer::getCapturePoint(geometry_msgs::Point& point)
{
  point.x = capturePointValue_->x;
  point.y = capturePointValue_->y;
  point.z = 0.0;
}

void RobotStateInformer::getCenterOfMass(geometry_msgs::Point& point)
{
  point.x = centerOfMassValue_->x;
  point.y = centerOfMassValue_->y;
  point.z = centerOfMassValue_->z;
}

void RobotStateInformer::getPelvisIMUReading(sensor_msgs::Imu& msg)
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
void RobotStateInformer::getJointPositions(std::vector<double>& positions)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  positions = currentStatePtr_->position;
}

bool RobotStateInformer::getJointPositions(const std::string& paramName, std::vector<double>& positions)
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

void RobotStateInformer::getJointVelocities(std::vector<double>& velocities)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  velocities = currentStatePtr_->velocity;
}

bool RobotStateInformer::getJointVelocities(const std::string& paramName, std::vector<double>& velocities)
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

void RobotStateInformer::getJointEfforts(std::vector<double>& efforts)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  efforts = currentStatePtr_->effort;
}

bool RobotStateInformer::getJointEfforts(const std::string& paramName, std::vector<double>& efforts)
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

double RobotStateInformer::getJointPosition(const std::string& jointName)
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

double RobotStateInformer::getJointVelocity(const std::string& jointName)
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

double RobotStateInformer::getJointEffort(const std::string& jointName)
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

void RobotStateInformer::getJointNames(std::vector<std::string>& jointNames)
{
  std::lock_guard<std::mutex> guard(currentStateMutex_);
  jointNames = currentStatePtr_->name;
}

bool RobotStateInformer::getCurrentPose(const std::string& frameName, geometry_msgs::Pose& pose,
                                        const std::string& baseFrame)
{
  tf::StampedTransform origin;
  if (getTransform(frameName, origin, baseFrame))
  {
    tf::pointTFToMsg(origin.getOrigin(), pose.position);
    tf::quaternionTFToMsg(origin.getRotation(), pose.orientation);
    return true;
  }
  else
  {
    false;
  }
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
    listener_.waitForTransform(qt_in.header.frame_id, target_frame, ros::Time(0), ros::Duration(2));
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
  if (transformQuaternion(in, out, to_frame))
  {
    qt_out = out.quaternion;
    return true;
  }
  return false;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::PointStamped& pt_in, geometry_msgs::PointStamped& pt_out,
                                        const std::string target_frame)
{
  try
  {
    listener_.waitForTransform(pt_in.header.frame_id, target_frame, ros::Time(0), ros::Duration(2));
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

bool RobotStateInformer::transformPoint(const geometry_msgs::Point& pt_in, geometry_msgs::Point& pt_out,
                                        const std::string& from_frame, const std::string& to_frame)
{
  geometry_msgs::PointStamped in, out;
  in.point = pt_in;
  in.header.frame_id = from_frame;
  if (transformPoint(in, out, to_frame))
  {
    pt_out = out.point;
    return true;
  }
  return false;
}

bool RobotStateInformer::transformPose(const geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped& pose_out,
                                       const std::string& to_frame)
{
  try
  {
    listener_.waitForTransform(pose_in.header.frame_id, to_frame, ros::Time(0), ros::Duration(2));
    listener_.transformPose(to_frame, pose_in, pose_out);
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
  if (transformPose(in, out, to_frame))
  {
    pose_out = out.pose;
    return true;
  }
  return false;
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
    listener_.waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(2));
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

bool RobotStateInformer::transformVector(const geometry_msgs::Vector3Stamped& vec_in,
                                         geometry_msgs::Vector3Stamped& vec_out, const std::string target_frame)
{
  try
  {
    listener_.waitForTransform(vec_in.header.frame_id, target_frame, ros::Time(0), ros::Duration(2));
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
  if (transformVector(in, out, to_frame))
  {
    vec_out = out.vector;
    return true;
  }
  return false;
}
