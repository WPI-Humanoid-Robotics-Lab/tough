/*
 * Copyright 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/rendering/Camera.hh>
#include <sensor_msgs/Imu.h>

#include "srcsim/SRCMultiSenseSLPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SRCMultiSenseSL)

/////////////////////////////////////////////////
SRCMultiSenseSL::SRCMultiSenseSL()
{
  /// \todo: hardcoded for now, make them into plugin parameters
  this->spindlePID.Init(0.03, 0.30, 0.00001, 1., -1., 10.0, -10.0);
  this->spindleOn = true;
  this->spindleSpeed = 0;
  this->spindleMaxRPM = 50.0;
  this->spindleMinRPM = 0;
  this->multiCameraExposureTime = 0.001;
  this->multiCameraGain = 1.0;
  // the parent link of the head_imu_sensor ends up being upperNeckPitchLink
  // after fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_sensor block.
  this->imuLinkName = "upperNeckPitchLink";

  // change default imager mode to 1 (1Hz ~ 30Hz)
  // in simulation, we are using 800X800 pixels @30Hz
  this->imagerMode = 1;
  this->rosNamespace = "/multisense";

  this->pmq = new PubMultiQueue();
}

/////////////////////////////////////////////////
SRCMultiSenseSL::~SRCMultiSenseSL()
{
  delete this->pmq;
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->valModel = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;

  ROS_DEBUG("Loading MultiSense ROS node.");

  this->lastTime = this->world->GetSimTime();

  // Get imu link
  this->imuLink = this->valModel->GetLink(this->imuLinkName);
  if (!this->imuLink)
    gzerr << "IMU link name[" << this->imuLinkName << "] not found\n";

  // Get sensors
  this->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName() + "::" + this->imuLink->GetScopedName() + "::"
        "head_imu_sensor"));
  if (!this->imuSensor)
    gzerr << "head_imu_sensor not found\n" << "\n";

  // \todo: add ros topic / service to reset imu (imuReferencePose, etc.)
  this->spindleLink = this->valModel->GetLink("valkyrie::hokuyo_link");
  if (!this->spindleLink)
  {
    gzerr << "spindle link not found, plugin will stop loading\n";
    return;
  }

  this->spindleJoint = this->valModel->GetJoint("valkyrie::hokuyo_joint");
  if (!this->spindleJoint)
  {
    gzerr << "spindle joint not found, plugin will stop loading\n";
    return;
  }

  // publish joint states for spindle joint
  this->jointStates.name.resize(1);
  this->jointStates.position.resize(1);
  this->jointStates.velocity.resize(1);
  this->jointStates.effort.resize(1);

  // sensors::Sensor_V s = sensors::SensorManager::Instance()->GetSensors();
  // for (sensors::Sensor_V::iterator siter = s.begin();
  //                                  siter != s.end(); ++siter)
  //   gzerr << (*siter)->GetName() << "\n";

  this->multiCameraSensor =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(
        sensors::SensorManager::Instance()->GetSensor("stereo_camera"));

  if (!this->multiCameraSensor)
    gzerr << "multicamera sensor not found";

  // get default frame rate
# if GAZEBO_MAJOR_VERSION >= 7
  this->multiCameraFrameRate = this->multiCameraSensor->UpdateRate();
# else
  this->multiCameraFrameRate = this->multiCameraSensor->GetUpdateRate();
# endif

  if (!sensors::SensorManager::Instance()->GetSensor("head_hokuyo_sensor"))
    gzerr << "laser sensor not found";

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&SRCMultiSenseSL::LoadThread, this));
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::LoadThread()
{
  // create ros node
  this->rosnode_ = new ros::NodeHandle("");

  // publish multi queue
  this->pmq->startServiceThread();

  this->rosNamespace = "/multisense";

  // ros publications
  // publish joint states for tf (robot state publisher)
  this->pubJointStatesQueue = this->pmq->addPub<sensor_msgs::JointState>();
  this->pubJointStates = this->rosnode_->advertise<sensor_msgs::JointState>(
    this->rosNamespace + "/joint_states", 10);

  // publish imu data
  this->pubImuQueue = this->pmq->addPub<sensor_msgs::Imu>();
  this->pubImu =
    this->rosnode_->advertise<sensor_msgs::Imu>(
      this->rosNamespace + "/imu", 10);

  // ros subscription
  ros::SubscribeOptions set_spindle_speed_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_spindle_speed", 100,
    boost::bind(static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SRCMultiSenseSL::SetSpindleSpeed), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_sub_ =
    this->rosnode_->subscribe(set_spindle_speed_so);

  /// for deprecation from ~/multisense[_sl]/fps to ~/multisense[_sl]/set_fps
  /// per issue 272
  ros::SubscribeOptions set_multi_camera_frame_rate_so_old =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/fps", 100,
    boost::bind(static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SRCMultiSenseSL::SetMultiCameraFrameRateOld), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_frame_rate_sub_old_ =
    this->rosnode_->subscribe(set_multi_camera_frame_rate_so_old);

  ros::SubscribeOptions set_multi_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_fps", 100,
    boost::bind(static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SRCMultiSenseSL::SetMultiCameraFrameRate), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_multi_camera_frame_rate_so);

  /* FIXME currently this causes simulation to crash,
  ros::SubscribeOptions set_multi_camera_resolution_so =
    ros::SubscribeOptions::create<std_msgs::Int32>(
    this->rosNamespace + "/set_camera_resolution_mode", 100,
    boost::bind(static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Int32::ConstPtr&)>(
        &SRCMultiSenseSL::SetMultiCameraResolution), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_resolution_sub_ =
    this->rosnode_->subscribe(set_multi_camera_resolution_so);
  */

  /* not implemented, not supported
  ros::SubscribeOptions set_spindle_state_so =
    ros::SubscribeOptions::create<std_msgs::Bool>(
    this->rosNamespace + "/set_spindle_state", 100,
    boost::bind( static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Bool::ConstPtr&)>(
        &SRCMultiSenseSL::SetSpindleState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_sub_ =
    this->rosnode_->subscribe(set_spindle_state_so);

  ros::SubscribeOptions set_multi_camera_exposure_time_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_camera_exposure_time", 100,
    boost::bind( static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SRCMultiSenseSL::SetMultiCameraExposureTime),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_exposure_time_sub_ =
    this->rosnode_->subscribe(set_multi_camera_exposure_time_so);

  ros::SubscribeOptions set_multi_camera_gain_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_camera_gain", 100,
    boost::bind( static_cast<void (SRCMultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &SRCMultiSenseSL::SetMultiCameraGain),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_gain_sub_ =
    this->rosnode_->subscribe(set_multi_camera_gain_so);
  */

  /// \todo: waiting for gen_srv to be implemented (issue #37)
  /* Advertise services on the custom queue
  std::string set_spindle_speed_service_name(
    this->rosNamespace + "/set_spindle_speed");
  ros::AdvertiseServiceOptions set_spindle_speed_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_speed_service_name,
      boost::bind(&SRCMultiSenseSL::SetSpindleSpeed,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_service_ =
    this->rosnode_->advertiseService(set_spindle_speed_aso);

  std::string set_spindle_state_service_name(
    this->rosNamespace + "/set_spindle_state");
  ros::AdvertiseServiceOptions set_spindle_state_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_state_service_name,
      boost::bind(&SRCMultiSenseSL::SetSpindleState,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_service_ =
    this->rosnode_->advertiseService(set_spindle_state_aso);
  */

  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&SRCMultiSenseSL::QueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&SRCMultiSenseSL::UpdateStates, this));
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  // get imu data from imu link
  if (this->imuSensor)
  {
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = this->imuLinkName;
    imuMsg.header.stamp = ros::Time(curTime.Double());

    // compute angular rates
    {
# if GAZEBO_MAJOR_VERSION >= 7
      math::Vector3 wLocal = this->imuSensor->AngularVelocity();
# else
      math::Vector3 wLocal = this->imuSensor->GetAngularVelocity();
# endif
      imuMsg.angular_velocity.x = wLocal.x;
      imuMsg.angular_velocity.y = wLocal.y;
      imuMsg.angular_velocity.z = wLocal.z;
    }

    // compute acceleration
    {
# if GAZEBO_MAJOR_VERSION >= 7
      math::Vector3 accel = this->imuSensor->LinearAcceleration();
# else
      math::Vector3 accel = this->imuSensor->GetLinearAcceleration();
# endif
      imuMsg.linear_acceleration.x = accel.x;
      imuMsg.linear_acceleration.y = accel.y;
      imuMsg.linear_acceleration.z = accel.z;
    }

    // compute orientation
    {
      math::Quaternion imuRot =
# if GAZEBO_MAJOR_VERSION >= 7
        this->imuSensor->Orientation();
# else
        this->imuSensor->GetOrientation();
# endif
      imuMsg.orientation.x = imuRot.x;
      imuMsg.orientation.y = imuRot.y;
      imuMsg.orientation.z = imuRot.z;
      imuMsg.orientation.w = imuRot.w;
    }

    this->pubImuQueue->push(imuMsg, this->pubImu);
  }

  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->jointStates.name[0] = this->spindleJoint->GetName();
    this->jointStates.position[0] = this->spindleJoint->GetAngle(0).Radian();
    this->jointStates.velocity[0] = this->spindleJoint->GetVelocity(0);
    this->jointStates.effort[0] = 0;

    if (this->spindleOn)
    {
      // PID control (velocity) spindle
      double spindleError = this->spindleJoint->GetVelocity(0)
                          - this->spindleSpeed;
      double spindleCmd = this->spindlePID.Update(spindleError, dt);
      this->spindleJoint->SetForce(0, spindleCmd);

      this->jointStates.effort[0] = spindleCmd;

      this->lastTime = curTime;
    }
    else
    {
      this->spindlePID.Reset();
    }
    this->pubJointStatesQueue->push(this->jointStates, this->pubJointStates);
  }
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
bool SRCMultiSenseSL::SetSpindleSpeed(std_srvs::Empty::Request &/*_req*/,
                                   std_srvs::Empty::Response &/*res*/)
{
  return true;
}

/////////////////////////////////////////////////
bool SRCMultiSenseSL::SetSpindleState(std_srvs::Empty::Request &/*_req*/,
                                   std_srvs::Empty::Response &/*_res*/)
{
  return true;
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg)
{
  this->spindleSpeed = static_cast<double>(_msg->data);
  if (this->spindleSpeed > this->spindleMaxRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMaxRPM * 2.0*M_PI / 60.0;
  else if (this->spindleSpeed < this->spindleMinRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMinRPM * 2.0*M_PI / 60.0;
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetSpindleState(const std_msgs::Bool::ConstPtr &_msg)
{
  this->spindleOn = static_cast<double>(_msg->data);
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetMultiCameraFrameRateOld(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  ROS_WARN("Frame rate was modified but the ros topic ~/mutlisense_sl/fps"
           " has been replaced by ~/mutlisense_sl/set_fps per issue 272.");
  this->SetMultiCameraFrameRate(_msg);
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetMultiCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  // limit frame rate to what is capable
  this->multiCameraFrameRate = static_cast<double>(_msg->data);

  // FIXME: Hardcoded lower limit on all resolution
  if (this->multiCameraFrameRate < 1.0)
  {
    ROS_INFO("Camera rate cannot be below 1Hz at any resolution\n");
    this->multiCameraFrameRate = 1.0;
  }

  // FIXME: Hardcoded upper limit.  Need to switch rates between modes.
  if (this->imagerMode == 0)
  {
    if (this->multiCameraFrameRate > 15.0)
    {
      ROS_INFO("Camera rate cannot be above 15Hz at this resolution\n");
      this->multiCameraFrameRate = 15.0;
    }
  }
  else if (this->imagerMode == 1)
  {
    if (this->multiCameraFrameRate > 30.0)
    {
      ROS_INFO("Camera rate cannot be above 30Hz at this resolution\n");
      this->multiCameraFrameRate = 30.0;
    }
  }
  else if (this->imagerMode == 2)
  {
    if (this->multiCameraFrameRate > 60.0)
    {
      ROS_INFO("Camera rate cannot be above 60Hz at this resolution\n");
      this->multiCameraFrameRate = 60.0;
    }
  }
  else if (this->imagerMode == 3)
  {
    if (this->multiCameraFrameRate > 70.0)
    {
      ROS_INFO("Camera rate cannot be above 70Hz at this resolution\n");
      this->multiCameraFrameRate = 70.0;
    }
  }
  else
  {
    ROS_ERROR("MultiSense SL internal state error (%d)", this->imagerMode);
  }

  this->multiCameraSensor->SetUpdateRate(this->multiCameraFrameRate);
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetMultiCameraResolution(
  const std_msgs::Int32::ConstPtr &_msg)
{
  /// see MultiSenseSLPlugin.h for available modes
  if (_msg->data < 0 || _msg->data > 3)
  {
    ROS_WARN("set_camera_resolution_mode must"
              " be between 0 - 3:\n"
              "  0 - 2MP (2048*1088) @ up to 15 fps\n"
              "  1 - 1MP (2048*544) @ up to 30 fps\n"
              "  2 - 0.5MP (1024*544) @ up to 60 fps (default)\n"
              "  3 - VGA (640*480) @ up to 70 fps\n");
    return;
  }

  this->imagerMode = _msg->data;

  unsigned int width = 640;
  unsigned int height = 480;
  if (this->imagerMode == 0)
  {
    width = 2048;
    height = 1088;
    if (this->multiCameraFrameRate > 15)
    {
      ROS_INFO("Reducing frame rate to 15Hz.");
      this->multiCameraFrameRate = 15.0;
    }
  }
  else if (this->imagerMode == 1)
  {
    width = 2048;
    height = 544;
    if (this->multiCameraFrameRate > 30)
    {
      ROS_INFO("Reducing frame rate to 30Hz.");
      this->multiCameraFrameRate = 30.0;
    }
  }
  else if (this->imagerMode == 2)
  {
    width = 1024;
    height = 544;
    if (this->multiCameraFrameRate > 60)
    {
      ROS_INFO("Reducing frame rate to 60Hz.");
      this->multiCameraFrameRate = 60.0;
    }
  }
  else if (this->imagerMode == 3)
  {
    width = 640;
    height = 480;
    if (this->multiCameraFrameRate > 70)
    {
      ROS_INFO("Reducing frame rate to 70Hz.");
      this->multiCameraFrameRate = 70.0;
    }
  }

  this->multiCameraSensor->SetUpdateRate(this->multiCameraFrameRate);

# if GAZEBO_MAJOR_VERSION >= 7
  for (unsigned int i = 0; i < this->multiCameraSensor->CameraCount(); ++i)
  {
    this->multiCameraSensor->Camera(i)->SetImageWidth(width);
    this->multiCameraSensor->Camera(i)->SetImageHeight(height);
  }
# else
  for (unsigned int i = 0; i < this->multiCameraSensor->GetCameraCount(); ++i)
  {
    this->multiCameraSensor->GetCamera(i)->SetImageWidth(width);
    this->multiCameraSensor->GetCamera(i)->SetImageHeight(height);
  }
# endif
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetMultiCameraExposureTime(
    const std_msgs::Float64::ConstPtr &_msg)
{
  this->multiCameraExposureTime = static_cast<double>(_msg->data);
  gzwarn << "setting camera exposure time in sim not implemented\n";
}

/////////////////////////////////////////////////
void SRCMultiSenseSL::SetMultiCameraGain(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraGain = static_cast<double>(_msg->data);
  gzwarn << "setting camera gain in sim not implemented\n";
}
