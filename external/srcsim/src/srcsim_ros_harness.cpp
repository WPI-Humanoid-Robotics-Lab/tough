/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include "srcsim/srcsim_ros_harness.h"

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SRCSimRosHarness)

/////////////////////////////////////////////////
SRCSimRosHarness::SRCSimRosHarness()
{
}

/////////////////////////////////////////////////
SRCSimRosHarness::~SRCSimRosHarness()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();

  this->rosnode_->shutdown();
  delete this->rosnode_;
}

/////////////////////////////////////////////////
// Load the controller
void SRCSimRosHarness::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Load the plugin
  SRCHarnessPlugin::Load(_parent, _sdf);

  this->robotNamespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace_ = _sdf->Get<std::string>("robotNamespace") + "/";

  // Init ROS
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + _parent->GetName() + "/harness/velocity", 1,
    boost::bind(&SRCSimRosHarness::OnVelocity, this, _1),
    ros::VoidPtr(), &this->queue_);
  this->velocitySub_ = this->rosnode_->subscribe(so);

  so = ros::SubscribeOptions::create<std_msgs::Bool>(
    "/" + _parent->GetName() + "/harness/detach", 1,
    boost::bind(&SRCSimRosHarness::OnDetach, this, _1),
    ros::VoidPtr(), &this->queue_);
  this->detachSub_ = this->rosnode_->subscribe(so);

  so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
    "/" + _parent->GetName() + "/harness/attach", 1,
    boost::bind(&SRCSimRosHarness::OnAttach, this, _1),
    ros::VoidPtr(), &this->queue_);
  this->attachSub_ = this->rosnode_->subscribe(so);

  // Custom Callback Queue
  this->callbackQueueThread_ =
    boost::thread(boost::bind(&SRCSimRosHarness::QueueThread, this));
}

/////////////////////////////////////////////////
void SRCSimRosHarness::OnVelocity(const std_msgs::Float32::ConstPtr &msg)
{
  // Set the target winch velocity
  this->SetWinchVelocity(msg->data);
}

/////////////////////////////////////////////////
void SRCSimRosHarness::OnDetach(const std_msgs::Bool::ConstPtr &msg)
{
  // Detach if true
  if (msg->data)
    this->Detach();
}

/////////////////////////////////////////////////
void SRCSimRosHarness::OnAttach(const geometry_msgs::Pose::ConstPtr &msg)
{
  ignition::math::Vector3d pos(
      msg->position.x,
      msg->position.y,
      msg->position.z);
  ignition::math::Quaterniond rot(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);
  this->Attach(ignition::math::Pose3d(pos, rot));
}

/////////////////////////////////////////////////
void SRCSimRosHarness::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
