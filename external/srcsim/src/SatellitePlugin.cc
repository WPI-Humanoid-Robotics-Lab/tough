/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

#include <srcsim/Satellite.h>
#include "srcsim/SatellitePlugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
SatelliteDishPlugin::SatelliteDishPlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
void SatelliteDishPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Read in parameters from SDF
  if (!_sdf->HasElement("yaw_ratio") ||
      !_sdf->HasElement("pitch_ratio") ||
      !_sdf->HasElement("frequency") ||
      !_sdf->HasElement("yaw_pid") ||
      !_sdf->HasElement("pitch_pid") ||
      !_sdf->HasElement("yaw_target") ||
      !_sdf->HasElement("pitch_target") ||
      !_sdf->HasElement("topic") ||
      !_sdf->HasElement("tolerance") ||
      !_sdf->HasElement("time"))
  {
    gzerr << "Missing required parameters, plugin will not load."
          << std::endl;
    return;
  }

  this->yawRatio = _sdf->Get<double>("yaw_ratio");
  this->pitchRatio = _sdf->Get<double>("pitch_ratio");
  this->yawTarget = _sdf->Get<double>("yaw_target");
  this->pitchTarget = _sdf->Get<double>("pitch_target");
  this->frequency = _sdf->Get<double>("frequency");
  this->tolerance = _sdf->Get<double>("tolerance");
  this->targetTime = common::Time(_sdf->Get<double>("time"));

  // Get joints
  this->yawJoint = _model->GetJoint("dish_yaw");
  this->pitchJoint = _model->GetJoint("dish_pitch");
  this->yawWheel = _model->GetJoint("wheel_yaw");
  this->pitchWheel = _model->GetJoint("wheel_pitch");

  if (!this->yawWheel || !this->pitchWheel)
  {
    gzerr << "Missing joint in model, plugin will not load."
          << std::endl;
    return;
  }

  // Configure controller
  this->controller = _model->GetJointController();

  auto vec = _sdf->Get<ignition::math::Vector3d>("yaw_pid");
  common::PID pidYaw(vec.X(), vec.Y(), vec.Z());
  this->controller->SetPositionPID(this->yawJoint->GetScopedName(), pidYaw);

  vec = _sdf->Get<ignition::math::Vector3d>("pitch_pid");
  common::PID pidPitch(vec.X(), vec.Y(), vec.Z());
  this->controller->SetPositionPID(this->pitchJoint->GetScopedName(),
      pidPitch);

  // Create joint to the world
  physics::JointPtr joint;
  joint = _model->GetWorld()->GetPhysicsEngine()->CreateJoint("fixed",
      _model);
  joint->Load(nullptr, _model->GetLink("base"), ignition::math::Pose3d());
  joint->Attach(nullptr, _model->GetLink("base"));

  // Update loop
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&SatelliteDishPlugin::OnUpdate, this,
      std::placeholders::_1)));

  // ROS transport
  this->rosNode.reset(new ros::NodeHandle());

  auto topic = _sdf->Get<std::string>("topic");
  this->satelliteRosPub =
      this->rosNode->advertise<srcsim::Satellite>(topic, 1000);
}

/////////////////////////////////////////////////
void SatelliteDishPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  static auto lastUpdate = common::Time::Zero;

  // Limit update rate for performance
  if (_info.simTime - lastUpdate < 1/this->frequency)
    return;

  lastUpdate = _info.simTime;

  // Check if correct
  auto currentYaw = this->yawJoint->GetAngle(0).Radian();
  auto currentPitch = this->pitchJoint->GetAngle(0).Radian();

  auto yawCorrectNow =
      fabs(currentYaw - this->yawTarget) < IGN_DTOR(this->tolerance);

  bool pitchCorrectNow =
      fabs(currentPitch - this->pitchTarget) < IGN_DTOR(this->tolerance);

  bool yawCompleted = false;
  if (yawCorrectNow)
  {
    if (this->yawCorrectStart == common::Time::Zero)
      this->yawCorrectStart = lastUpdate;

    yawCompleted = lastUpdate - this->yawCorrectStart > this->targetTime;
  }
  else
  {
    this->yawCorrectStart = common::Time::Zero;
  }

  bool pitchCompleted = false;
  if (pitchCorrectNow)
  {
    if (this->pitchCorrectStart == common::Time::Zero)
      this->pitchCorrectStart = lastUpdate;

    pitchCompleted = lastUpdate - this->pitchCorrectStart > this->targetTime;
  }
  else
  {
    this->pitchCorrectStart = common::Time::Zero;
  }

  // Control joints
  this->controller->SetPositionTarget(this->yawJoint->GetScopedName(),
      this->yawWheel->GetAngle(0).Radian() * this->yawRatio);
  this->controller->SetPositionTarget(this->pitchJoint->GetScopedName(),
      this->pitchWheel->GetAngle(0).Radian() * this->pitchRatio);

  // Publish
  srcsim::Satellite msg;
  msg.target_pitch = this->pitchTarget;
  msg.target_yaw = this->yawTarget;
  msg.current_pitch = currentPitch;
  msg.current_yaw = currentYaw;
  msg.pitch_correct_now = pitchCorrectNow;
  msg.yaw_correct_now = yawCorrectNow;
  msg.pitch_completed = pitchCompleted;
  msg.yaw_completed = yawCompleted;

  this->satelliteRosPub.publish(msg);
}
