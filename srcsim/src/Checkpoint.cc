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

#include <gazebo/common/Console.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>

#include "srcsim/Checkpoint.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Checkpoint::Checkpoint(const sdf::ElementPtr &_sdf)
{
  // Get robot pose
  if (_sdf && _sdf->HasElement("skip_robot_pose"))
  {
    this->robotSkipPose = _sdf->Get<ignition::math::Pose3d>("skip_robot_pose");
  }
}

/////////////////////////////////////////////////
void Checkpoint::Skip()
{
  if (this->robotSkipPose == ignition::math::Pose3d::Zero)
    return;

  // Teleport robot
  // TODO: Reset joints
  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world pointer, robot won't be teleported."
        << std::endl;
    return;
  }
  auto robot = world->GetModel("valkyrie");
  if (!robot)
  {
    gzerr << "Failed to get model pointer, robot won't be teleported."
        << std::endl;
    return;
  }
  robot->SetWorldPose(this->robotSkipPose);
}

/////////////////////////////////////////////////
bool BoxCheckpoint::CheckBox(const std::string &_namespace)
{
  // First time checking
  if (!this->gzNode && !this->boxDone)
  {
    // Initialize node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Setup contains subscriber
    this->boxSub = this->gzNode->Subscribe(_namespace + "/box/contains",
        &BoxCheckpoint::OnBox, this);

    // Setup toggle publisher
    auto toggleTopic = _namespace + "/box/toggle";
    this->togglePub = this->gzNode->Advertise<msgs::Int>(toggleTopic);

    // Toggle box plugin on
    msgs::Int msg;
    msg.set_data(1);
    this->togglePub->Publish(msg);
  }

  if (this->boxDone && this->togglePub)
  {
    // Toggle box plugin on
    msgs::Int msg;
    msg.set_data(0);
    this->togglePub->Publish(msg);

    // Clear transport
    this->gzNode->Fini();
    this->togglePub.reset();
    this->gzNode.reset();
  }

  return this->boxDone;
}

//////////////////////////////////////////////////
void BoxCheckpoint::OnBox(ConstIntPtr &_msg)
{
  this->boxDone = _msg->data() == 0 ? false : true;
}

/////////////////////////////////////////////////
bool TouchCheckpoint::CheckTouch(const std::string &_namespace)
{
  // First time checking
  if (!this->touchGzSub && !this->touchDone)
  {
    // Initialize node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Enable touch plugin
    this->enableGzPub = this->gzNode->Advertise<msgs::Int>(
        _namespace + "/enable");

    msgs::Int msg;
    msg.set_data(1);
    this->enableGzPub->Publish(msg);

    // Subscribe to touch msgs
    this->touchGzSub = this->gzNode->Subscribe(_namespace + "/touched",
        &TouchCheckpoint::OnTouchGzMsg, this);
  }

  if (this->touchDone && this->enableGzPub)
  {
    msgs::Int msg;
    msg.set_data(0);
    this->enableGzPub->Publish(msg);

    this->touchGzSub.reset();
  }

  return this->touchDone;
}

//////////////////////////////////////////////////
void TouchCheckpoint::OnTouchGzMsg(ConstIntPtr &/*_msg*/)
{
  this->touchDone = true;
}
