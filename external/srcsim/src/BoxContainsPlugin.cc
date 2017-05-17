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

#include <gazebo/common/UpdateInfo.hh>

#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/World.hh>

#include "srcsim/BoxContainsPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(BoxContainsPlugin)

/////////////////////////////////////////////////
BoxContainsPlugin::BoxContainsPlugin() : WorldPlugin()
{
}

/////////////////////////////////////////////////
void BoxContainsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Load SDF params
  if (!_sdf->HasElement("size"))
  {
    gzerr << "Missing required parameter <size>" << std::endl;
    return;
  }

  if (!_sdf->HasElement("pose"))
  {
    gzerr << "Missing required parameter <pose>" << std::endl;
    return;
  }

  if (!_sdf->HasElement("entity"))
  {
    gzerr << "Missing required parameter <entity>" << std::endl;
    return;
  }

  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }

  auto size = _sdf->Get<ignition::math::Vector3d>("size");
  auto pose = _sdf->Get<ignition::math::Pose3d>("pose");
  this->entityName = _sdf->Get<std::string>("entity");
  this->ns = _sdf->Get<std::string>("namespace");

  this->box = ignition::math::OrientedBoxd(size, pose);

  this->world = _world;

  // Start/stop "service"
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->toggleSub = this->gzNode->Subscribe("/" + this->ns + "/box/toggle",
      &BoxContainsPlugin::Toggle, this);

  auto enabled = _sdf->HasElement("enabled") && _sdf->Get<bool>("enabled");
  if (enabled)
  {
    ConstIntPtr msg;
    this->Toggle(msg);
  }
}

//////////////////////////////////////////////////
void BoxContainsPlugin::Toggle(ConstIntPtr &/*_msg*/)
{
  // Start
  if (!this->updateConnection)
  {
    this->entity = this->world->GetEntity(this->entityName);
    if (!this->entity)
    {
      gzerr << "Can't find entity[" << entity <<
          "] in world. Failed to enable Box Plugin." << std::endl;
      return;
    }

    // Start update
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BoxContainsPlugin::OnUpdate, this, std::placeholders::_1));

    this->containsPub = this->gzNode->Advertise<msgs::Int>(
        "/" + this->ns + "/box/contains");
    gzmsg << "Started box contains plugin [" << this->ns << "]" << std::endl;
  }
  // Stop
  else
  {
    this->updateConnection.reset();
    gzmsg << "Stopped box contains plugin [" << this->ns << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
void BoxContainsPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // For safety
  if (!this->entity)
  {
    gzerr << "Entity is null" << std::endl;
    return;
  }

  auto pos = this->entity->GetWorldPose().Ign().Pos();
  auto containsNow = this->box.Contains(pos) ? 1 : 0;

  if (containsNow != this->contains)
  {
    this->contains = containsNow;

    msgs::Int msg;
    msg.set_data(this->contains);

    this->containsPub->Publish(msg);
  }
}

