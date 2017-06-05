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
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>

#include "srcsim/Checkpoint.hh"
#include "srcsim/HarnessManager.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Checkpoint::Checkpoint(const sdf::ElementPtr &_sdf)
{
  // Get robot skip pose
  if (_sdf && _sdf->HasElement("skip_robot_pose"))
  {
    this->robotSkipPose = _sdf->Get<ignition::math::Pose3d>("skip_robot_pose");
  }

  // Get robot start pose
  if (_sdf && _sdf->HasElement("start_robot_pose"))
  {
    this->robotStartPose = _sdf->Get<ignition::math::Pose3d>("start_robot_pose");
  }

  // Get models to delete when checkpoint begins
  if (_sdf && _sdf->HasElement("delete_entity"))
  {
    auto elem = _sdf->GetElement("delete_entity");
    while (elem)
    {
      this->deleteEntities.push_back(elem->Get<std::string>());

      elem = elem->GetNextElement("delete_entity");
    }
  }

  // Get models to insert when checkpoint begins
  if (_sdf && _sdf->HasElement("insert_entity"))
  {
    auto elem = _sdf->GetElement("insert_entity");
    while (elem)
    {
      auto inner = elem->GetFirstElement();
      this->insertEntities.push_back(inner->ToString(""));

      elem = elem->GetNextElement("insert_entity");
    }
  }
}

/////////////////////////////////////////////////
common::Time Checkpoint::StartTime() const
{
  return this->startTime;
}

/////////////////////////////////////////////////
void Checkpoint::Start()
{
  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world pointer, can't start checkpoint."
        << std::endl;
    return;
  }

  // Now the checkpoint starts
  this->startTime = world->GetSimTime();

  // Insert / delete entities
  if (this->insertEntities.empty() && this->deleteEntities.empty())
    return;

  auto tmpGzNode = transport::NodePtr(new transport::Node());
  tmpGzNode->Init();

  // Delete entities
  for (auto entityName : this->deleteEntities)
  {
    transport::requestNoReply(tmpGzNode, "entity_delete", entityName);
    gzmsg << "Requested to delete [" << entityName << "]" << std::endl;
  }
  this->deleteEntities.clear();

  // Insert entities
  if (this->insertEntities.empty())
    return;

  auto factoryPub = tmpGzNode->Advertise<msgs::Factory>("~/factory");
  auto lightFactoryPub = tmpGzNode->Advertise<msgs::Light>("~/factory/light");

  for (const auto &entityStr : this->insertEntities)
  {
    auto sdfStr =
        "<sdf version='" + std::string(SDF_VERSION) + "'>"
        + entityStr
        + "</sdf>";

    // Get entity name for debug message
    std::string name;

    sdf::SDFPtr entitySDF(new sdf::SDF);
    entitySDF->SetFromString(sdfStr);

    if (!entitySDF || !entitySDF->Root())
    {
      gzerr << "Failed to parse entity SDF: " << std::endl << entityStr
            << std::endl;
      continue;
    }

    auto root = entitySDF->Root();
    sdf::ElementPtr light;

    if (root->HasElement("include"))
    {
      name = root->GetElement("include")->Get<std::string>("uri");
    }
    else if (root->HasElement("model"))
    {
      name = root->GetElement("model")->Get<std::string>("name");
    }
    else if (root->HasElement("light"))
    {
      light = root->GetElement("light");
      name = light->Get<std::string>("name");
    }
    else
    {
      gzerr << "Invalid entity SDF: " << std::endl << entityStr
            << std::endl;
      continue;
    }

    if (!light)
    {
      msgs::Factory msg;
      msg.set_sdf(sdfStr);
      factoryPub->Publish(msg);
    }
    else
    {
      msgs::Light msg = msgs::LightFromSDF(light);
      lightFactoryPub->Publish(msg);
    }
    gzmsg << "Requested to insert entity [" << name << "]" << std::endl;
  }
  this->insertEntities.clear();
  factoryPub.reset();
  tmpGzNode->Fini();
  tmpGzNode.reset();
}

/////////////////////////////////////////////////
void Checkpoint::Skip()
{
  // Perform start so models are inserted/deleted
  this->Start();

  if (this->robotSkipPose == ignition::math::Pose3d::Zero)
    return;

  // Teleport robot
  HarnessManager::Instance()->NewGoal(this->robotSkipPose);
}

/////////////////////////////////////////////////
void Checkpoint::Restart(const common::Time &_penalty)
{
  this->totalPenalty += _penalty;
}

/////////////////////////////////////////////////
common::Time Checkpoint::PenaltyTime() const
{
  return this->totalPenalty;
}

/////////////////////////////////////////////////
bool BoxCheckpoint::CheckBox(const std::string &_namespace)
{
  // First time checking
  if (!this->gzNode && !this->boxDone)
  {
    this->Start();

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
    this->Start();

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
