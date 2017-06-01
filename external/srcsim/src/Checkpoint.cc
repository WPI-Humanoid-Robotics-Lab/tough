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

  // Get models to delete when checkpoint begins
  if (_sdf && _sdf->HasElement("delete_model"))
  {
    auto elem = _sdf->GetElement("delete_model");
    while (elem)
    {
      this->deleteModels.push_back(elem->Get<std::string>());

      elem = elem->GetNextElement("delete_model");
    }
  }

  // Get models to insert when checkpoint begins
  if (_sdf && _sdf->HasElement("insert_model"))
  {
    auto elem = _sdf->GetElement("insert_model");
    while (elem)
    {
      auto inner = elem->GetFirstElement();
      this->insertModels.push_back(inner->ToString(""));

      elem = elem->GetNextElement("insert_model");
    }
  }
}

/////////////////////////////////////////////////
void Checkpoint::Start()
{
  if (this->insertModels.empty() && this->deleteModels.empty())
    return;

  auto tmpGzNode = transport::NodePtr(new transport::Node());
  tmpGzNode->Init();

  // Delete models
  for (auto modelName : this->deleteModels)
  {
    transport::requestNoReply(tmpGzNode, "entity_delete", modelName);
    gzmsg << "Requested to delete [" << modelName << "]" << std::endl;
  }
  this->deleteModels.clear();

  // Insert models
  if (this->insertModels.empty())
    return;

  auto factoryPub = tmpGzNode->Advertise<msgs::Factory>("~/factory");

  for (const auto &modelStr : this->insertModels)
  {
    auto sdfStr =
        "<sdf version='" + std::string(SDF_VERSION) + "'>"
        + modelStr
        + "</sdf>";

    // Get model name for debug message
    std::string name;

    sdf::SDFPtr modelSDF(new sdf::SDF);
    modelSDF->SetFromString(sdfStr);

    if (!modelSDF || !modelSDF->Root())
    {
      gzerr << "Failed to parse model SDF: " << std::endl << modelStr
            << std::endl;
      continue;
    }

    auto root = modelSDF->Root();

    if (root->HasElement("include"))
    {
      name = root->GetElement("include")->Get<std::string>("uri");
    }
    else if (root->HasElement("model"))
    {
      name = root->GetElement("model")->Get<std::string>("name");
    }
    else
    {
      gzerr << "Failed to parse model SDF: " << std::endl << modelStr
            << std::endl;
      continue;
    }

    msgs::Factory msg;
    msg.set_sdf(sdfStr);
    factoryPub->Publish(msg);
    gzmsg << "Requested to insert model [" << name << "]" << std::endl;
  }
  this->insertModels.clear();
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
