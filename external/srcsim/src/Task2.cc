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

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>

#include "srcsim/Task2.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task2::Task2(const sdf::ElementPtr &_sdf) : Task(_sdf)
{
  gzmsg << "Creating Task [2] ... ";

  // Get SDF for each checkpoint
  sdf::ElementPtr cp1Elem, cp2Elem, cp3Elem, cp4Elem, cp5Elem, cp6Elem;
  if (_sdf)
  {
    if (_sdf->HasElement("checkpoint1"))
      cp1Elem = _sdf->GetElement("checkpoint1");

    if (_sdf->HasElement("checkpoint2"))
      cp2Elem = _sdf->GetElement("checkpoint2");

    if (_sdf->HasElement("checkpoint3"))
      cp3Elem = _sdf->GetElement("checkpoint3");

    if (_sdf->HasElement("checkpoint4"))
      cp4Elem = _sdf->GetElement("checkpoint4");

    if (_sdf->HasElement("checkpoint5"))
      cp5Elem = _sdf->GetElement("checkpoint5");

    if (_sdf->HasElement("checkpoint6"))
      cp6Elem = _sdf->GetElement("checkpoint6");
  }

  // Checkpoint 1: Lift solar panel
  std::unique_ptr<Task2CP1> cp1(new Task2CP1(cp1Elem));
  this->checkpoints.push_back(std::move(cp1));

  // Checkpoint 2: Place solar panel near cable
  std::unique_ptr<Task2CP2> cp2(new Task2CP2(cp2Elem));
  this->checkpoints.push_back(std::move(cp2));

  // Checkpoint 3: Deploy solar panel
  std::unique_ptr<Task2CP3> cp3(new Task2CP3(cp3Elem));
  this->checkpoints.push_back(std::move(cp3));

  // Checkpoint 4: Lift cable
  std::unique_ptr<Task2CP4> cp4(new Task2CP4(cp4Elem));
  this->checkpoints.push_back(std::move(cp4));

  // Checkpoint 5: Plug cable
  std::unique_ptr<Task2CP5> cp5(new Task2CP5(cp5Elem));
  this->checkpoints.push_back(std::move(cp5));

  // Checkpoint 6: Walk to final box
  std::unique_ptr<Task2CP6> cp6(new Task2CP6(cp6Elem));
  this->checkpoints.push_back(std::move(cp6));

  gzmsg << "Task [2] created" << std::endl;
}

/////////////////////////////////////////////////
size_t Task2::Number() const
{
  return 2u;
}

/////////////////////////////////////////////////
bool Task2CP1::Check()
{
  return this->CheckTouch("/task2/checkpoint1");
}

/////////////////////////////////////////////////
Task2CP2::Task2CP2(const sdf::ElementPtr &_sdf) : BoxCheckpoint(_sdf)
{
  if (_sdf && _sdf->HasElement("panel_pose"))
    this->panelSkipPose = _sdf->Get<ignition::math::Pose3d>("panel_pose");
  else
    gzwarn << "Missing <panel_pose>, using default value" << std::endl;
}

/////////////////////////////////////////////////
bool Task2CP2::Check()
{
  return this->CheckBox("/task2/checkpoint2");
}

/////////////////////////////////////////////////
void Task2CP2::Skip()
{
  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world" << std::endl;
    return;
  }

  auto panel = world->GetModel("solar_panel");
  if (!panel)
  {
    gzerr << "Failed to get [solar_panel] model" << std::endl;
    return;
  }

  panel->SetWorldPose(this->panelSkipPose);

  Checkpoint::Skip();
}

/////////////////////////////////////////////////
void Task2CP3::OnSolarPanelGzMsg(ConstIntPtr &/*_msg*/)
{
  this->panelDone = true;
}

/////////////////////////////////////////////////
bool Task2CP3::Check()
{
  // First time
  if (!this->panelGzSub && !this->panelDone)
  {
    this->Start();

    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Enable solar panel plugin
    this->enableGzPub = this->gzNode->Advertise<msgs::Int>(
        "/task2/checkpoint3/enable");

    msgs::Int msg;
    msg.set_data(1);
    this->enableGzPub->Publish(msg);

    // Subscribe to solar panel msgs
    this->panelGzSub = this->gzNode->Subscribe("/task2/checkpoint3/opened",
        &Task2CP3::OnSolarPanelGzMsg, this);
  }

  if (this->panelDone)
  {
    msgs::Int msg;
    msg.set_data(0);
    this->enableGzPub->Publish(msg);

    this->panelGzSub.reset();
  }

  return this->panelDone;
}

/////////////////////////////////////////////////
void Task2CP3::Skip()
{
  if (this->panelDone)
  {
    gzwarn << "Trying to skip Task 2 Checkpoint 3, "
           << "but this checkpoint is already done!" << std::endl;
    return;
  }

  if (!this->enableGzPub)
  {
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Enable solar panel plugin
    this->enableGzPub = this->gzNode->Advertise<msgs::Int>(
        "/task2/checkpoint3/enable");
  }

  msgs::Int msg;
  msg.set_data(2);
  this->enableGzPub->Publish(msg);

  this->enableGzPub.reset();

  Checkpoint::Skip();
}

/////////////////////////////////////////////////
bool Task2CP4::Check()
{
  return this->CheckTouch("/task2/checkpoint4");
}

/////////////////////////////////////////////////
bool Task2CP5::Check()
{
  // Previously finished
  if (this->done)
    return true;

  // First time
  if (!this->sensor)
  {
    this->Start();

    this->world = physics::get_world();

    if (this->world)
    {
      auto s = sensors::SensorManager::Instance()->GetSensor(this->sensorName);
      if (!s)
      {
        gzerr << "Failed to get sensor [" << this->sensorName << "]" << std::endl;
        return false;
      }

      this->sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(s);
      if (!this->sensor)
      {
        gzerr << "Failed to cast sensor [" << this->sensorName << "]" << std::endl;
        return false;
      }

      this->sensor->SetActive(true);
    }
  }

  // If couldn't get sensor
  if (!this->sensor)
  {
    gzerr << "Couldn't get world or sensor pointer. "
          << "Not checking Task 2 Checkpoint 5." << std::endl;
    return false;
  }

  // Get all contacts
  auto contacts = this->sensor->Contacts();

  bool touching = false;

  for (auto const &contact : contacts.contact())
  {
    // Check for the plug
    bool col1Plug = contact.collision1().find(this->plug) != std::string::npos;
    bool col2Plug = contact.collision2().find(this->plug) != std::string::npos;

    // Check for the outlet
    bool col1Outlet = contact.collision1().find(this->outlet) !=
        std::string::npos;
    bool col2Outlet = contact.collision2().find(this->outlet) !=
        std::string::npos;

    // We need at least one plug-outlet or outlet-plug contact
    if ((col1Plug && col2Outlet) || (col1Outlet && col2Plug))
    {
      touching = true;
      break;
    }
  }

  auto simTime = world->GetSimTime();

  // Not touching
  if (!touching)
  {
    // Stopped touching
    if (this->touchStart != common::Time::Zero)
    {
      gzmsg << "Plug stopped touching outlet at " << simTime
            << " seconds" << std::endl;
      this->touchStart = common::Time::Zero;
    }
    return false;
  }

  // Just started touching
  if (this->touchStart == common::Time::Zero)
  {
    this->touchStart = simTime;
    gzmsg << "Plug started touching outlet at " << this->touchStart
          << " seconds" << std::endl;
  }

  // Check if it has been plugged for long enough
  auto itsPlugTime = simTime - this->touchStart > this->targetTime;

  if (!itsPlugTime)
    return false;

  // We set done to true because the checkpoint has technically been completed.
  // If we fail to create a joint below, this is not the competitor's fault.
  this->done = true;

  // Create fixed joint

  // Get models
  auto cableModel = world->GetModel(this->cable);
  auto plugLink = boost::dynamic_pointer_cast<physics::Link>(
      world->GetEntity(this->plug));

  auto outletLink = boost::dynamic_pointer_cast<physics::Link>(
      world->GetEntity(this->outletParent));

  if (!cableModel || !plugLink || !outletLink)
  {
    gzerr << "Failed to get a model, cable won't be plugged."
          << std::endl;
    return this->done;
  }

  auto fixedJoint = world->GetPhysicsEngine()->CreateJoint("fixed", cableModel);
  fixedJoint->SetName(this->cable + "_plug_fixed_joint__");

  fixedJoint->Load(plugLink, outletLink, ignition::math::Pose3d::Zero);
  fixedJoint->Init();

  gzmsg << "The cable plug has been fixed to the outlet" << std::endl;

  return this->done;
}

/////////////////////////////////////////////////
bool Task2CP6::Check()
{
  return this->CheckBox("/task2/checkpoint6");
}

