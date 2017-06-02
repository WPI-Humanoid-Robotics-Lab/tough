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

#include "srcsim/HarnessManager.hh"
#include "srcsim/Task1.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task1::Task1(const sdf::ElementPtr &_sdf) : Task(_sdf)
{
  gzmsg << "Creating Task [1] ... ";

  // Get SDF for each checkpoint
  sdf::ElementPtr cp1Elem, cp2Elem, cp4Elem;
  if (_sdf)
  {
    if (_sdf->HasElement("checkpoint1"))
      cp1Elem = _sdf->GetElement("checkpoint1");

    if (_sdf->HasElement("checkpoint2"))
      cp2Elem = _sdf->GetElement("checkpoint2");

    if (_sdf->HasElement("checkpoint4"))
      cp4Elem = _sdf->GetElement("checkpoint4");
  }

  // Checkpoint 1: Walk to satellite dish
  std::unique_ptr<Task1CP1> cp1(new Task1CP1(cp1Elem));
  this->checkpoints.push_back(std::move(cp1));

  // Checkpoints 2: Correct pitch or yaw
  std::unique_ptr<Task1CP2> cp2(new Task1CP2(cp2Elem));
  this->checkpoints.push_back(std::move(cp2));

  // Checkpoints 3: Correct pitch and yaw
  // it is not supported to skip the previous checkpoint and start with this
  // therefore simply reusing checkpoint 2
  std::unique_ptr<Task1CP3> cp3(new Task1CP3(cp2Elem));
  this->checkpoints.push_back(std::move(cp3));

  // Checkpoints 3: Walk to final box
  std::unique_ptr<Task1CP4> cp4(new Task1CP4(cp4Elem));
  this->checkpoints.push_back(std::move(cp4));

  gzmsg << "Task [1] created" << std::endl;
}

/////////////////////////////////////////////////
size_t Task1::Number() const
{
  return 1u;
}

/////////////////////////////////////////////////
bool Task1CP1::Check()
{
  return this->CheckBox("/task1/checkpoint1");
}

/////////////////////////////////////////////////
void Task1CP1::Restart(const common::Time &_penalty)
{
  // This is the 1st CP of the task: reharness back at start box
  HarnessManager::Instance()->NewGoal(
      ignition::math::Pose3d(0, 0, 1.257, 0, 0, 0));

  Checkpoint::Restart(_penalty);
}

/////////////////////////////////////////////////
void Task1CP2::OnSatelliteRosMsg(const srcsim::Satellite &_msg)
{
  this->oneAxisDone = _msg.yaw_completed || _msg.pitch_completed;
}

/////////////////////////////////////////////////
bool Task1CP2::Check()
{
  // First time
  if (!this->satelliteRosSub && !this->oneAxisDone)
  {
    this->Start();

    // Subscribe to satellite msgs
    this->rosNode.reset(new ros::NodeHandle());
    this->satelliteRosSub = this->rosNode->subscribe(
        "/task1/checkpoint2/satellite", 10, &Task1CP2::OnSatelliteRosMsg, this);

    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Enable satellite plugin
    auto togglePub = this->gzNode->Advertise<msgs::Int>(
        "/task1/checkpoint2/enable");

    msgs::Int msg;
    msg.set_data(1);
    togglePub->Publish(msg);
  }

  return this->oneAxisDone;
}

/////////////////////////////////////////////////
void Task1CP3::OnSatelliteRosMsg(const srcsim::Satellite &_msg)
{
  this->satelliteDone = _msg.yaw_completed && _msg.pitch_completed;
}

/////////////////////////////////////////////////
bool Task1CP3::Check()
{
  // First time
  if (!this->satelliteRosSub && !this->satelliteDone)
  {
    this->Start();

    // Subscribe to satellite msgs
    this->rosNode.reset(new ros::NodeHandle());
    this->satelliteRosSub = this->rosNode->subscribe(
        "/task1/checkpoint2/satellite", 10, &Task1CP3::OnSatelliteRosMsg, this);
  }

  if (this->satelliteDone && this->rosNode)
  {
    // Gazebo node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Disable satellite plugin
    auto togglePub = this->gzNode->Advertise<msgs::Int>(
        "/task1/checkpoint2/enable");

    msgs::Int msg;
    msg.set_data(0);
    togglePub->Publish(msg);

    this->rosNode.reset();
  }

  return this->satelliteDone;
}

/////////////////////////////////////////////////
bool Task1CP4::Check()
{
  return this->CheckBox("/task1/checkpoint4");
}

