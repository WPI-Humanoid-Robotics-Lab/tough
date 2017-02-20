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

#include "srcsim/Task1.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task1::Task1(const common::Time &_timeout,
    const std::vector<ignition::math::Pose3d> _poses)
    : Task(_timeout)
{
  // Checkpoint 1: Walk to satellite dish
  std::unique_ptr<Task1CP1> cp1(new Task1CP1(ignition::math::Pose3d::Zero));
  this->checkpoints.push_back(std::move(cp1));

  // Checkpoints 2: Correct pitch / yaw
  std::unique_ptr<Task1CP2> cp2(new Task1CP2(_poses[0]));
  this->checkpoints.push_back(std::move(cp2));

  // Checkpoints 3: Walk to final box
  std::unique_ptr<Task1CP3> cp3(new Task1CP3(_poses[1]));
  this->checkpoints.push_back(std::move(cp3));

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
void Task1CP2::OnSatelliteRosMsg(const srcsim::Satellite &_msg)
{
  this->satelliteDone = _msg.yaw_completed && _msg.pitch_completed;
}

/////////////////////////////////////////////////
bool Task1CP2::Check()
{
  // First time
  if (!this->satelliteRosSub && !this->satelliteDone)
  {
    // Subscribe to satellite msgs
    this->rosNode.reset(new ros::NodeHandle());
    this->satelliteRosSub = this->rosNode->subscribe(
        "/task1/checkpoint2/satellite", 10, &Task1CP2::OnSatelliteRosMsg, this);
  }

  if (this->satelliteDone && this->rosNode)
  {
    // Gazebo node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    auto togglePub = this->gzNode->Advertise<msgs::Int>(
        "/task1/checkpoint2/toggle"); 

    msgs::Int msg;
    msg.set_data(0);
    togglePub->Publish(msg);

    this->rosNode.reset();
  }

  return this->satelliteDone; 
}

/////////////////////////////////////////////////
bool Task1CP3::Check()
{
  return this->CheckBox("/task1/checkpoint3");
}

