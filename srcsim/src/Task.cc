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
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <srcsim/Task.h>
#include <std_msgs/Time.h>

#include "srcsim/Task.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task::Task(const common::Time &_timeout) : timeout(_timeout)
{
  // ROS transport
  this->rosNode.reset(new ros::NodeHandle());

  this->taskRosPub = this->rosNode->advertise<srcsim::Task>(
      "/srcsim/finals/task", 1000);
}

/////////////////////////////////////////////////
void Task::Start(const common::Time &_time, const size_t _checkpoint,
    bool _skipped)
{
  // Double-check that we're not going back to a previous checkpoint
  if (_checkpoint <= this->current)
  {
    gzerr << "Trying to start task [" << unsigned(this->Number()) <<
        "] checkpoint [" << unsigned(_checkpoint) <<
        "], and current checkpoint is [" << unsigned(this->current) << "]. " <<
        "It's not possible to go back to a previous checkpoint." << std::endl;
    return;
  }

  // Starting task
  if (this->current == 0)
    this->startTime = _time;

  // Check if there are checkpoints being skipped
  while (this->current < _checkpoint)
  {
    if (this->current > 0)
    {
      gzmsg << "Task [" << this->Number() << "] - Checkpoint ["
            << this->current << "] - Skipped (" << _time << ")" << std::endl;
      this->checkpointsCompletion.push_back(common::Time::Zero);

      _skipped = true;
    }

    this->current++;
  }

  // Checkpoint
  this->current = _checkpoint;

  // Teleport robot if tasks / checkpoints are being skipped
  // TODO: Reset joints
  if (_skipped)
  {
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
    robot->SetWorldPose(this->checkpoints[this->current - 1]->startPose);
  }

  gzmsg << "Task [" << this->Number() << "] - Checkpoint [" << this->current
        << "] - Started (" << _time << ")" << std::endl;
}

/////////////////////////////////////////////////
void Task::Update(const common::Time &_time)
{
  // Timeout
  auto elapsed = _time - this->startTime;
  this->timedOut = !this->finished && elapsed > this->timeout;

  if (this->timedOut)
  {
    elapsed = this->timeout;
    this->current = 0;
  }

  // Checkpoints
  if (!this->timedOut && this->current != 0)
  {
    // Check if current checkpoint is complete
    if (this->checkpoints[this->current - 1]->Check())
    {
      gzmsg << "Task [" << this->Number() << "] - Checkpoint [" << this->current
            << "] - Completed (" << _time << ")" << std::endl;

      // Sanity check
      if (this->checkpointsCompletion.size() >= this->checkpoints.size())
        gzerr << "Too many checkpoint completions!" << std::endl;

      this->checkpointsCompletion.push_back(_time);

      this->current++;

      // Finish task if this was the last checkpoint
      this->finished = this->current > this->checkpoints.size();

      // Otherwise, start next checkpoint
      if (!this->finished)
      {
        gzmsg << "Task [" << this->Number() << "] - Checkpoint ["
              << this->current << "] - Started (" << _time << ")" << std::endl;
      }
    }
  }

  // Publish ROS task message
  srcsim::Task msg;
  msg.task = this->Number();
  msg.current_checkpoint = this->current;
  msg.finished = this->finished;
  msg.timed_out = this->timedOut;
  msg.start_time.fromSec(this->startTime.Double());
  msg.elapsed_time.fromSec(elapsed.Double());

  for (const auto &time : this->checkpointsCompletion)
  {
    ros::Time t(time.Double());
    msg.checkpoints_completion.push_back(t);
  }

  this->taskRosPub.publish(msg);
}

/////////////////////////////////////////////////
size_t Task::CheckpointCount() const
{
  return this->checkpoints.size();
}

/////////////////////////////////////////////////
size_t Task::CurrentCheckpointId() const
{
  return this->current;
}

