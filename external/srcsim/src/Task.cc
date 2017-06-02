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

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <srcsim/Task.h>
#include <std_msgs/Time.h>

#include "srcsim/Task.hh"

using namespace gazebo;

common::Time Task::previousPenalty = common::Time::Zero;

/////////////////////////////////////////////////
Task::Task(const sdf::ElementPtr &_sdf)
{
  // Get timeout
  if (_sdf && _sdf->HasElement("timeout"))
  {
    this->timeout = _sdf->Get<double>("timeout");
  }
  else
  {
    gzwarn << "Timeout not specified, using default value ["
           << this->timeout << "]" << std::endl;
  }

  // ROS transport
  this->rosNode.reset(new ros::NodeHandle());

  this->taskRosPub = this->rosNode->advertise<srcsim::Task>(
      "/srcsim/finals/task", 1000);
}

/////////////////////////////////////////////////
void Task::Start(const common::Time &_time, const size_t _checkpoint)
{
  std::lock_guard<std::mutex> lock(this->updateMutex);
  // Double-check that we're not going back to a previous checkpoint
  if (_checkpoint < this->current)
  {
    gzerr << "Trying to start task [" << unsigned(this->Number()) <<
        "] checkpoint [" << unsigned(_checkpoint) <<
        "], and current checkpoint is [" << unsigned(this->current) << "]. " <<
        "It's not possible to go back to a previous checkpoint." << std::endl;
    return;
  }

  // Restarting current checkpoint
  if (_checkpoint == this->current)
  {
    // Trigger skip on previous cp to rearrange world
    if (this->current > 1)
      this->checkpoints[this->current - 2]->Skip();

    // Apply time penalty to current checkpoint
    this->ApplyPenaltyTime();

    return;
  }

  // First checkpoint officially starts when entering start box
  if (_checkpoint == 1 && !this->startBox)
  {
    // Already waiting
    if (this->gzNode)
      return;

    // Initialize node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Setup contains subscriber
    this->boxSub = this->gzNode->Subscribe(
        "/task" + std::to_string(this->Number()) + "/start/box/contains",
        &Task::OnStartBox, this);

    // Setup toggle publisher
    this->togglePub = this->gzNode->Advertise<msgs::Int>(
        "/task" + std::to_string(this->Number()) + "/start/box/toggle");

    // Toggle box plugin on
    msgs::Int msg;
    msg.set_data(1);
    this->togglePub->Publish(msg);

    gzmsg << "Task [" << this->Number()
          << "] - Started: time will start counting as you leave the box."
          << std::endl;

    return;
  }

  // Starting task
  if (this->current == 0)
    this->startTime = _time;

  // Check if there are checkpoints being skipped
  this->SkipUpTo(_checkpoint - 1, true);

  // Checkpoint
  this->current = _checkpoint;

  this->forceCpCompletionRosSub = this->rosNode->subscribe(
      "/srcsim/finals/force_checkpoint_completion", 1,
      &Task::OnForceCpCompletionRosMsg, this);

  gzmsg << "Task [" << this->Number() << "] - Checkpoint [" << this->current
        << "] - Started (" << _time << ")" << std::endl;
}

/////////////////////////////////////////////////
void Task::Update(const common::Time &_time)
{
  std::lock_guard<std::mutex> lock(this->updateMutex);
  // Task has finished
  if (this->current > this->checkpoints.size())
    return;

  // While in start box (before time starts counting)
  if (this->current < 1)
  {
    // Publish ROS task message with zero CP and start time
    srcsim::Task msg;
    msg.task = this->Number();
    msg.current_checkpoint = this->current;
    msg.start_time.fromSec(0);
    msg.elapsed_time.fromSec(0);

    this->taskRosPub.publish(msg);
    return;
  }

  // Terminate start transport
  if (this->gzNode)
  {
    this->boxSub.reset();
    this->togglePub.reset();
    this->gzNode->Fini();
    this->gzNode.reset();
  }

  // Timeout
  auto elapsed = _time - this->startTime + this->totalPenalty;
  this->timedOut = !this->finished && elapsed > this->timeout;

  if (this->timedOut)
  {
    elapsed = this->timeout;
    // No penalty when skipping due to timeout
    this->Skip(false);
  }
  else
  {
    // Check if current checkpoint is complete
    bool completed = this->checkpoints[this->current - 1]->Check();
    if (this->forceCpCompletion) {
      gzmsg << "Force checkpoint completion based on ROS message." << std::endl;
      completed = true;
      // Reset flag until next ROS message is received
      this->forceCpCompletion = false;
    }
    if (completed)
    {
      gzmsg << "Task [" << this->Number() << "] - Checkpoint [" << this->current
            << "] - Completed (" << _time << ")" << std::endl;

      // Sanity check
      if (this->cpDuration.size() >= this->checkpoints.size())
        gzerr << "Too many checkpoint completions!" << std::endl;

      auto duration = _time - this->checkpoints[this->current - 1]->StartTime();

      // It is possible that 2 checkpoints are completed at the same time and
      // we end up with start time == end time. Add a millisecond so this doesn't
      // get flagged as skipped.
      if (duration == common::Time::Zero)
        duration = common::Time(0.001);

      this->cpDuration.push_back(duration);

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

  auto min = std::min(this->current, this->CheckpointCount());

  for (size_t i = 0; i < min; ++i)
  {
    // Publish duration for all finished cps
    if (i < this->cpDuration.size())
    {
      ros::Duration t(this->cpDuration[i].Double());
      msg.checkpoint_durations.push_back(t);
    }

    // Publish penalty for all cps including the current one
    ros::Duration p(this->checkpoints[i]->PenaltyTime().Double());
    msg.checkpoint_penalties.push_back(p);
  }

  this->taskRosPub.publish(msg);
}

/////////////////////////////////////////////////
void Task::Skip(const bool _penalty)
{
  // Skip all remaining up to last one
  this->SkipUpTo(this->CheckpointCount(), _penalty);
}

/////////////////////////////////////////////////
void Task::SkipUpTo(const size_t _lastSkipped, const bool _penalty)
{
  while (this->current <= _lastSkipped)
  {
    if (this->current > 0)
    {
      this->checkpoints[this->current - 1]->Skip();

      // Apply time penalty
      if (_penalty)
        this->ApplyPenaltyTime();

      gzmsg << "Task [" << this->Number() << "] - Checkpoint ["
            << this->current << "] - Skipped" << std::endl;
      this->cpDuration.push_back(common::Time::Zero);
    }

    this->current++;
  }
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

/////////////////////////////////////////////////
common::Time Task::GetCheckpointCompletion(const size_t _index) const
{
  if (_index < this->cpDuration.size())
  {
    return this->cpDuration[_index];
  }
  return common::Time::Zero;
}

/////////////////////////////////////////////////
common::Time Task::GetCheckpointPenalty(const size_t _index) const
{
  if (_index < this->CheckpointCount())
  {
    return this->checkpoints[_index]->PenaltyTime();
  }
  return common::Time::Zero;
}

//////////////////////////////////////////////////
void Task::OnStartBox(ConstIntPtr &_msg)
{
  this->startBox = _msg->data() == 0 ? false : true;

  auto world = physics::get_world();
  if (!world)
  {
    gzerr << "Failed to get world pointer, can't start task."
        << std::endl;
    return;
  }

  // Now the checkpoint starts
  this->Start(world->GetSimTime(), 1);
}

//////////////////////////////////////////////////
void Task::ApplyPenaltyTime()
{
  if (this->previousPenalty == common::Time::Zero)
  {
    this->previousPenalty = common::Time(30);
  }
  else
  {
    this->previousPenalty += common::Time(10);
  }

  // Increment the task's total penalty (to be used for checking timeout)
  this->totalPenalty += this->previousPenalty;

  // Increment the checkpoint's penalty (to be used by scoring)
  this->checkpoints[this->current - 1]->Restart(this->previousPenalty);

  gzmsg << "Applied penalty time of [" << int(this->previousPenalty.Double())
        << "] seconds" << std::endl;
}

/////////////////////////////////////////////////
void Task::OnForceCpCompletionRosMsg(const std_msgs::Empty::ConstPtr &)
{
  this->forceCpCompletion = true;
}
