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

#include <ros/time.h>
#include <srcsim/Score.h>

#include "srcsim/FinalsPlugin.hh"
#include "srcsim/Task1.hh"
#include "srcsim/Task2.hh"
#include "srcsim/Task3.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(FinalsPlugin)

/////////////////////////////////////////////////
FinalsPlugin::FinalsPlugin() : WorldPlugin()
{
}

/////////////////////////////////////////////////
void FinalsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  gzmsg << "Initializing Finals plugin ... " << std::endl;

  if (!_sdf)
  {
    gzerr << "Something went wrong, missing SDF pointer." << std::endl;
    return;
  }

  this->world = _world;
  this->product = 1;

  // Task 1
  if (_sdf->HasElement("task1"))
  {
    std::unique_ptr<Task1> task1(new Task1(_sdf->GetElement("task1")));
    this->tasks.push_back(std::move(task1));
    product *= 2;
  }
  else
  {
    gzmsg << "Task [1] won't be generated." << std::endl;
    // Just for counting purposes
    this->tasks.push_back(nullptr);
  }

  // Task 2
  if (_sdf->HasElement("task2"))
  {
    std::unique_ptr<Task2> task2(new Task2(_sdf->GetElement("task2")));
    this->tasks.push_back(std::move(task2));
    product *= 3;
  }
  else
  {
    gzmsg << "Task [2] won't be generated." << std::endl;
    // Just for counting purposes
    this->tasks.push_back(nullptr);
  }

  // Task 3
  if (_sdf->HasElement("task3"))
  {
    std::unique_ptr<Task3> task3(new Task3(_sdf->GetElement("task3")));
    this->tasks.push_back(std::move(task3));
    product *= 5;
  }
  else
  {
    gzmsg << "Task [3] won't be generated." << std::endl;
    // Just for counting purposes
    this->tasks.push_back(nullptr);
  }

  // ROS transport
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode.reset(new ros::NodeHandle());

  this->startTaskRosService = this->rosNode->advertiseService(
      "/srcsim/finals/start_task", &FinalsPlugin::OnStartTaskRosRequest, this);

  this->scoreRosPub = this->rosNode->advertise<srcsim::Score>(
      "/srcsim/finals/score", 1000);

  this->taskRosSub = this->rosNode->subscribe("/srcsim/finals/task", 10,
      &FinalsPlugin::OnTaskRosMsg, this);

  gzmsg << "Finals plugin loaded. Call start service to start a task."
        << std::endl;
}

/////////////////////////////////////////////////
bool FinalsPlugin::OnStartTaskRosRequest(srcsim::StartTask::Request &_req,
    srcsim::StartTask::Response &_res)
{
  _res.success = true;

  // Check if task exists
  if (_req.task_id > 3 || _req.task_id < 1)
  {
    gzerr << "Trying to start task [" << unsigned(_req.task_id) <<
        "]. Task numbers go from 1 to 3." << std::endl;
    _res.success = false;
    return true;
  }

  // In case world doesn't have all tasks
  if (this->product % 30 != 0)
  {
    // Task 1
    if (_req.task_id == 1 && this->product % 2 != 0)
      _res.success = false;

    // Task 2
    if (_req.task_id == 2 && this->product % 3 != 0)
      _res.success = false;

    // Task 3
    if (_req.task_id == 3 && this->product % 5 != 0)
      _res.success = false;

    if (!_res.success)
    {
      gzerr << "Trying to start task [" << unsigned(_req.task_id) <<
          "] but this task is not in the world." << std::endl;
      return true;
    }
  }

  // Check if checkpoint exists
  if (_req.checkpoint_id > this->tasks[_req.task_id - 1]->CheckpointCount() ||
      _req.checkpoint_id < 1)
  {
    gzerr << "Trying to start task [" << unsigned(_req.task_id) <<
        "] checkpoint [" << unsigned(_req.checkpoint_id) <<
        "]. Checkpoint numbers for this task go from 1 to " <<
        this->tasks[_req.task_id - 1]->CheckpointCount() <<"." << std::endl;
    _res.success = false;
    return true;
  }

  // Don't allow going back
  if (_req.task_id < this->current)
  {
    gzerr << "Trying to start task [" << unsigned(_req.task_id) <<
        "], and current task is [" << unsigned(this->current) << "]. " <<
        "It's not possible to go back to a previous task."
        << std::endl;
    _res.success = false;
    return true;
  }

  if (_req.task_id == this->current && _req.checkpoint_id <=
      this->tasks[_req.task_id - 1]->CurrentCheckpointId())
  {
    gzerr << "Trying to start task [" << unsigned(this->current) <<
        "] checkpoint [" << unsigned(_req.checkpoint_id) <<
        "], and current checkpoint is [" <<
        this->tasks[_req.task_id - 1]->CurrentCheckpointId() << "]. " <<
        "It's not possible to go back to a previous checkpoint." << std::endl;
    _res.success = false;
    return true;
  }

  // We won't allow skipping directly to a few checkpoints. Shortcut this here.
  if ((_req.task_id == 2 && _req.checkpoint_id == 2) ||
      (_req.task_id == 2 && _req.checkpoint_id == 5) ||
      (_req.task_id == 3 && _req.checkpoint_id == 5) ||
      (_req.task_id == 3 && _req.checkpoint_id == 7))
  {
    auto tid = unsigned(_req.task_id);
    auto cpid = unsigned(_req.checkpoint_id);
    auto prevCp = unsigned(cpid - 1);
    auto nextCp = unsigned(cpid + 1);
    gzerr << "It's not possible to skip to task [" << tid << "] checkpoint ["
          << cpid << "] (" << tid << "/" << cpid
          << "). You can either complete " << tid << "/" << prevCp
          << " and then skip to " << tid << "/" << nextCp
          << ", or skip straight to " << tid << "/" << nextCp << ". "
          << std::endl;
    _res.success = false;
    return true;
  }

  auto time = this->world->GetSimTime();

  // Tasks being skipped
  while (this->current < _req.task_id)
  {
    if (this->current > 0)
    {
      if (this->tasks[this->current - 1])
        this->tasks[this->current - 1]->Skip();

      gzmsg << "Task [" << unsigned(this->current)  << "] - Skipped (" << time
            << ")" << std::endl;
    }

    this->current++;
  }

  this->current = _req.task_id;

  // Start task
  this->tasks[this->current - 1]->Start(time, _req.checkpoint_id);

  // Start update
  if (!this->updateConnection)
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FinalsPlugin::OnUpdate, this, std::placeholders::_1));
  }

  _res.success = true;
  return true;
}

/////////////////////////////////////////////////
void FinalsPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (this->current == 0)
    return;

  this->tasks[this->current - 1]->Update(_info.simTime);

  // Publish ROS score message
  srcsim::Score msg;

  // Check up to the current task
  for (auto i = 1; i <= this->current; ++i)
  {
    // Skip tasks which haven't been created
    if (!this->tasks[i-1])
    {
      continue;
    }

    // Add completion time for all past checkpoints
    for (size_t j = 1; j <= this->tasks[i-1]->CheckpointCount(); ++j)
    {
      if (i == this->current && j >= this->tasks[i-1]->CurrentCheckpointId()) {
        // Skip the current (and future) checkpoints of the current task
        continue;
      }
      ros::Time t(this->tasks[i-1]->GetCheckpointCompletion(j-1).Double());
      msg.checkpoints_completion.push_back(t);
    }
  }

  // Compute score based on checkpoint completion times from all tasks
  msg.score = 0;
  uint8_t last_checkpoint_score = 0;
  for (auto t : msg.checkpoints_completion)
  {
    // TODO The data structure doesn't provide the information if the
    // checkpoint was reset (which isn't suppot yet). In that case the
    // last_checkpoint_score needs to be reset too.
    if (t.isZero())
    {
      // End winning streak in case of incomplete checkpoints
      last_checkpoint_score = 0;
    }
    else
    {
      // Next checkpoint gets one more point than previous checkpoint
      last_checkpoint_score += 1;
      msg.score += last_checkpoint_score;
    }
  }
  this->scoreRosPub.publish(msg);
}

/////////////////////////////////////////////////
void FinalsPlugin::OnTaskRosMsg(const srcsim::Task::ConstPtr &_msg)
{
  bool finishedTask = false;

  if (_msg->timed_out)
  {
    gzmsg << "Task [" << _msg->task << "] timed out." << std::endl;
    finishedTask = true;
  }

  if (_msg->finished)
  {
    gzmsg << "Task [" << _msg->task << "] finished." << std::endl;
    finishedTask = true;
  }

  if (!finishedTask)
    return;

  if ( _msg->task < 3 && this->tasks[this->current])
  {
    // Start next task
    this->current++;
    this->tasks[this->current - 1]->Start(this->world->GetSimTime(), 1);
  }
  else
  {
    gzmsg << "All tasks have finished." << std::endl;
    this->updateConnection.reset();
  }
}

