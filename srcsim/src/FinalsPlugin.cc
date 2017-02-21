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

#include <gazebo/common/Console.hh>

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
  this->world = _world;

  // Load SDF params
  int task1Timeout = 10;
  if (_sdf->HasElement("task_1_timeout"))
    task1Timeout = _sdf->Get<int>("task_1_timeout");

  int task2Timeout = 10;
  if (_sdf->HasElement("task_2_timeout"))
    task2Timeout = _sdf->Get<int>("task_2_timeout");

  int task3Timeout = 10;
  if (_sdf->HasElement("task_3_timeout"))
    task3Timeout = _sdf->Get<int>("task_3_timeout");

  ignition::math::Pose3d t1cp2Pose;
  if (_sdf->HasElement("task_1_checkpoint_2_pose"))
    t1cp2Pose = _sdf->Get<ignition::math::Pose3d>("task_1_checkpoint_2_pose");

  ignition::math::Pose3d t1cp3Pose;
  if (_sdf->HasElement("task_1_checkpoint_3_pose"))
    t1cp3Pose = _sdf->Get<ignition::math::Pose3d>("task_1_checkpoint_3_pose");

  ignition::math::Pose3d t2cp1Pose;
  if (_sdf->HasElement("task_2_checkpoint_1_pose"))
    t2cp1Pose = _sdf->Get<ignition::math::Pose3d>("task_2_checkpoint_1_pose");

  ignition::math::Pose3d t2cp2Pose;
  if (_sdf->HasElement("task_2_checkpoint_2_pose"))
    t2cp2Pose = _sdf->Get<ignition::math::Pose3d>("task_2_checkpoint_2_pose");

  ignition::math::Pose3d t2cp3Pose;
  if (_sdf->HasElement("task_2_checkpoint_3_pose"))
    t2cp3Pose = _sdf->Get<ignition::math::Pose3d>("task_2_checkpoint_3_pose");

  ignition::math::Pose3d t2cp4Pose;
  if (_sdf->HasElement("task_2_checkpoint_4_pose"))
    t2cp4Pose = _sdf->Get<ignition::math::Pose3d>("task_2_checkpoint_4_pose");

  ignition::math::Pose3d t2cp5Pose;
  if (_sdf->HasElement("task_2_checkpoint_5_pose"))
    t2cp5Pose = _sdf->Get<ignition::math::Pose3d>("task_2_checkpoint_5_pose");

  ignition::math::Pose3d t2cp6Pose;
  if (_sdf->HasElement("task_2_checkpoint_6_pose"))
    t2cp6Pose = _sdf->Get<ignition::math::Pose3d>("task_2_checkpoint_6_pose");

  ignition::math::Pose3d t3cp1Pose;
  if (_sdf->HasElement("task_3_checkpoint_1_pose"))
    t3cp1Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_1_pose");

  ignition::math::Pose3d t3cp2Pose;
  if (_sdf->HasElement("task_3_checkpoint_2_pose"))
    t3cp2Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_2_pose");

  ignition::math::Pose3d t3cp3Pose;
  if (_sdf->HasElement("task_3_checkpoint_3_pose"))
    t3cp3Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_3_pose");

  ignition::math::Pose3d t3cp4Pose;
  if (_sdf->HasElement("task_3_checkpoint_4_pose"))
    t3cp4Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_4_pose");

  ignition::math::Pose3d t3cp5Pose;
  if (_sdf->HasElement("task_3_checkpoint_5_pose"))
    t3cp5Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_5_pose");

  ignition::math::Pose3d t3cp6Pose;
  if (_sdf->HasElement("task_3_checkpoint_6_pose"))
    t3cp6Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_6_pose");

  ignition::math::Pose3d t3cp7Pose;
  if (_sdf->HasElement("task_3_checkpoint_7_pose"))
    t3cp7Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_7_pose");

  ignition::math::Pose3d t3cp8Pose;
  if (_sdf->HasElement("task_3_checkpoint_8_pose"))
    t3cp8Pose = _sdf->Get<ignition::math::Pose3d>("task_3_checkpoint_8_pose");

  std::vector<ignition::math::Pose3d> t1Poses = {t1cp2Pose, t1cp3Pose};
  std::vector<ignition::math::Pose3d> t2Poses = {t2cp1Pose, t2cp2Pose,
      t2cp3Pose, t2cp4Pose, t2cp5Pose, t2cp6Pose};
  std::vector<ignition::math::Pose3d> t3Poses = {t3cp1Pose, t3cp2Pose,
      t3cp3Pose, t3cp4Pose, t3cp5Pose, t3cp6Pose, t3cp7Pose, t3cp8Pose};

  // Instantiate tasks

  // Task 1: Satellite Dish
  std::unique_ptr<Task1> task1(new Task1(common::Time(task1Timeout), t1Poses));
  this->tasks.push_back(std::move(task1));

  // Task 2: Solar panel
  std::unique_ptr<Task2> task2(new Task2(common::Time(task2Timeout), t2Poses));
  this->tasks.push_back(std::move(task2));

  // Task 3: Habitat
  std::unique_ptr<Task3> task3(new Task3(common::Time(task3Timeout), t3Poses));
  this->tasks.push_back(std::move(task3));

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

  this->taskRosSub = this->rosNode->subscribe("/srcsim/finals/task", 10,
      &FinalsPlugin::OnTaskRosMsg, this);

  gzmsg << "Finals plugin loaded. Call start service to start a task."
        << std::endl;
}

/////////////////////////////////////////////////
bool FinalsPlugin::OnStartTaskRosRequest(srcsim::StartTask::Request &_req,
    srcsim::StartTask::Response &_res)
{
  // Check if task exists
  if (_req.task_id > 3 || _req.task_id < 1)
  {
    gzerr << "Trying to start task [" << unsigned(_req.task_id) <<
        "]. Task numbers go from 1 to 3." << std::endl;
    _res.success = false;
    return true;
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

  if (_req.task_id == this->current && _req.checkpoint_id <= this->current)
  {
    gzerr << "Trying to start task [" << unsigned(this->current) <<
        "] checkpoint [" << unsigned(_req.checkpoint_id) <<
        "], and current checkpoint is [" <<
        this->tasks[_req.task_id - 1]->CurrentCheckpointId() << "]. " <<
        "It's not possible to go back to a previous checkpoint." << std::endl;
    _res.success = false;
    return true;
  }

  auto time = this->world->GetSimTime();

  // Check if there are tasks being skipped
  bool skipped = false;
  while (this->current < _req.task_id)
  {
    if (this->current > 0)
    {
      gzmsg << "Task [" << unsigned(this->current)  << "] - Skipped (" << time
            << ")" << std::endl;
      skipped = true;
    }

    this->current++;
  }

  // Start update
  if (!this->updateConnection)
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FinalsPlugin::OnUpdate, this, std::placeholders::_1));
  }

  this->current = _req.task_id;

  // Start task
  this->tasks[this->current - 1]->Start(time, _req.checkpoint_id, skipped);

  _res.success = true;
  return true;
}

/////////////////////////////////////////////////
void FinalsPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (this->current == 0)
    return;

  this->tasks[this->current - 1]->Update(_info.simTime);
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

  if ( _msg->task < 3)
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

