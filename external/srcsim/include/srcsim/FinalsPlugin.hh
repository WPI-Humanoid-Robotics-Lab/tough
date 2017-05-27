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

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <srcsim/StartTask.h>
#include <srcsim/Task.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/World.hh>

#include "Task.hh"

namespace gazebo
{
  class FinalsPlugin : public WorldPlugin
  {
    // Documentation inherited
    public: FinalsPlugin();

    // Documentation inherited
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Start a specific task and checkpoint.
    /// \param[in] _req Request
    /// \param[in] _res Response
    private: bool OnStartTaskRosRequest(srcsim::StartTask::Request &_req,
        srcsim::StartTask::Response &_res);

    /// \brief Update on world update begin
    /// \param[in] _info Update info
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback when a task message is received.
    /// \param[in] _msg Task message
    private: void OnTaskRosMsg(const srcsim::Task::ConstPtr &_msg);

    /// \brief Connection to world update
    private: event::ConnectionPtr updateConnection;

    /// \brief Vector of tasks.
    /// tasks[0]: Task 1
    /// tasks[1]: Task 2
    /// tasks[2]: Task 3
    private: std::vector<std::unique_ptr<Task> > tasks;

    /// \brief Current task number, starting from 1. Zero means no task.
    private: uint8_t current = 0;

    /// \brief Pointer to the world
    private: physics::WorldPtr world;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros task service server
    private: ros::ServiceServer startTaskRosService;

    /// \brief Ros publisher which publishes the score.
    private: ros::Publisher scoreRosPub;

    /// \brief Ros task subscriber
    private: ros::Subscriber taskRosSub;

    /// \brief Product used to know which tasks are in the world
    private: int product;
  };
}
