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

#ifndef SRC_TASK1_HH_
#define SRC_TASK1_HH_

#include <vector>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>

#include <srcsim/Satellite.h>
#include "Checkpoint.hh"
#include "Task.hh"

namespace gazebo
{
  class Task1 : public Task
  {
    /// \brief Constructor
    /// \param[in] _sdf Pointer to SDF element for this task.
    public: Task1(const sdf::ElementPtr &_sdf);

    // Documentation inherited
    public: size_t Number() const;
  };

  /// \brief Task 1, Checkpoint 1: Walk to dish
  class Task1CP1 : public BoxCheckpoint
  {
    using BoxCheckpoint::BoxCheckpoint;

    /// \brief Check whether the robot is currently within the box volume in
    /// front of the satellite dish.
    public: bool Check();
  };

  /// \brief Task 1, Checkpoint 2: Satellite pitch and yaw
  class Task1CP2 : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Check whether the last message received from the satellite had
    /// the checkpoint complete.
    /// \return True if the checkpoint is compelete.
    public: bool Check();

    /// \brief Callback when a satellite message is received.
    /// \param[in] _msg Satellite message
    private: void OnSatelliteRosMsg(const srcsim::Satellite &_msg);

    /// \brief Whether the checkpoint is complete or not.
    private: bool satelliteDone = false;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Subscribes to ROS satellite messages.
    private: ros::Subscriber satelliteRosSub;
  };

  /// \brief Task 1, Checkpoint 3: Final box
  class Task1CP3 : public BoxCheckpoint
  {
    using BoxCheckpoint::BoxCheckpoint;

    /// \brief Check whether the robot is in the final box region.
    /// \return True if the checkpoint is compelete.
    public: bool Check();
  };
}
#endif
