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

#ifndef SRC_TASK_HH_
#define SRC_TASK_HH_

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <sdf/sdf.hh>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>

#include <gazebo/transport/transport.hh>

#include "Checkpoint.hh"

namespace gazebo
{
  /// \brief Base class for all 3 tasks.
  class Task
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this task.
    public: Task(const sdf::ElementPtr &_sdf);

    /// \brief Start this task at a specific checkpoint
    /// \param[in] _time Start time
    /// \param[in] _checkpoint Checkpoint Id
    public: void Start(const common::Time &_time, const size_t _checkpoint);

    /// \brief Update this task
    /// \param[in] _time Current time
    public: void Update(const common::Time &_time);

    /// \brief Skip this task
    public: void Skip();

    /// \brief Return the number of checkpoints in this task.
    /// \return Number of checkpoints.
    public: size_t CheckpointCount() const;

    /// \brief Return the index of the current checkpoint.
    /// \return Index of current checkpoint.
    public: size_t CurrentCheckpointId() const;

    /// \brief Return the completion time of a checkpoint.
    /// \param[in] _index Index of the checkpoint
    /// \return The competion time
    public: common::Time GetCheckpointCompletion(size_t index) const;

    /// \brief Return this task's number.
    /// \return Task number.
    public: virtual size_t Number() const = 0;

    /// \brief Callback when messages are received from the BoxContainsPlugin.
    /// \param[in] _msg 1 if robot is inside box, 0 otherwise.
    private: void OnStartBox(ConstIntPtr &_msg);

    /// \brief Vector of checkpoints for this task.
    /// checkpoints[0]: Checkpoint 1
    /// checkpoints[1]: Checkpoint 2
    /// checkpoints[2]: Checkpoint 3
    /// ...
    protected: std::vector<std::unique_ptr<Checkpoint> > checkpoints;

    /// \brief Current checkpoint number, starting from 1. Zero means the task
    /// hasn't started, Count+1 means that the task has finished.
    protected: size_t current = 0;

    /// \brief Vector of times when checkpoints were completed.
    /// Time is zero for skipped checkpoints.
    private: std::vector<common::Time> checkpointsCompletion;

    /// \brief Time when the task started
    private: common::Time startTime;

    /// \brief Total time allowed for the task.
    private: common::Time timeout = 300;

    /// \brief True if task has been completed before timeout.
    private: bool finished = false;

    /// \brief True if task has timed out before completion.
    private: bool timedOut = false;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros publisher which publishes the task's status.
    private: ros::Publisher taskRosPub;

    /// \brief Gazebo transport node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo subscriber of box contains messages.
    private: transport::SubscriberPtr boxSub;

    /// \brief Gazebo publisher of toggle messages.
    private: transport::PublisherPtr togglePub;

    /// \brief Flag to indicate whether the robot has reached the start box.
    private: bool startBox = false;
  };
}
#endif
