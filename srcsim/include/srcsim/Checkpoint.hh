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

#ifndef SRC_CHECKPOINT_HH_
#define SRC_CHECKPOINT_HH_

#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  class Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _startPose Pose to start from if skipped to this checkpoint.
    public: Checkpoint(const ignition::math::Pose3d &_startPose);

    /// \brief Check whether checkpoint has been completed.
    /// Any publishers or subscribers are created the first time this is
    /// called, and cleaned up once it returns true.
    /// \return True if completed.
    public: virtual bool Check() = 0;

    /// \brief Start pose.
    public: ignition::math::Pose3d startPose;
  };

  /// \brief A checkpoint tied to a BoxContainsPlugin.
  class BoxCheckpoint : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Callback when messages are received from the BoxContainsPlugin.
    /// \param[in] _msg 1 if robot is inside box, 0 otherwise.
    public: void OnBox(ConstIntPtr &_msg);

    /// \brief Check whether the box checkpoint has been completed.
    /// \return True if completed.
    protected: bool CheckBox(const std::string &_namespace);

    /// \brief Gazebo transport node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo subscriber of box contains messages.
    private: transport::SubscriberPtr boxSub;

    /// \brief Gazebo publisher of toggle messages.
    private: transport::PublisherPtr togglePub;

    /// \brief Flag to indicate whether the robot has reached the box.
    private: bool boxDone = false;
  };
}
#endif
