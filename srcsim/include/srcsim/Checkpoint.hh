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
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  class Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    public: Checkpoint(const sdf::ElementPtr &_sdf);

    /// \brief Check whether checkpoint has been completed.
    /// Any publishers or subscribers are created the first time this is
    /// called, and cleaned up once it returns true.
    /// \return True if completed.
    public: virtual bool Check() = 0;

    /// \brief Skip this checkpoint.
    /// This function rearranges the world as if the checkpoint had been
    /// completed.
    /// The base implementation teleports the robot, but checkpoints can
    /// override the function to move other objects too.
    public: virtual void Skip();

    /// \brief The pose the robot should be in when this checkpoint is skipped.
    public: ignition::math::Pose3d robotSkipPose;
  };

  /// \brief A checkpoint tied to a BoxContainsPlugin.
  class BoxCheckpoint : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Check whether the box checkpoint has been completed.
    /// \return True if completed.
    protected: bool CheckBox(const std::string &_namespace);

    /// \brief Callback when messages are received from the BoxContainsPlugin.
    /// \param[in] _msg 1 if robot is inside box, 0 otherwise.
    private: void OnBox(ConstIntPtr &_msg);

    /// \brief Gazebo transport node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo subscriber of box contains messages.
    private: transport::SubscriberPtr boxSub;

    /// \brief Gazebo publisher of toggle messages.
    private: transport::PublisherPtr togglePub;

    /// \brief Flag to indicate whether the robot has reached the box.
    private: bool boxDone = false;
  };

  /// \brief A checkpoint tied to a TouchPlugin.
  class TouchCheckpoint : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Callback when a touch message is received.
    /// This means the touch is complete.
    /// \param[in] _msg Unused message.
    public: void OnTouchGzMsg(ConstIntPtr &_msg);

    /// \brief Check whether the touch checkpoint has been completed.
    /// \param[in] _namespace Namespace for the Gazebo topics in this plugin.
    /// \return True if completed.
    protected: bool CheckTouch(const std::string &_namespace);

    /// \brief Gazebo transport node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo subscriber of touch messages.
    private: transport::SubscriberPtr touchGzSub;

    /// \brief Gazebo publisher of enable messages.
    private: transport::PublisherPtr enableGzPub;

    /// \brief Flag to indicate whether the touch is complete.
    private: bool touchDone = false;
  };
}
#endif
