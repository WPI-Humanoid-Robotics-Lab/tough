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

#ifndef SRC_HARNESSMANAGER_HH_
#define SRC_HARNESSMANAGER_HH_

#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include <gazebo/common/SingletonT.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  /// \brief Manages harness
  class HarnessManager : public SingletonT<HarnessManager>
  {
    /// \brief Constructor
    private: HarnessManager();

    /// \brief Destructor
    private: virtual ~HarnessManager();

    /// \brief Add a new goal, overriding previous one
    /// \brief _pose Goal pose to be attached
    public: void NewGoal(const ignition::math::Pose3d &_pose);

    /// \brief Update harness state
    /// \brief _time Sim time
    public: void Update(const common::Time &_time);

    /// \brief Send a message requesting detach.
    public: void TriggerDetach();

    /// \brief Send a message requesting attach, according to the latest pose
    /// goal.
    public: void TriggerAttach();

    /// \brief Send a message requesting to lower the harness.
    public: void TriggerLower();

    /// \brief Send a message requesting to switch to high level control.
    public: void TriggerStand();

    /// \brief Returns true if harness is detached
    /// \return True if detached
    public: bool IsDetached();

    /// \brief Returns true if harness is attached
    /// \return True if attached
    public: bool IsAttached();

    /// \brief Returns true if harness has been lowered enough
    /// \return True if lowered
    public: bool IsLowered();

    /// \brief Returns true if robot can stand without the harness
    /// \return True if standing
    public: bool IsStanding();

    /// \brief Callback when receiving wrench message from ankle.
    /// \param[in] _msg The message
    private: void OnSensorMsg(ConstWrenchStampedPtr &_msg);

    /// \brief Enum of possible transitions
    private: enum Transition
    {
      NONE,
      DETACH_TO_ATTACH,
      ATTACH_TO_LOWER,
      LOWER_TO_STAND,
      STAND_TO_DETACH,
      DETACH_TO_NONE,
    };

    /// \brief Current transition
    private: Transition transition = NONE;

    /// \brief latest pose goal
    private: ignition::math::Pose3d goal;

    /// \brief True if a new goal has been received
    private: bool goalChanged = false;

    /// \brief Pointer to model being harnessed (hardcoded valkyrie)
    private: gazebo::physics::ModelPtr model;

    /// \brief Stores force on the ankle from latest sensor message.
    private: double latestAnkleForce = 0.0;

    /// \brief Number of iterations to check sensor before being sure that robot
    /// has been lowered or is standing.
    private: const unsigned int itThreshold = 500;

    /// \brief How many iterations has it been lowered
    private: unsigned int itLowering = 0;

    /// \brief How many iterations has it been standing
    private: unsigned int itStanding = 0;

    /// \brief Gazebo node for communication
    private: gazebo::transport::NodePtr gzNode;

    /// \brief Publishes attach messages
    private: gazebo::transport::PublisherPtr attachGzPub;

    /// \brief Publishes detach messages
    private: gazebo::transport::PublisherPtr detachGzPub;

    /// \brief Publishes lower messages
    private: gazebo::transport::PublisherPtr lowerGzPub;

    /// \brief Subscribe to sensor messages
    private: gazebo::transport::SubscriberPtr sensorGzSub;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros publisher for control messages.
    private: ros::Publisher controlRosPub;

    /// \brief Ros publisher for harness status messages.
    private: ros::Publisher statusRosPub;

    // Singleton implementation
    private: friend class SingletonT<HarnessManager>;
  };
  /// \}
}
#endif
