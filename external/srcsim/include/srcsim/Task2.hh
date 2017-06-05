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

#ifndef SRC_TASK2_HH_
#define SRC_TASK2_HH_

#include <vector>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/transport/transport.hh>

#include "Checkpoint.hh"
#include "Task.hh"

namespace gazebo
{
  class Task2 : public Task
  {
    /// \brief Constructor
    /// \param[in] _sdf Pointer to SDF element for this task.
    public: Task2(const sdf::ElementPtr &_sdf);

    // Documentation inherited
    public: size_t Number() const;
  };

  /// \brief Task 2, Checkpoint 1: Lift panel
  class Task2CP1 : public TouchCheckpoint
  {
    using TouchCheckpoint::TouchCheckpoint;

    /// \brief Check whether the panel has been touching the robot and
    /// nothing else for long enough.
    /// \return True if the checkpoint is complete.
    public: bool Check();

    /// \brief Reharness at start box
    /// \param[in] _penalty Penalty time to add
    public: void Restart(const common::Time &_penalty);
  };

  /// \brief Task 2, Checkpoint 2: Place panel near cable
  class Task2CP2 : public BoxCheckpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf Pointer to SDF element for this checkpoint.
    public: Task2CP2(const sdf::ElementPtr &_sdf);

    /// \brief Check whether the panel is within reach of the cable.
    /// \return True if the checkpoint is complete.
    public: bool Check();

    /// \brief Skip this checkpoint. This places the solar panel on the array.
    public: void Skip();

    /// \brief
    private: ignition::math::Pose3d panelSkipPose;
  };

  /// \brief Task 2, Checkpoint 3: Deploy solar panel
  class Task2CP3 : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Check whether the solar panel has been deployed.
    /// \return True if the checkpoint is complete.
    public: bool Check();

    /// \brief Skip this checkpoint. This deploys the solar panel.
    public: void Skip();

    /// \brief Callback when a message about the solar panel is received.
    /// This means the panel has been opened.
    /// \param[in] _msg Unused message.
    private: void OnSolarPanelGzMsg(ConstIntPtr &/*_msg*/);

    /// \brief Whether the checkpoint is complete or not.
    private: bool panelDone = false;

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Subscribes to solar panel messages.
    private: transport::SubscriberPtr panelGzSub;

    /// \brief Publishes enable messages.
    private: transport::PublisherPtr enableGzPub;
  };

  /// \brief Task 2, Checkpoint 4: Lift cable
  class Task2CP4 : public TouchCheckpoint
  {
    using TouchCheckpoint::TouchCheckpoint;

    /// \brief Check whether the cable's tip has been touching the robot and
    /// nothing else for long enough.
    /// \return True if the checkpoint is complete.
    public: bool Check();
  };

  /// \brief Task 2, Checkpoint 5: Plug cable
  class Task2CP5 : public Checkpoint
  {
    using Checkpoint::Checkpoint;

    /// \brief Check whether the robot is in the final box region.
    /// \return True if the checkpoint is complete.
    public: bool Check();

    /// \brief Whether the checkpoint is complete or not.
    private: bool done = false;

    /// \brief Pointer to contact sensor
    private: sensors::ContactSensorPtr sensor;

    /// \brief Pointer to the world
    private: physics::WorldPtr world;

    /// \brief Name of contact sensor
    private: std::string sensorName = "outlet_sensor";

    /// \brief Name of cable model
    private: std::string cable = "solar_panel_cable";

    /// \brief Name of link which has the cable plug
    private: std::string plug = "solar_panel_cable::link_16";

    /// \brief Name of link which has the outlet
    private: std::string outletParent = "solar_panel::panel_02";

    /// \brief Name of outlet collision
    private: std::string outlet = "solar_panel::panel_02::outlet";

    /// \brief Time when started touching.
    private: common::Time touchStart;

    /// \brief Target time to continuously touch.
    private: common::Time targetTime = 0.5;
  };

  /// \brief Task 2, Checkpoint 6: Final box
  class Task2CP6 : public BoxCheckpoint
  {
    using BoxCheckpoint::BoxCheckpoint;

    /// \brief Check whether the robot is in the final box region.
    /// \return True if the checkpoint is complete.
    public: bool Check();
  };
}
#endif
