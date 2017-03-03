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
#ifndef GAZEBO_PLUGINS_SOLARPANELPLUGIN_HH_
#define GAZEBO_PLUGINS_SOLARPANELPLUGIN_HH_

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/transport/Node.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE SolarPanelPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SolarPanelPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for toggle "service".
    /// \param[in] _msg Unused message
    public: void Toggle(ConstIntPtr &/*_msg*/);

    /// \brief Update plugin's function.
    private: void OnUpdate(const common::UpdateInfo &/*_info*/);

    /// \brief Pointer to the solar panel model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the button joint.
    private: physics::JointPtr buttonJoint;

    /// \brief Pointer to the joints locking the panel closed.
    private: std::vector<physics::JointPtr> lockJoints;

    /// \brief Joints which will be actuated to open the panel.
    private: std::vector<physics::JointPtr> panelJoints;

    /// \brief Whether the button is pressed.
    private: bool pressed = false;

    /// \brief Lower limit of the button's joint.
    private: double lowerLimit;

    /// \brief Upper limit of the button's joint.
    private: double upperLimit;

    /// \brief The range of motion of the button's joint.
    private: double range;

    /// \brief Between 0% of the range and this value the button is considered
    /// not pressed. From this value to 100% the button is considered pressed.
    private: static const int kPercentageButtonPressed = 75;

    /// \brief Contact sensor attached to button
    private: sensors::ContactSensorPtr contactSensor;

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Publisher which publishes a message once the panel is open.
    private: transport::PublisherPtr openedPub;

    /// \brief Subscriber to toggle messages.
    private: transport::SubscriberPtr toggleSub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
