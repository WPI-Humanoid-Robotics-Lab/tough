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

#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo
{
  class SatelliteDishPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SatelliteDishPlugin();

    // Documentation inherited
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for enable "service".
    /// \param[in] _msg 0 to stop, 1 to start
    public: void Enable(ConstIntPtr &_msg);

    /// \brief Called on every world update.
    /// \param[in] _info Info about the world.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Ratio between wheel and dish yaw.
    private: double yawRatio;

    /// \brief Ratio between wheel and dish pitch.
    private: double pitchRatio;

    /// \brief Target yaw angle in radians.
    private: double yawTarget;

    /// \brief Target pitch angle in radians.
    private: double pitchTarget;

    /// \brief Publish frequecy in Hz.
    private: double frequency;

    /// \brief Angle tolerance in degrees.
    private: double tolerance;

    /// \brief Topic for satellite messages.
    private: std::string satTopic;

    /// \brief Pointer to the yaw wheel joint.
    private: physics::JointPtr yawWheel;

    /// \brief Pointer to the pitch wheel joint.
    private: physics::JointPtr pitchWheel;

    /// \brief Pointer to the yaw dish joint.
    private: physics::JointPtr yawJoint;

    /// \brief Pointer to the pitch dish joint.
    private: physics::JointPtr pitchJoint;

    /// \brief Pointer to the joint controller.
    private: physics::JointControllerPtr controller;

    /// \brief Time to hold the correct angles.
    private: common::Time targetTime;

    /// \brief Time when yaw satrted being correct.
    private: common::Time yawCorrectStart;

    /// \brief Time when pitch satrted being correct.
    private: common::Time pitchCorrectStart;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ros satellite publisher
    private: ros::Publisher satelliteRosPub;

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Subscriber to enable messages.
    private: transport::SubscriberPtr enableSub;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;
  };
  GZ_REGISTER_MODEL_PLUGIN(SatelliteDishPlugin)
}

