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
#ifndef GAZEBO_PLUGINS_WINDPLUGIN_HH_
#define GAZEBO_PLUGINS_WINDPLUGIN_HH_

#include <fstream>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include "srcsim/Console.h"

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \brief Plugin that controls lights for SRC qual task 1.
  class GAZEBO_VISIBLE Qual1Plugin : public WorldPlugin
  {
    /// \brief Constructor
    public: Qual1Plugin();

    /// \brief Load the plugin
    /// \param[in] _world Pointer to the world.
    /// \param[in] _sdf Pointer to the plugin's sdf element.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Change the color of a light
    /// \param[in] _light Number of the light (1-44)
    /// \param[in] _clr Color for the light.
    private: void Switch(int _light, const gazebo::common::Color &_clr);

    /// \brief Callback when the competitor has signaled they are ready to
    /// start.
    /// \param[in] _msg Unused empty message.
    private: void OnStart(const std_msgs::EmptyConstPtr &_msg);

    /// \brief Callback when a light answer is received from a competitor
    /// \param[in] _msg The position of the light in Val's camera frame.
    private: void OnLight(const srcsim::ConsoleConstPtr &_msg);

    /// \brief Callback for World Update events.
    private: virtual void OnUpdate();

    /// \brief Log information
    /// \param[in] _string String to output
    /// \param[in} _stamp True to output sim time with the _string
    private: void Log(const std::string &_string, const bool _stamp);

    /// \brief Gazebo transport node
    private: gazebo::transport::NodePtr node;

    /// \brief Gazebo publisher for updating visuals.
    private: gazebo::transport::PublisherPtr pub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief A class that contains light switching info
    private: class LightCtrl
             {
               /// \brief Console???
               public: int console;

               /// \brief The light index
               public: int light;

               /// \brief Delay time for the light change
               public: gazebo::common::Time delay;

               /// \brief Color to change the light to when the delay
               /// has passed.
               public: gazebo::common::Color color;
             };

    /// \brief The pattern of of light changes.
    private: std::vector<LightCtrl> lightPattern;

    /// \brief Iterator to lightPattern
    private: std::vector<LightCtrl>::iterator lightPatternIter;

    /// \brief Time of the last light change
    private: gazebo::common::Time prevLightTime;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Mutex used to prevent interleaved messages.
    private: std::mutex mutex;

    /// \brief Log file output stream;
    private: std::ofstream logStream;

    /// \brief Ros node handle
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Ros light subscriber
    private: ros::Subscriber lightSub;

    /// \brief Ros start subscriber
    private: ros::Subscriber startSub;

    private: std::mutex iterMutex;
  };
}

#endif
