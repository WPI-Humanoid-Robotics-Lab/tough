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

#include <sstream>
#include <ignition/math/Rand.hh>
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/transport/Publisher.hh"

#include "srcsim/Qual1Plugin.hh"
using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Qual1Plugin)

/////////////////////////////////////////////////
Qual1Plugin::Qual1Plugin()
{
}

/////////////////////////////////////////////////
void Qual1Plugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  char *homePath = getenv("HOME");
  if (!homePath)
  {
    gzerr << "Unable to get HOME environment variable. Report this error to "
      << "an SRC official.\n";
    return;
  }
  std::string logPath = homePath;
  logPath += "/src_qual1_" + common::Time::GetWallTimeAsISOString() + ".log";

  this->logStream.open(logPath.c_str(), std::ios::out);
  if (!this->logStream.is_open())
  {
    gzerr << "Unable to open log file[" << logPath << "].\n"
      << "Make sure permissions are set correctly, and then retry.\n";
    return;
  }

  // Output header information
  this->Log("# switch <light_index> <r> <g> <b> <a> <sim_sec> <sim_nsec>",
      false);
  this->Log("# answer <x> <y> <z> <r> <g> <b> <sim_sec> <sim_nsec>", false);

  this->world = _world;

  this->node = transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();

  // Wait for a subscriber to connect
  this->pub = this->node->Advertise<gazebo::msgs::Visual>("~/visual");
  this->pub->WaitForConnection();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Load SDF parameters.
  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement("robot_namespace")->Get<std::string>() +
      "/";
  }

  this->rosnode.reset(new ros::NodeHandle(robotNamespace));

  this->lightSub = this->rosnode->subscribe("/srcsim/qual1/start", 10,
      &Qual1Plugin::OnStart, this);
  this->startSub = this->rosnode->subscribe("/srcsim/qual1/light", 10,
      &Qual1Plugin::OnLight, this);

  this->prevLightTime = _world->GetSimTime();

  int maxLight = 43;
  int numLightSwitches = 10;
  common::Time onDelay(1, 0);
  common::Time offDelay(5, 0);

  if (_sdf->HasElement("switch_count"))
    numLightSwitches = _sdf->Get<int>("switch_count");

  if (_sdf->HasElement("on_delay"))
    onDelay.sec = _sdf->Get<int>("on_delay");

  this->lightPattern.push_back({1, 44, {5, 0}, gazebo::common::Color::White});

  std::vector<gazebo::common::Color> colors =
  {
    gazebo::common::Color::Red,
    gazebo::common::Color::Blue,
    gazebo::common::Color::Green,
  };

  // Generate random lights
  for (int i = 0; i < numLightSwitches; ++i)
  {
    int light = ignition::math::Rand::IntUniform(1, maxLight);
    int console = 1;

    int colorIdx = ignition::math::Rand::IntUniform(0, colors.size() - 1);

    offDelay.sec = ignition::math::Rand::IntUniform(5, 10);
    this->lightPattern.push_back({console, light, onDelay,
        colors[colorIdx]});
    this->lightPattern.push_back({console, light, offDelay,
        gazebo::common::Color::Black});
  }
  this->lightPattern.push_back({1, 44, {5, 0}, gazebo::common::Color::Black});

  this->lightPatternIter = this->lightPattern.begin();

}

/////////////////////////////////////////////////
void Qual1Plugin::Switch(int _light, const gazebo::common::Color &_clr)
{
  std::ostringstream name, parent;
  name << "console1::visuals::light" << _light;
  parent << "console1::visuals";

  // Construct light visual message.
  gazebo::msgs::Visual msg;
  msg.set_name(name.str());
  msg.set_parent_name(parent.str());
  gazebo::msgs::Set(msg.mutable_material()->mutable_ambient(), _clr);
  gazebo::msgs::Set(msg.mutable_material()->mutable_diffuse(), _clr);
  gazebo::msgs::Set(msg.mutable_material()->mutable_emissive(), _clr);

  // Change the light
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void Qual1Plugin::OnStart(const std_msgs::EmptyConstPtr & /*_msg*/)
{
  this->Log("start", true);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&Qual1Plugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void Qual1Plugin::OnLight(const srcsim::ConsoleConstPtr &_msg)
{
  // Log the answer
  std::ostringstream stream;
  stream << "answer " << _msg->x << " " << _msg->y << " " << _msg->z
    << " " << _msg->r << " " << _msg->g << " " << _msg->b;
  std::lock_guard<std::mutex> lock(this->iterMutex);
  (*this->lightPatternIter).delay.sec = 0;
  this->Log(stream.str(), true);
}

/////////////////////////////////////////////////
void Qual1Plugin::Log(const std::string &_string, const bool _stamp)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->logStream << _string;
  if (_stamp)
  {
    this->logStream << " " << this->world->GetSimTime().sec
      << " " << this->world->GetSimTime().nsec;
  }
  this->logStream << std::endl;
}

/////////////////////////////////////////////////
void Qual1Plugin::OnUpdate()
{
  // Stop when the light pattern reaches the end
  if (this->lightPatternIter == this->lightPattern.end())
    return;

  std::lock_guard<std::mutex> lock(this->iterMutex);

  // Compute the time difference.
  gazebo::common::Time dt = this->world->GetSimTime() - this->prevLightTime;

  // Update the lights.
  if (dt >= (*this->lightPatternIter).delay)
  {
    // Log light change data
    std::ostringstream stream;
    stream << "switch " << (*this->lightPatternIter).light << " "
           << (*this->lightPatternIter).color;
    this->Log(stream.str(), true);

    // Switch the light
    this->Switch((*this->lightPatternIter).light,
        (*this->lightPatternIter).color);

    this->lightPatternIter++;
    this->prevLightTime = this->world->GetSimTime();
  }
}
