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
#ifndef SRCSIM_ROS_HARNESS_H
#define SRCSIM_ROS_HARNESS_H

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

#include <srcsim/SRCHarnessPlugin.hh>

namespace gazebo
{
/// \brief See the Gazebo documentation about the HarnessPlugin. This ROS
/// wrapper exposes three topics:
///
///  1. /<plugin_model_name>/harness/velocity
///      - Message Type: std_msgs::Float32
///      - Purpose: Set target winch velocity
///
///  2. /<plugin_model_name>/harness/detach
///      - Message Type: std_msgs::Bool
///      - Purpose: Detach the <detach> joint.
///
///  3. /<plugin_model_name>/harness/attach
///      - Message Type: geometry_msgs::Pose
///      - Purpose: Teleport the model to specified pose and re-attach harness.
class SRCSimRosHarness : public SRCHarnessPlugin
{
    /// \brief Constructor
    public: SRCSimRosHarness();

    /// \brief Destructor
    public: virtual ~SRCSimRosHarness();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Receive winch velocity control messages.
    /// \param[in] msg Float message that is the target winch velocity.
    private: virtual void OnVelocity(const std_msgs::Float32::ConstPtr &msg);

    /// \brief Receive detach messages
    /// \param[in] msg Boolean detach message. Detach joints if data is
    /// true.
    private: virtual void OnDetach(const std_msgs::Bool::ConstPtr &msg);

    /// \brief Receive attach messages
    /// \param[in] msg Pose attach message. Teleport and attach harness.
    private: virtual void OnAttach(const geometry_msgs::Pose::ConstPtr &msg);

    /// \brief Custom callback queue thread
    private: void QueueThread();

    /// \brief pointer to ros node
    private: ros::NodeHandle *rosnode_;

    /// \brief Subscriber to velocity control messages.
    private: ros::Subscriber velocitySub_;

    /// \brief Subscriber to detach control messages.
    private: ros::Subscriber detachSub_;

    /// \brief Subscriber to attach control messages.
    private: ros::Subscriber attachSub_;

    /// \brief for setting ROS name space
    private: std::string robotNamespace_;
    private: ros::CallbackQueue queue_;
    private: boost::thread callbackQueueThread_;
};
}
#endif
