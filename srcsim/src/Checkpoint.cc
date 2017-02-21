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

#include <gazebo/common/Console.hh>

#include "srcsim/Checkpoint.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Checkpoint::Checkpoint(const ignition::math::Pose3d &_startPose)
    : startPose(_startPose)
{
}

/////////////////////////////////////////////////
bool BoxCheckpoint::CheckBox(const std::string &_namespace)
{
  // First time checking
  if (!this->gzNode && !this->boxDone)
  {
    // Initialize node
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Setup contains subscriber
    this->boxSub = this->gzNode->Subscribe(_namespace + "/box/contains",
        &BoxCheckpoint::OnBox, this);

    // Setup toggle publisher
    auto toggleTopic = _namespace + "/box/toggle";
    this->togglePub = this->gzNode->Advertise<msgs::Int>(toggleTopic);

    // Toggle box plugin on
    msgs::Int msg;
    msg.set_data(1);
    this->togglePub->Publish(msg);
  }

  if (this->boxDone && this->togglePub)
  {
    // Toggle box plugin on
    msgs::Int msg;
    msg.set_data(0);
    this->togglePub->Publish(msg);

    // Clear transport
    this->gzNode->Fini();
    this->togglePub.reset();
    this->gzNode.reset();
  }

  return this->boxDone;
}

//////////////////////////////////////////////////
void BoxCheckpoint::OnBox(ConstIntPtr &_msg)
{
  this->boxDone = _msg->data() == 0 ? false : true;
}

