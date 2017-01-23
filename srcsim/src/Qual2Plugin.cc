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

#include <functional>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include "srcsim/Qual2Plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Qual2Plugin)

/////////////////////////////////////////////////
Qual2Plugin::Qual2Plugin()
{
}

/////////////////////////////////////////////////
void Qual2Plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  GZ_ASSERT(_sdf, "SDF pointer is null");

  // Read the hinge joint name.
  if (!_sdf->HasElement("hinge_joint_name"))
  {
    gzerr << "<hinge_joint_name> not specified in SDF\n";
    return;
  }
  auto hingeJointName = _sdf->Get<std::string>("hinge_joint_name");
  this->hingeJoint = _model->GetJoint(hingeJointName);
  if (!this->hingeJoint)
  {
    gzerr << "Joint [" << hingeJointName << "] not found" << std::endl;
    return;
  }

  // Read the button joint name.
  if (!_sdf->HasElement("button_joint_name"))
  {
    gzerr << "<button_joint_name> not specified in SDF\n";
    return;
  }
  auto buttonJointName = _sdf->Get<std::string>("button_joint_name");
  this->buttonJoint = _model->GetJoint(buttonJointName);
  if (!this->buttonJoint)
  {
    gzerr << "Joint [" << buttonJointName << "] not found" << std::endl;
    return;
  }

  this->lowerLimit = this->buttonJoint->GetLowerLimit(0).Radian();
  this->upperLimit = this->buttonJoint->GetUpperLimit(0).Radian();
  this->range = this->upperLimit - this->lowerLimit;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&Qual2Plugin::OnUpdate, this));
}

/////////////////////////////////////////////////
bool Qual2Plugin::Pressed() const
{
  return this->pressed;
}

////////////////////////////////////////////////
void Qual2Plugin::OpenTheDoor()
{
  // Apply force on the hinge to open the door.
  this->hingeJoint->SetForce(0, 400.0);
}

////////////////////////////////////////////////
void Qual2Plugin::CloseTheDoor()
{
  // Apply force on the hinge to close the door.
  this->hingeJoint->SetForce(0, -400.0);
}


/////////////////////////////////////////////////
void Qual2Plugin::OnUpdate()
{
  // Something went wrong, nothing to do.
  if (!this->buttonJoint || !this->hingeJoint)
    return;

  int percentagePressed =
    ((this->buttonJoint->GetAngle(0).Radian() - this->lowerLimit) /
      this->range) * 100;

  this->pressed = percentagePressed >= this->kPercentageButtonPressed;

  // Trigger the opening of the door.
  if (this->pressed)
    this->opening = true;

  // Open/close the door.
  if (this->opening)
    this->OpenTheDoor();
  else
    this->CloseTheDoor();
}
