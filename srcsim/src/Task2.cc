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

#include "srcsim/Task2.hh"

using namespace gazebo;

/////////////////////////////////////////////////
Task2::Task2(const common::Time &_timeout,
    const std::vector<ignition::math::Pose3d> _poses)
    : Task(_timeout)
{
  // Checkpoint 6: Walk to final box
  std::unique_ptr<Task2CP6> cp6(new Task2CP6(_poses[0]));
  this->checkpoints.push_back(std::move(cp6));

  gzmsg << "Task [2] created" << std::endl;
}

/////////////////////////////////////////////////
size_t Task2::Number() const
{
  return 2u;
}

/////////////////////////////////////////////////
bool Task2CP6::Check()
{
  return this->CheckBox("/task2/checkpoint6");
}

