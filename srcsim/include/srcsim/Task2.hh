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
#include <ignition/math/Pose3.hh>

#include "Checkpoint.hh"
#include "Task.hh"

namespace gazebo
{
  class Task2 : public Task
  {
    /// \brief Constructor
    /// \param[in] _timeout Timeout for this task
    /// \param[in] _poses Vector of starting poses for checkpoints.
    public: Task2(const common::Time &_timeout,
        const std::vector<ignition::math::Pose3d> _poses);

    // Documentation inherited
    public: size_t Number() const;
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
