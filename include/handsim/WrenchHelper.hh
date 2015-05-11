/*
 * Copyright 2015 Open Source Robotics Foundation
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
#ifndef _HANDSIM_HAPTIX_WRENCH_HELPER_HH_
#define _HANDSIM_HAPTIX_WRENCH_HELPER_HH_

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo
{
  class WrenchHelper
  {
    /// \brief constructor
    public: WrenchHelper() {}

    /// \brief constructor
    public: WrenchHelper(math::Vector3 _force, math::Vector3 _torque)
      : force(_force), torque(_torque)
      {}

    /// \brief Operator =
    /// \param[in] _wrench wrench to set from.
    /// \return *this
    public: WrenchHelper &operator =(const WrenchHelper &_wrench);

    /// \brief Operator +
    /// \param[in] _wrench wrench to add
    /// \return *this
    public: inline WrenchHelper &operator +(const WrenchHelper &_wrench);

    /// \brief Operator -
    /// \param[in] _wrench wrench to subtract
    /// \return *this
    public: inline WrenchHelper &operator -(const WrenchHelper &_wrench);

    /// \brief linear forces
    public: math::Vector3 force;

    /// \brief angular torques
    public: math::Vector3 torque;

    /// \brief reference link frame
    public: physics::LinkPtr referenceFrame;
  };
}
#endif
