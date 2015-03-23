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
#ifndef _HANDSIM_HAPTIX_JOINTHELPER_HH_
#define _HANDSIM_HAPTIX_JOINTHELPER_HH_

#include <string>

#include <gazebo/math/Angle.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/Joint.hh>

namespace gazebo
{
  /// \brief keep an array of joints, some are valid gazebo joints
  /// others just keep a "fake" state here.
  class JointHelper
  {
    public: JointHelper();

    public: ~JointHelper();

    public: JointHelper &operator=(physics::JointPtr _joint);

    public: void SetJoint(physics::JointPtr _joint);

    public: void SetJointName(const std::string &_name);

    public: math::Angle GetAngle(int _index);

    public: double GetVelocity(int _index);

    public: bool SetForce(int _index, double _force);

    public: double GetForce(int _index);

    public: math::Angle GetUpperLimit(int _index) const;

    public: math::Angle GetLowerLimit(int _index) const;

    public: void SetPosition(double _position);

    public: bool HasJoint() const;

    private: math::Angle fakePosition;

    private: double fakeVelocity;

    private: double fakeTorque;

    private: math::Angle fakeUpperLimit;

    private: math::Angle fakeLowerLimit;

    private: std::string jointName;

    private: physics::JointPtr realJoint;

    private: bool hasJoint;
  };
}
#endif
