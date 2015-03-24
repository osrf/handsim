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

#include "handsim/JointHelper.hh"

using namespace gazebo;

/////////////////////////////////////////////////
JointHelper::JointHelper()
{
  this->fakePosition = math::Angle(0.0);
  this->fakeVelocity = 0.0;
  this->fakeTorque = 0.0;
  this->fakeLowerLimit = -1e16;
  this->fakeUpperLimit = 1e16;
  this->fakeEffortLimit = 0;
  this->realJoint = NULL;
  this->hasJoint = false;
}

/////////////////////////////////////////////////
JointHelper::~JointHelper()
{
}

/////////////////////////////////////////////////
JointHelper &JointHelper::operator=(physics::JointPtr _joint)
{
  this->realJoint = _joint;
  this->hasJoint = true;
  return *this;
}

/////////////////////////////////////////////////
void JointHelper::SetJoint(physics::JointPtr _joint)
{
  this->realJoint = _joint;
  this->hasJoint = true;
}

/////////////////////////////////////////////////
std::string JointHelper::GetName()
{
  if (this->hasJoint)
    return this->realJoint->GetName();
  else
    return this->jointName;
}

/////////////////////////////////////////////////
math::Angle JointHelper::GetAngle(int _index)
{
  if (this->hasJoint)
  {
    return this->realJoint->GetAngle(_index);
  }
  else
  {
    return this->fakePosition;
  }
}

/////////////////////////////////////////////////
double JointHelper::GetVelocity(int _index)
{
  if (this->hasJoint)
    return this->realJoint->GetVelocity(_index);
  else
    return this->fakeVelocity;
}

/////////////////////////////////////////////////
bool JointHelper::SetForce(int _index, double _force)
{
  if (this->hasJoint)
  {
    this->realJoint->SetForce(_index, _force);
    return true;
  }
  else
  {
    if (_index == 0)
    {
      this->fakeTorque = _force;
    }
    else
    {
      // we only support _index == 0
      gzerr << "only joint index == 0 is supported.\n";
    }
    return false;
  }
}

/////////////////////////////////////////////////
double JointHelper::GetForce(int _index)
{
  if (this->hasJoint)
    return this->realJoint->GetForce(_index);
  else
    return this->fakeTorque;
}

/////////////////////////////////////////////////
math::Angle JointHelper::GetUpperLimit(int _index) const
{
  if (this->hasJoint)
    return this->realJoint->GetUpperLimit(_index);
  else
    return this->fakeUpperLimit;
}

/////////////////////////////////////////////////
math::Angle JointHelper::GetLowerLimit(int _index) const
{
  if (this->hasJoint)
    return this->realJoint->GetLowerLimit(_index);
  else
    return this->fakeLowerLimit;
}

/////////////////////////////////////////////////
void JointHelper::SetPosition(double _position)
{
  this->fakePosition = _position;
}

/////////////////////////////////////////////////
void JointHelper::SetJointName(const std::string &_name)
{
  this->jointName = _name;
}

/////////////////////////////////////////////////
void JointHelper::SetEffortLimit(unsigned int _index, double _effort)
{
  if (this->hasJoint)
    this->realJoint->SetEffortLimit(_index, _effort);
  else
    this->fakeEffortLimit = _effort;
}

/////////////////////////////////////////////////
bool JointHelper::HasJoint() const
{
  return this->hasJoint;
}
