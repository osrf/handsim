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
  /// This class essentially contains a `gazebo::physics::JointPtr`,
  /// which either points to a real Gazebo Joint object, or is left NULL.
  /// When controller calls an equivalent Gazebo Joint API using this
  /// JointHelper class, it either passes them through to the Gazebo
  /// Joint class for valid Gazebo Joints, or get/set the local values
  /// for the "fake" joint.
  /// A pesudo-code example below:
  ///
  /// ~~~
  /// HaptixGazeboJointHelper realJoint;
  /// realJoint.SetJoint(realGazeboJointPtr);
  /// HaptixGazeboJointHelper noJoint;
  /// realJoint.GetPosition(0);  // effectively calls realJoint->GetPosition(0)
  /// noJoint.SetPosition(10);  // sets noJoint.fakeJoint to 10
  /// noJoint.GetPosition(0);  // returns noJoint.fakeJoint
  /// ~~~
  ///
  /// The reason for creating this class is that we've created artificial
  /// motor joints which do not exist in the Gazebo model for the arm
  /// (e.g. `indexm`, `middlem`, `ringm` and `pinkym`), but we want to
  /// manipulate these *fake joints*, in a transparent way similar to
  /// the real joints.
  class JointHelper
  {
    /// \brief Constructor
    public: JointHelper();

    /// \brief Destructor
    public: ~JointHelper();

    /// \brief Operator =
    /// \param[in] _joint a valid Gazebo Joint pointer
    public: JointHelper &operator=(physics::JointPtr _joint);

    /// \brief Assign a valid gazebo joint
    /// \param[in] _joint a valid Gazebo Joint pointer
    public: void SetJoint(physics::JointPtr _joint);

    /// \brief Set name of this joint
    /// \param[in] _name name of this joint
    public: void SetJointName(const std::string &_name);

    /// \brief Set angle of this joint
    /// \param[in] _index joint index to set angle
    /// \return Angle of the joint
    public: math::Angle GetAngle(int _index);

    /// \brief Get angular velocity of this joint
    /// \param[in] _index joint index to get angular velocity
    /// \return Joint velocity
    public: double GetVelocity(int _index);

    /// \brief Set force of this joint
    /// \param[in] _index joint index to set force
    /// \return True if the force was set correctly
    public: bool SetForce(int _index, double _force);

    /// \brief Get force of this joint
    /// \param[in] _index joint index to get force
    /// \return Force on a joint.
    public: double GetForce(int _index);

    /// \brief Get upper joint limit
    /// \param[in] _index joint index to get upper limit
    /// \return upper joint limit
    public: math::Angle GetUpperLimit(int _index) const;

    /// \brief Get lower joint limit
    /// \param[in] _index joint index to get lower limit
    /// \return lower joint limit
    public: math::Angle GetLowerLimit(int _index) const;

    /// \brief Set joint position
    /// \param[in] _index joint index to set position
    public: void SetPosition(double _position);

    /// \brief Set joint effort limit
    /// \param[in] _effort Effort limit for the axis.
    /// \param[in] _index Index of the axis to set.
    public: void SetEffortLimit(unsigned int _index, double _effort);

    /// \brief Returns true if joint has been assigned
    /// \return true if joint has been assigned
    public: bool HasJoint() const;

    private: math::Angle fakePosition;

    private: double fakeVelocity;

    private: double fakeTorque;

    private: math::Angle fakeUpperLimit;

    private: math::Angle fakeLowerLimit;

    private: math::Angle fakeEffortLimit;

    private: std::string jointName;

    private: physics::JointPtr realJoint;

    private: bool hasJoint;
  };
}
#endif
