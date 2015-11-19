/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _HANDSIM_HAPTIX_MOTORINFO_HH_
#define _HANDSIM_HAPTIX_MOTORINFO_HH_

#include <vector>
#include <string>


namespace gazebo
{
  /// \brief: class containing info on motors
  class MotorInfo
  {
    /// \brief: motor name
    public: std::string name;

    /// \brief: joint name associated with each motor
    public: std::string jointName;

    /// \brief: max continuous motor torque
    public: double motorTorque;

    /// \brief: gear_ratio = motor_angle / joint_angle
    /// assuming the _hxCommand::ref_pos and _hxCommand::ref_vel are
    /// motor position and motor velocities, use gear_ratio to
    /// compute simulation joint torques.
    public: double gearRatio;

    /// \brief: joint_offset
    /// assuming the _hxCommand::ref_pos is motor position,
    /// use joint_offset and gear_ratio to
    /// compute motor position based on simulation joint position.
    /// motor_pos = (motor_offset + joint_pos) * gear_ratio
    public: double encoderOffset;

    /// \brief: index of joint controlled by this motor
    public: int index;

    /// \brief: enable clutch if true
    public: bool clutch;

    /// \brief: This factor is multiplied to effort before sending effort command
    ///         to an actuator joint. Its value should be:
    ///
    ///           1 - sum(effortDifferentials.multiplier)
    ///
    ///         i.e. The sum of all effortDifferential multiplier and this multiplier
    ///         should be 1.
    public: double effortDifferentialMultiplier;

    /// \brief: share power between joints
    ///   Share multiplier * actuator effort to this joint.
    ///   This joint is commanded by an effort based controller,
    ///     not position controlled.
    ///   Total torque is distributed between:
    ///     - <powered_motor_joint> with weight of 1.0.
    ///     - all of <effort_differential> joints with weight specified by
    ///       <multiplier>'s.
    ///   Total torque on motor joint is:
    ///     1 / (1 + SUM( effort_differential/multiplier ))
    ///   Total torque on any differential joint is:
    ///     effort_differential/multiplier /
    ///     (1 + SUM( effort_differential/multiplier ))
    public: class EffortDifferential
    {
      /// \brief: index of joint controlled by this differential
      public: int index;

      /// \brief: share effort with motor joint and other joints
      /// on EffortDifferentials
      public: double multiplier;
    };

    /// \brief: joint coupling enforced at position/velocity command level.
    public: std::vector<EffortDifferential> effortDifferentials;

    /// \brief: index of coupled joints
    public: class GearBox
    {
      /// \brief: index of joint controlled by this gearbox
      public: int index;

      /// \brief: index of joint used as reference for offset
      public: int referenceIndex;

      /// \brief: see example for motorInfos
      public: double offset;

      /// \brief: see example for motorInfos
      public: double multiplier1;

      public: double multiplier2;
    };

    /// \brief: joint coupling enforced at position/velocity command level.
    public: std::vector<GearBox> gearboxes;
  };
}
#endif
