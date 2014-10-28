/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_HAPTIX_CONTROL_PLUGIN_HH
#define GAZEBO_HAPTIX_CONTROL_PLUGIN_HH

#include <string>
#include <map>
#include <vector>
#include <math.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>

#include "polhemus_driver/polhemus_driver.h"

#include <spnav.h>

// #include <haptix/comm/Comm.h>
#include <haptix/comm/haptix.h>
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxDevice.pb.h>
#include <haptix/comm/msg/hxSensor.pb.h>
#include <ignition/transport.hh>

namespace gazebo
{
  class Wrench
  {
    /// \brief Operator =
    /// \param[in] _wrench wrench to set from.
    /// \return *this
    public: Wrench &operator =(const Wrench &_wrench)
            {
              this->force = _wrench.force;
              this->torque = _wrench.torque;
              return *this;
            }

    /// \brief Operator +
    /// \param[in] _wrench wrench to add
    /// \return *this
    public: inline Wrench &operator +(const Wrench &_wrench)
            {
              this->force += _wrench.force;
              this->torque += _wrench.torque;
              return *this;
            }

    /// \brief Operator -
    /// \param[in] _wrench wrench to subtract
    /// \return *this
    public: inline Wrench &operator -(const Wrench &_wrench)
            {
              this->force -= _wrench.force;
              this->torque -= _wrench.torque;
              return *this;
            }

    /// \brief linear forces
    public: math::Vector3 force;

    /// \brief angular torques
    public: math::Vector3 torque;

    /// \brief reference link frame
    public: physics::LinkPtr referenceFrame;
  };

  /// \defgroup haptix_control_plugins HAPTIX Control Plugins
  /// \addtogroup haptix_control_plugins
  /// \{
  class HaptixControlPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: HaptixControlPlugin();

    /// \brief Destructor
    public: virtual ~HaptixControlPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Gazebo loop: Update the controller on every simulation tick.
    private: void GazeboUpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: common::Time lastTime;

    // basic polhemus interfaces
    private: bool havePolhemus;
    private: polhemus_conn_t *polhemusConn;
    private: math::Pose convertPolhemusToPose(double x, double y, double z,
      double roll, double pitch, double yaw);
    private: math::Pose convertPolhemusToPose(const polhemus_pose_t &_pose);
    private: void UpdatePolhemus();
    private: boost::thread polhemusThread;

    // for polhemus arm base link pose control
    private: physics::JointPtr baseJoint;
    private: physics::LinkPtr baseLink;
    private: math::Pose targetBaseLinkPose;
    /// \brief base link pose in world frame on startup.  This is where
    /// the user spawned the base link model.
    private: math::Pose initialBaseLinkPose;
    // used to PID base link pose
    private: common::PID posPid;
    private: common::PID rotPid;
    private: Wrench wrench;
    private: void UpdateBaseLink(double _dt);

    // for polhemus view point tracking
    private: gazebo::transport::NodePtr gazebonode;
    private: gazebo::transport::PublisherPtr polhemusJoyPub;
    private: gazebo::msgs::Pose joyMsg;
    private: math::Pose targetCameraPose;
    private: math::Pose initialCameraPose;

    // subscribe to key events from gazebo qt window
    private: gazebo::transport::SubscriberPtr keySub;
    private: void OnKey(ConstRequestPtr &_msg);
    private: char keyPressed;

    // for tracking polhemus setup, where is the source in the world frame?
    private: physics::LinkPtr polhemusSourceLink;
    private: math::Pose sourceWorldPose;
    // transform from polhemus sensor orientation to base link frame
    private: math::Pose baseLinkToArmSensor;
    // transform from polhemus sensor orientation to camera frame
    private: math::Pose cameraToHeadSensor;

    private: void GetRobotStateFromSim();

    /// \brief: Update joint PIDs in simulation on every tick
    private: void UpdateHandControl(double _dt);

    /// \brief: state and command messages
    private: haptix::comm::msgs::hxSensor robotState;
    private: haptix::comm::msgs::hxCommand robotCommand;
    public: class SimRobotCommand
    {
      public: double ref_pos;
      public: double ref_vel;
      public: double gain_pos;
      public: double gain_vel;
    };
    /// \brief: commanding all the joints in robot, and map
    /// robotCommand motor joints to a subset of the joints here.
    private: std::vector<SimRobotCommand> simRobotCommand;

    /// \brief: joint names matching those of gazebo model
    private: std::map<unsigned int, std::string> jointNames;
    private: std::vector<physics::JointPtr> joints;

    /// \brief: class containing info on motors
    public: class MotorInfo
    {
      /// \brief: motor name
      public: std::string name;
      /// \brief: joint name associated with each motor
      public: std::string jointName;

      /// \brief: gear_ratio = motor_angle / joint_angle
      /// assuming the _hxCommand::ref_pos and _hxCommand::ref_vel are
      /// motor position and motor velocities, use gear_ratio to
      /// compute simulation joint torques.
      public: double gearRatio;
    };
    private: std::map<unsigned int, MotorInfo> motorInfos;

    /// \brief: list of gazebo joints that corresponds to each motor
    private: std::vector<unsigned int> motors;

    /// \brief: contact sensor names
    private: std::map<unsigned int, std::string> contactSensorNames;

    /// \brief: gazebo contact sensors
    private: std::vector<sensors::ContactSensorPtr> contactSensors;

    /// \brief: imu sensor names
    private: std::map<unsigned int, std::string> imuSensorNames;

    /// \brief: gazebo imu sensors
    private: std::vector<sensors::ImuSensorPtr> imuSensors;

    /// \brief: internal PIDs for holding all actuated joints in gazebo
    private: std::vector<common::PID> pids;

    /// \brief: ignition transport node for talking to haptix comm
    private: ignition::transport::Node ignNode;

    /// \brief: Provide device info through haptix_comm
    private: void HaptixGetDeviceInfoCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxDevice &_req,
      haptix::comm::msgs::hxDevice &_rep, bool &_result);

    /// \brief: Simulation responder to team controller client nodes
    private: void HaptixUpdateCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxCommand &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result);

    /// \brief: initialize gazebo controllers
    private: void LoadHandControl();

    // spacenav params
    private: bool LoadSpacenav();
    private: void UpdateSpacenav(double _dt);
    private: bool haveSpacenav;
    private: int static_count_threshold;
    private: bool zero_when_static;
    private: int static_trans_deadband;
    private: int static_rot_deadband;
    private: spnav_event sev;
    private: int no_motion_count;
    private: bool motion_stale;
    private: bool joy_stale;
    private: bool queue_empty;
    private: math::Vector3 spnPosOffset;
    private: math::Vector3 spnRotOffset;
    class SpnState
    {
      public: std::vector<int> buttons;
      public: std::vector<double> axes;
    };
    private: SpnState spnState;

    // keyboard params and methods
    private: bool LoadKeyboard();
    private: void UpdateKeyboard(double _dt);
    private: bool haveKeyboard;

    private: boost::mutex updateMutex;
    private: boost::mutex baseLinkMutex;
    private: sdf::ElementPtr sdf;
  };

/// \}
}
#endif

