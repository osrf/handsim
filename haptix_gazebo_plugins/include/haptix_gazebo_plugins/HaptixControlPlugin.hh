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

    /// \breif A pointer to the world
    private: physics::WorldPtr world;

    /// \breif A pointer to the parent model
    private: physics::ModelPtr model;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief For computing dt
    private: common::Time lastTime;

    /**********************************************/
    /*                                            */
    /*       basic polhemus interface stuff       */
    /*                                            */
    /**********************************************/
    /// \brief true if there a working polhemus connected to this computer?
    private: bool havePolhemus;
    /// \brief connection handle
    private: polhemus_conn_t *polhemusConn;
    /// \brief convenience utility function
    private: math::Pose convertPolhemusToPose(double x, double y, double z,
      double roll, double pitch, double yaw);
    /// \brief convenience utility function
    private: math::Pose convertPolhemusToPose(const polhemus_pose_t &_pose);
    /// \brief polhemus polling thread to detect sensor locations
    private: void UpdatePolhemus();
    /// \brief polhemus polling thread handle
    private: boost::thread polhemusThread;

    /**********************************************/
    /*                                            */
    /*       arm base control stuff               */
    /*                                            */
    /**********************************************/
    /// \brief: a pointer to the arm base "floating" joint for positioning
    /// the arm in space. This joint is provides implicit viscous damping
    /// of the arm base motion.
    private: physics::JointPtr baseJoint;
    /// \brief: base link of the arm.
    private: physics::LinkPtr baseLink;
    /// \brief: target position for the arm base link in world frame.
    private: math::Pose targetBaseLinkPose;
    /// \brief base link pose in world frame on startup.  This is where
    /// the user spawned the base link model.
    private: math::Pose initialBaseLinkPose;
    /// \brief PID for controlling arm base link position in world frame.
    private: common::PID posPid;
    /// \brief PID for controlling arm base link orientation in world frame.
    private: common::PID rotPid;
    /// \brief force for moving the arm base link to target location.
    private: Wrench wrench;
    /// \brief this actually does the PID force calculation and applies
    /// force to the arm base link.
    /// \param[in] _dt time step size.
    private: void UpdateBaseLink(double _dt);

    /**********************************************/
    /*                                            */
    /*   for polhemus based view point tracking   */
    /*                                            */
    /**********************************************/
    /// \brief gazebo gz transport node for commanding gazebo UserCamera
    private: gazebo::transport::NodePtr gazeboNode;
    /// \brief gazebo gz transport node publisher handle
    private: gazebo::transport::PublisherPtr polhemusJoyPub;
    /// \brief gazebo gz transport message
    private: gazebo::msgs::Pose joyMsg;
    /// \brief target UserCamera pose in world frame
    private: math::Pose targetCameraPose;
    /// \brief initial UserCamera pose in world frame, not used.
    private: math::Pose initialCameraPose;

    /**********************************************/
    /*                                            */
    /*   for subscribing to key events published  */
    /*   by gzclient window                       */
    /*                                            */
    /**********************************************/
    /// \brief gazebo key event subscriber handle
    private: gazebo::transport::SubscriberPtr keySub;
    /// \brief gazebo key event subscriber callback function
    private: void OnKey(ConstRequestPtr &_msg);
    /// \brief stores the key pressed from gazebo key event
    private: char keyPressed;

    /// \brief Subscriber to spacenav messages.
    private: gazebo::transport::SubscriberPtr joySub;

    /// \brief Callback for subscriber to spacenav messages.
    /// \param[in] _msg Joystick data.
    private: void OnJoy(ConstJoystickPtr &_msg);

    /// \brief Copy of latest Joystick message.
    private: msgs::Joystick latestJoystickMessage;

    /// \brief Flag to indicate when new Joystick message has been received.
    private: bool newJoystickMessage;

    /// \brief Mutex to protect access to newJoystickMessage
    private: boost::mutex joystickMessageMutex;

    // for tracking polhemus setup, where is the source in the world frame?
    private: physics::ModelPtr polhemusSourceModel;
    private: math::Pose sourceWorldPose;
    // transform from polhemus sensor orientation to base link frame
    private: math::Pose baseLinkToArmSensor;
    // transform from polhemus sensor orientation to camera frame
    private: math::Pose cameraToHeadSensor;

    // control the hand
    private: void GetRobotStateFromSim();

    /// \brief: Update joint PIDs in simulation on every tick
    /// \param[in] _dt time step to be passed into PID class for control update.
    private: void UpdateHandControl(double _dt);

    /// \brief: state and command messages
    private: haptix::comm::msgs::hxSensor robotState;
    private: haptix::comm::msgs::hxCommand robotCommand;
    private: class SimRobotCommand
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
    /// All joints to be controlled by this plugin.
    /// Joint id's must be consecutive.
    /// Example:
    ///   <joint id="0">joint_33</joint>
    ///   <joint id="1">joint_55</joint>
    private: std::map<unsigned int, std::string> jointNames;
    private: std::vector<physics::JointPtr> joints;

    /// \brief: class containing info on motors
    private: class MotorInfo
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

      /// \brief: index of joint controlled by this motor
      public: unsigned int index;

      /// \brief: index of coupled joints
      public: class GearBox
      {
        /// \brief: index of joint controlled by this gearbox
        public: unsigned int index;
        /// \brief: see example for motorInfos
        public: double offset;
        /// \brief: see example for motorInfos
        public: double multiplier;
      };
      /// \brief: joint coupling enforced at position/velocity command level.
      public: std::vector<GearBox> gearboxes;
    };
    /// \brief: user controllable joints via motor commands in hxCommand.
    /// <gearbox> joint coupling is only applied at position/velocity command
    /// level. <gear_ratio> is not used yet.
    ///
    /// For example, code below means:
    ///   joint_33 position = (joint_55 position - 0.1) * 1.5
    ///
    /// Example:
    ///   <motor id="3" name="motor_a">
    ///     <powered_motor_joint>joint_33</powered_motor_joint>
    ///     <gear_ratio>399.0</gear_ratio>
    ///     <gearbox>
    ///       <joint>joint_55</joint>
    ///       <offset>0.1</offset>
    ///       <multiplier>1.5</multiplier>
    ///     </gearbox>
    ///   </motor>
    private: std::map<unsigned int, MotorInfo> motorInfos;

    /// \brief: list of gazebo joints that corresponds to each motor
    private: std::vector<unsigned int> motors;

    /// \brief: contact sensor names
    /// Reads from plugin SDF, example:
    ///   <contactSensor id="0">contact_sensor_0</contactSensor>
    ///   <contactSensor id="1">contact_sensor_5</contactSensor>
    private: std::map<unsigned int, std::string> contactSensorNames;

    /// \brief: gazebo contact sensors
    /// create a list of contact sensors based on contactSensorNames
    private: std::vector<sensors::ContactSensorPtr> contactSensors;

    /// \brief: imu sensor names
    /// Reads from plugin SDF, example:
    ///   <imuSensor id="0">imu_sensor_9</imuSensor>
    ///   <imuSensor id="1">imu_sensor_5</imuSensor>
    private: std::map<unsigned int, std::string> imuSensorNames;

    /// \brief: gazebo imu sensors
    /// create a list of imu sensors based on imuSensorNames
    private: std::vector<sensors::ImuSensorPtr> imuSensors;

    /// \brief: internal PIDs for holding all actuated joints in gazebo
    /// One PID controller per joint specified by plugin's <joint> param.
    /// pid id's must match joint id's.
    /// Example:
    ///   <pid id="0"  p="10.0" i="0" d="0" cmd_max="10.0" cmd_min="-10.0"/>
    ///   <pid id="1"  p="10.0" i="0" d="0" cmd_max="10.0" cmd_min="-10.0"/>
    private: std::vector<common::PID> pids;

    /// \brief: ignition transport node for talking to haptix comm
    private: ignition::transport::Node ignNode;

    /// \brief: Provide device info through haptix_comm
    /// \param[in] _service service name
    /// \param[in] _req request data, not used here.
    /// \param[out] _rep respond data, returns info in haptix::comm::hxDevice.
    /// \param[out] _result returns true if request was successful
    private: void HaptixGetDeviceInfoCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxDevice &_req,
      haptix::comm::msgs::hxDevice &_rep, bool &_result);

    /// \brief: Simulation responder to team controller client nodes
    /// \param[in] _service service name
    /// \param[in] _req request data, contains robot control commands.
    /// \param[out] _rep respond data, returns robot states in hxSensor struct.
    /// \param[out] _result returns true if request was successful
    private: void HaptixUpdateCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxCommand &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result);

    /// \brief: initialize gazebo controllers
    private: void LoadHandControl();

    /// \brief Update arm position based on spacenav input.
    /// The spacenav input is interpreted as a desired velocity
    /// that is integrated over a specified timestep.
    /// \param[in] _dt Time step to integrate over.
    private: void UpdateSpacenav(double _dt);

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

