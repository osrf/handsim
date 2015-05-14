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
#ifndef _HANDSIM_HAPTIX_CONTROL_PLUGIN_HH_
#define _HANDSIM_HAPTIX_CONTROL_PLUGIN_HH_

#include <math.h>
#include <handsim/polhemus_driver.h>

#include <haptix/comm/haptix.h>

#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxRobot.pb.h>
#include <haptix/comm/msg/hxSensor.pb.h>
#include <haptix/comm/msg/hxGrasp.pb.h>
#include <ignition/transport.hh>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <string>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/KeyEvent.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>

#include <handsim/Optitrack.hh>
#include <handsim/MotorInfo.hh>
#include <handsim/WrenchHelper.hh>
#include <handsim/JointHelper.hh>

namespace gazebo
{
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
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Reset();

    /// \brief Gazebo loop: Update the controller on every simulation tick.
    private: void GazeboUpdateStates();

    /// \brief convenience utility function
    private: math::Pose convertPolhemusToPose(double x, double y, double z,
      double roll, double pitch, double yaw);

    /// \brief convenience utility function
    private: math::Pose convertPolhemusToPose(const polhemus_pose_t &_pose);

    /// \brief polhemus polling thread to detect sensor locations
    private: void UpdatePolhemus();

    /// \brief this actually does the PID force calculation and applies
    /// force to the arm base link.
    /// \param[in] _dt time step size.
    private: void UpdateBaseLink(double _dt);

    /// \brief callback for subscriber to the user camera pose publisher
    private: void OnUserCameraPose(ConstPosePtr &_msg);

    /// \brief callback for subscriber to the hydra publisher
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief Callback for subscriber to spacenav messages.
    /// \param[in] _msg Joystick data.
    private: void OnJoy(ConstJoystickPtr &_msg);

    /// \brief Callback for subscriber to pause polhemus
    /// \param[in] _msg pause state
    private: void OnPause(ConstIntPtr &_msg);

    /// \brief Update the state of the robot hand based the commanded states.
    private: void GetRobotStateFromSim();

    /// \brief Get hand control information from client.
    private: void GetHandControlFromClient();

    /// \brief: Update joint PIDs in simulation on every tick
    /// \param[in] _dt time step to be passed into PID class for control update.
    private: void UpdateHandControl(double _dt);

    /// \brief: gazebo contact sensors
    private: void OnContactSensorUpdate(int _i);

    /// \brief: publish HaptixControlPlugin status
    private: void PublishHaptixControlStatus();

    /// \brief: Provide robot info through haptix_comm
    /// \param[in] _service service name
    /// \param[in] _req request data, not used here.
    /// \param[out] _rep respond data, returns info in haptix::comm::hxRobot.
    /// \param[out] _result returns true if request was successful
    private: void HaptixGetRobotInfoCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxRobot &_req,
      haptix::comm::msgs::hxRobot &_rep, bool &_result);

    /// \brief: Simulation responder to team controller client nodes
    /// \param[in] _service service name
    /// \param[in] _req request data, contains robot control commands.
    /// \param[out] _rep respond data, returns robot states in hxSensor struct.
    /// \param[out] _result returns true if request was successful
    private: void HaptixUpdateCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxCommand &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result);

    /// \brief: Simulation responder to execute predefined grasps
    /// \param[in] _service Service name
    /// \param[in] _req Requested grasp
    /// \param[out] _rep Reply in the form of commanded joint angles
    /// \param[out] _result True if the command was successful
    private: void HaptixGraspCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxGrasp &_req,
      haptix::comm::msgs::hxCommand &_rep, bool &_result);

    /// \brief Simulation responder to sensor read command
    /// \param[in] _service Service name
    /// \param[in] _req Request sensor (unused)
    /// \param[out] _rep Reply sensor
    /// \param[out] _result True if command was successful
    private: void HaptixReadCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxSensor &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result);

    /// \brief: initialize gazebo controllers
    private: void LoadHandControl();

    /// \brief Update arm position based on spacenav input.
    /// The spacenav input is interpreted as a desired velocity
    /// that is integrated over a specified timestep.
    /// \param[in] _dt Time step to integrate over.
    private: void UpdateSpacenav(double _dt);

    // keyboard params and methods

    /// \brief Initialize parameters for keyboard teleop
    /// \return Whether or not keyboard teleop initialization was successful
    private: bool LoadKeyboard();

    /// \brief Update the state of the arm based on keyboard input.
    /// \param[in] _dt Timestep between updates
    private: void UpdateKeyboard(double _dt);

    /// \brief Callback to set the saved keyboard pose based on user input.
    /// \param[in] _topic Topic for message transport: /haptix/arm_pose_inc.
    /// \param[in] _pose The input pose.
    private: void SetKeyboardPose(const std::string &/*_topic*/,
                                  const msgs::Pose &_pose);

    /// \brief Callback to set the model pose directly. Also update controller.
    /// \param[in] _topic Topic for message transport: /haptix/arm_model_pose.
    /// \param[in] _pose The input model pose in world frame.
    private: void SetWorldPose(const std::string &/*_topic*/,
                               const msgs::Pose &_pose);

    /// \brief Callback on Optitrack head tracker update
    private: void OnUpdateOptitrackHead(ConstPosePtr &_pose);

    /// \brief Callback on Optitrack arm tracker update
    private: void OnUpdateOptitrackArm(ConstPosePtr &_pose);

    /// \brief Callback on Optitrack monitor tracker update
    private: void OnUpdateOptitrackMonitor(ConstPointCloudPtr &_pose);

    /// \brief Callback on message to toggle viewpoint rotations due to mocap
    /// \param[in] _msg Message sent by publisher
    private: void OnToggleViewpointRotations(ConstIntPtr &_msg);

    /// \breif A pointer to the world
    private: physics::WorldPtr world;

    /// \breif A pointer to the parent model
    private: physics::ModelPtr model;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the connection for the end of an update
    private: event::ConnectionPtr updateConnectionEnd;

    /// \brief For computing dt
    private: common::Time lastTime;

    /// \brief For user control code throttling
    private: common::Time lastSimTimeForControlThrottling;

    ////////////////////////////////////////////////
    //
    //       basic polhemus interface stuff
    //
    ////////////////////////////////////////////////

    /// \brief true if there a working polhemus connected to this computer?
    private: bool havePolhemus;

    /// \brief connection handle
    private: polhemus_conn_t *polhemusConn;

    /// \brief polhemus polling thread handle
    private: boost::thread polhemusThread;

    ////////////////////////////////////////////////
    //
    //       arm base control stuff
    //
    ////////////////////////////////////////////////

    /// \brief: a pointer to the arm base "floating" joint for positioning
    /// the arm in space. This joint is provides implicit viscous damping
    /// of the arm base motion.
    private: physics::JointPtr baseJoint;

    /// \brief: base link of the arm.
    private: physics::LinkPtr baseLink;

    /// \brief: target position for the arm base link in world frame.
    private: math::Pose targetBaseLinkPose;

    /// \brief: from spacenav target pose to the arm base link
    private: math::Pose baseLinktoSpacenavPose;

    /// \brief base link pose in world frame on startup.  This is where
    /// the user spawned the base link model.
    private: math::Pose initialBaseLinkPose;

    /// \brief PID for controlling arm base link position in world frame.
    private: common::PID posPid;

    /// \brief PID for controlling arm base link orientation in world frame.
    private: common::PID rotPid;

    /// \brief force for moving the arm base link to target location.
    private: WrenchHelper wrench;

    /// \brief publish HaptixControlPlugin status
    private: gazebo::transport::PublisherPtr haptixStatusPub;

    ////////////////////////////////////////////////
    //
    //   for polhemus based view point tracking
    //
    ////////////////////////////////////////////////

    /// \brief gazebo gz transport node for commanding gazebo UserCamera
    private: gazebo::transport::NodePtr gazeboNode;

    /// \brief gazebo gz transport node publisher handle
    private: gazebo::transport::PublisherPtr viewpointJoyPub;

    /// \brief gazebo gz transport message
    private: gazebo::msgs::Pose joyMsg;

    /// \brief initial UserCamera pose in world frame, not used.
    private: math::Pose initialCameraPose;

    /// \brief respond to successful pausing of polhemus
    private: gazebo::transport::PublisherPtr pausePub;

    ////////////////////////////////////////////////
    //
    //   get user camera pose
    //
    ////////////////////////////////////////////////

    /// \brief subscribe to user camera pose publisher
    private: gazebo::transport::SubscriberPtr userCameraPoseSub;

    /// \brief store camera pose
    private: math::Pose userCameraPose;

    /// \brief have we received at least one camera pose?
    private: bool userCameraPoseValid;

    /// \brief Mutex to protect access to userCameraPose
    private: boost::mutex userCameraPoseMessageMutex;

    /// \brief subscribe to hydra
    private: gazebo::transport::SubscriberPtr hydraSub;

    /// \brief store hydra pose
    private: math::Pose hydraPose;

    /// \brief Mutex to protect access to hydraPose
    private: boost::mutex hydraMessageMutex;

    /// \brief have a hydra?
    private: bool haveHydra;

    ////////////////////////////////////////////////
    //
    //   variables for pause management
    //
    ////////////////////////////////////////////////

    /// \brief true if motion tracking is paused
    private: bool pauseTracking;

    /// \brief got command to pause polhemus updates
    private: bool gotPauseRequest;

    /// \brief Subscriber to spacenav messages.
    private: gazebo::transport::SubscriberPtr joySub;

    /// \brief Subscriber to pause polhemus
    private: gazebo::transport::SubscriberPtr pauseSub;

    /// \brief Copy of latest Joystick message.
    private: msgs::Joystick latestJoystickMessage;

    /// \brief Flag to indicate when new Joystick message has been received.
    private: bool newJoystickMessage;

    /// \brief Mutex to protect access to newJoystickMessage
    private: boost::mutex joystickMessageMutex;

    /// \brief Model for tracking the polhemus source.
    private: physics::ModelPtr polhemusSourceModel;

    /// \brief Pose of the polhemus source in the world frame.
    private: math::Pose sourceWorldPose;

    /// \brief used to offset polhemus source for arm sensor during calibration
    private: math::Pose sourceWorldPoseArmOffset;

    /// \brief used to offset polhemus source for head sensor during calibration
    private: math::Pose sourceWorldPoseHeadOffset;

    /// \brief Transform from polhemus sensor orientation to base link frame.
    private: math::Pose baseLinkToArmSensor;

    /// \brief Transform from polhemus sensor orientation to camera frame
    private: math::Pose cameraToHeadSensor;

    /// \brief Transform from hydra sensor orientation to base link frame.
    private: math::Pose baseLinkToHydraSensor;

    /// \brief: state and command messages
    private: haptix::comm::msgs::hxSensor robotState;

    /// \brief: keep a local copy of hxCommand
    private: haptix::comm::msgs::hxCommand robotCommand;

    /// \brief: simulation commands for all the pid'd simulation joints
    private: class SimRobotCommand
    {
      public: double ref_pos;
      public: double ref_vel_max;
      public: double gain_pos;
      public: double gain_vel;
    };

    /// \brief: commanding all the joints in robot, and map
    /// robotCommand motor joints to a subset of the joints here.
    private: std::vector<SimRobotCommand> simRobotCommands;

    /// \brief: joint names matching those of gazebo model
    /// All joints to be controlled by this plugin.
    /// Joint id's must be consecutive.
    /// Example:
    ///   <joint id="0">joint_33</joint>
    ///   <joint id="1">joint_55</joint>
    private: std::map<unsigned int, std::string> jointNames;

    private: std::vector<JointHelper*> haptixJoints;

    /// \brief: user controllable joints via motor commands in hxCommand.
    /// <gearbox> joint coupling is only applied at position/velocity command
    /// level. <gear_ratio> is not used yet.
    ///
    /// Example transmission:
    ///   <motor id="3" name="motor_a">
    ///     <powered_motor_joint>joint_33</powered_motor_joint>
    ///     <gear_ratio>399.0</gear_ratio>
    ///     <!-- per JHU requirements, transmission is applied in 2 stages,
    ///          for powered motor joint angle less than offset,
    ///          multiplier1 is used. For powered motor joint angle
    ///          greater than minimum offset, multiplier2 is used.
    ///
    ///          For example:
    ///
    ///            minOffset angle = min(all gearboxes offset angles) = 0.15
    ///            if (joint_33 angle < minOffset angle)
    ///              joint_55 angle = 100.0 * joint_33 angle
    ///              joint_66 angle = 0.0 * joint_33 angle
    ///            else
    ///              joint_55 angle = 1.5 * (joint_33 angle - minOffset angle)
    ///                       + 100.0 * minOffset angle
    ///              joint_66 angle = 10.5 * (joint_33 angle - minOffset angle)
    ///                       + 0.0 * minOffset angle
    ///     -->
    ///     <gearbox>
    ///       <joint>joint_55</joint>
    ///       <offset>0.15</offset>
    ///       <multiplier1>100.0</multiplier1>
    ///       <multiplier2>1.5</multiplier2>
    ///     </gearbox>
    ///     <gearbox>
    ///       <joint>joint_66</joint>
    ///       <offset>0.25</offset>
    ///       <multiplier1>0.0</multiplier1>
    ///       <multiplier2>10.5</multiplier2>
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

    /// \brief: data structure for storing contact sensor infos
    private: class ContactSensorInfo
    {
      public: sensors::SensorPtr sensor;
      public: event::ConnectionPtr connection;
      // aggregated forces and torques from contact
      public: math::Vector3 contactForce;
      public: math::Vector3 contactTorque;
    };

    /// \brief: create a list of contact sensors based on contactSensorNames
    private: std::vector<ContactSensorInfo> contactSensorInfos;

    /// \brief: imu sensor names
    /// Reads from plugin SDF, example:
    ///   <imuSensor id="0">imu_sensor_9</imuSensor>
    ///   <imuSensor id="1">imu_sensor_5</imuSensor>
    private: std::map<unsigned int, std::string> imuSensorNames;

    /// \brief: gazebo imu sensors
    /// create a list of imu sensors based on imuSensorNames
    private: std::vector<sensors::ImuSensorPtr> imuSensors;

    /// \brief: list of predefined grasps
    /// Give each one a name and the desired positions for the motors. E.g.:
    ///    <grasp name="MyGrasp">0 0 0 0.7679 0 0 1.3963 0 0 0 0
    ///                          0.8727 0 0.5236 -0.349</grasp>
    private: std::map<std::string, std::vector<float> > grasps;

    /// \brief: current desired grasp pose
    /// If graspMode is true, then these are the desired positions of the finger
    /// motors.
    private: std::vector<float> graspPositions;

    /// \brief: are we in predefined grasp mode?
    /// If true, then only values drawn from predefined grasps are commanded;
    /// else direct finger motor control is performed.  We start not in
    /// predefined mode; we enter this mode on receipt of a non-empty request to
    /// "haptix/gazebo/grasp" service; we exit this mode on receipt of an empty
    /// request to the same service.
    private: bool graspMode;

    /// \brief: internal PIDs for holding all actuated joints in gazebo
    /// One PID controller per joint specified by plugin's <joint> param.
    /// pid id's must match joint id's.
    /// Example:
    ///   <pid id="0"  p="10.0" i="0" d="0" cmd_max="10.0" cmd_min="-10.0"/>
    ///   <pid id="1"  p="10.0" i="0" d="0" cmd_max="10.0" cmd_min="-10.0"/>
    private: std::vector<common::PID> pids;

    /// \brief: ignition transport node for talking to haptix comm
    private: ignition::transport::Node ignNode;

    /// \brief True if keyboard teleop is enabled, false otherwise.
    private: bool haveKeyboard;

    /// \brief The pose commanded by user input.
    math::Pose keyboardPose;

    /// \brief True if keyboardPose is old data that has already been consumed
    bool staleKeyboardPose;

    private: boost::mutex updateMutex;
    private: boost::mutex baseLinkMutex;
    private: boost::mutex pauseMutex;
    private: sdf::ElementPtr sdf;

    /// \brief Optitrack packet receiver, used to start an optitrack thread
    private: haptix::tracking::Optitrack optitrack;

    /// \brief OptiTrack receiving thread
    private: std::shared_ptr<std::thread> optitrackThread;

    /// \brief Subscriber to Optitrack head tracker updates
    private: gazebo::transport::SubscriberPtr optitrackHeadSub;

    /// \brief Subscriber to Optitrack arm tracker updates
    private: gazebo::transport::SubscriberPtr optitrackArmSub;

    /// \brief Subscriber to Optitrack monitor tracker updates
    private: gazebo::transport::SubscriberPtr optitrackMonitorSub;

    /// \brief hardcoded offset between controlled position of the link and
    /// arm sensor
    private: gazebo::math::Pose elbowMarker;
    private: gazebo::math::Pose elbowMarkerCorrected;

    /// \brief Transform from camera frame to Optitrack head marker.
    private: math::Pose headMarker;

    /// \brief Optitrack calibration offset for the head
    private: gazebo::math::Pose optitrackHeadOffset;

    /// \brief Optitrack calibration offset for the arm
    private: gazebo::math::Pose optitrackArmOffset;

    /// \brief Pose of the optitrack monitor tracker in the Optitrack frame
    private: gazebo::math::Pose cameraMonitor;

    /// \brief Pose of the fake "screen" frame in the monitor tracker frame
    private: gazebo::math::Pose monitorScreen;

    /// \brief Pose of the fake screen in the world frame
    private: gazebo::math::Pose worldScreen;

    /// \brief Low-pass filter for head position (reduces jitter)
    private: gazebo::math::OnePoleVector3 headPosFilter;

    /// \brief Low-pass filter for head orientation (reduces jitter)
    private: gazebo::math::OnePoleQuaternion headOriFilter;

    /// \brief Receives messages to toggle viewpoint rotations.
    private: gazebo::transport::SubscriberPtr viewpointRotationsSub;

    /// \brief Mutex to lock optitrack monitor data
    private: std::mutex optitrackMonitorMutex;

    /// \brief True if motion capture rotations the head, false otherwise.
    private: std::atomic<bool> viewpointRotationsEnabled;

    /// \brief True if optitrackArmOffset has been initialized
    private: bool armOffsetInitialized;

    /// \brief True if optitrackHeadOffset has been initialized
    private: bool headOffsetInitialized;

    /// \brief Rate at which this plugin should update
    private: double updateRate;
  };

/// \}
}
#endif
