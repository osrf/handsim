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

#include <gazebo/gui/KeyEventHandler.hh>

#include "handsim/HaptixControlPlugin.hh"

namespace gazebo
{
/////////////////////////////////////////////////
// Constructor
HaptixControlPlugin::HaptixControlPlugin()
{
  this->pauseTracking = true;
  this->gotPauseRequest = false;
  this->havePolhemus = false;
  this->polhemusConn = NULL;
  this->haveHydra = false;
  this->graspMode = false;
  this->staleKeyboardPose = false;
  this->newJoystickMessage = false;
  this->userCameraPoseValid = false;
  this->haveKeyboard = false;
  this->armOffsetInitialized = false;
  this->headOffsetInitialized = false;
  this->updateRate = 50.0;
  this->viewpointRotationsEnabled = false;

  // Advertise haptix services.
  this->ignNode.Advertise("/haptix/gazebo/GetRobotInfo",
    &HaptixControlPlugin::HaptixGetRobotInfoCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/Update",
    &HaptixControlPlugin::HaptixUpdateCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/Grasp",
    &HaptixControlPlugin::HaptixGraspCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/Read",
    &HaptixControlPlugin::HaptixReadCallback, this);
}

/////////////////////////////////////////////////
// Destructor
HaptixControlPlugin::~HaptixControlPlugin()
{
  this->polhemusThread.join();
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  event::Events::DisconnectWorldUpdateEnd(this->updateConnectionEnd);
}

/////////////////////////////////////////////////
// Load the controller
void HaptixControlPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;
  this->world->EnablePhysicsEngine(true);

  // start a transport node for polhemus head pose view point control
  this->gazeboNode =
    gazebo::transport::NodePtr(new gazebo::transport::Node());
  fprintf(stderr, "world name: [%s]\n", this->world->GetName().c_str());
  this->gazeboNode->Init(this->world->GetName());
  this->viewpointJoyPub =
    this->gazeboNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

  this->pausePub =
    this->gazeboNode->Advertise<gazebo::msgs::Int>("~/motion_tracking/pause_response");

  this->joySub =
    this->gazeboNode->Subscribe("~/user_camera/joy_twist",
      &HaptixControlPlugin::OnJoy, this);

  this->pauseSub =
    this->gazeboNode->Subscribe("~/motion_tracking/pause_request",
      &HaptixControlPlugin::OnPause, this);

  this->userCameraPoseValid = false;
  this->userCameraPoseSub =
    this->gazeboNode->Subscribe("~/user_camera/pose",
      &HaptixControlPlugin::OnUserCameraPose, this);

  this->hydraSub =
    this->gazeboNode->Subscribe("~/hydra",
      &HaptixControlPlugin::OnHydra, this);

  // Set the update rate
  if (this->sdf->HasElement("update_rate"))
    this->updateRate = this->sdf->Get<double>("update_rate");

  this->baseJoint =
    this->model->GetJoint(this->sdf->Get<std::string>("base_joint"));
  if (!this->baseJoint)
  {
    gzerr << "<base_joint>" << this->sdf->Get<std::string>("base_joint")
          << "<base_joint> does not exist\n";
    return;
  }

  this->baseLink =
    this->model->GetLink(this->sdf->Get<std::string>("base_link"));
  if (!this->baseLink)
  {
    gzerr << "<base_link>" << this->sdf->Get<std::string>("base_link")
          << "<base_link> does not exist\n";
    return;
  }
  // this is where the user spawned the base link of the arm
  this->initialBaseLinkPose = this->baseLink->GetWorldPose();
  this->targetBaseLinkPose = this->initialBaseLinkPose;
  // param for spacenav control, this is the point in arm base link
  // frame for which we want to control with the spacenav
  this->baseLinktoSpacenavPose = math::Pose(0, -0.4, 0, 0, 0, 0);
  if (_sdf->HasElement("spacenav_control_point_offset"))
  {
    this->baseLinktoSpacenavPose =
      _sdf->Get<math::Pose>("spacenav_control_point_offset");
  }

  // get polhemus_source model location
  // for tracking polhemus setup, where is the source in the world frame
  this->polhemusSourceModel = this->world->GetModel("polhemus_source");

  if (!this->polhemusSourceModel)
  {
    /// \TODO: make this a sdf param for the plugin?
    gzwarn << "no polhemus_source model detected using predefine location.\n";
    this->sourceWorldPose = math::Pose(-0.5, 0, 1.3, 3.14159, 0, -1.57159);
  }
  else
  {
    this->sourceWorldPose = this->polhemusSourceModel->GetWorldPose();
  }
  gzdbg << "Polhemus Source Pose [" << this->sourceWorldPose << "]\n";
  this->sourceWorldPoseArmOffset = math::Pose();
  this->sourceWorldPoseHeadOffset = math::Pose();
  // transform from polhemus sensor orientation to base link frame
  // -0.3 meters towards wrist from elbow
  // 90 degrees yaw to the left
  this->baseLinkToArmSensor = math::Pose(0, -0.3, 0, 0, 0, -0.5*M_PI);
  // transform from polhemus sensor orientation to camera frame
  // 10cm to the right of the sensor is roughly where the eyes are
  // -0.3 rad pitch up: sensor is usually tilted upwards when worn on head
  this->cameraToHeadSensor = math::Pose(0, 0.10, 0, 0.0, -0.3, 0.0);

  // transformation from camera to marker in camera frame
  this->cameraToOptitrackHeadMarker = math::Pose(-0.02, -0.025, 0.125,
                                           -M_PI/2, -M_PI/2, 0);

  this->viewpointRotationsSub = this->gazeboNode->Subscribe(
      "~/motion_tracking/viewpoint_rotations",
          &HaptixControlPlugin::OnToggleViewpointRotations, this);

  // hydra sensor offset
  this->baseLinkToHydraSensor = math::Pose(0, -0.3, 0, 0, 1.0*M_PI, -0.5*M_PI);

  // for controller time control
  this->lastTime = this->world->GetSimTime();

  // For user control code update rate throttling
  this->lastSimTimeForControlThrottling = this->world->GetSimTime();

  // initialize PID's
  double baseJointImplicitDamping = 100.0;
  if (this->sdf->HasElement("base_pid_pos"))
  {
    sdf::ElementPtr basePidPos = this->sdf->GetElement("base_pid_pos");
    double pVal, iVal, dVal, cmdMaxVal, cmdMinVal;
    basePidPos->GetAttribute("p")->Get(pVal);
    basePidPos->GetAttribute("i")->Get(iVal);
    basePidPos->GetAttribute("d")->Get(dVal);
    basePidPos->GetAttribute("cmd_max")->Get(cmdMaxVal);
    basePidPos->GetAttribute("cmd_min")->Get(cmdMinVal);
    this->posPid.Init(pVal, iVal, 0, 0, 0, cmdMaxVal, cmdMinVal);
    baseJointImplicitDamping = dVal;
  }
  else
  {
    gzwarn << "no <base_pid_pos> block, using defaults.\n";
    this->posPid.Init(10000, 0, 0, 0, 0, 10000, -10000);
  }

  if (this->sdf->HasElement("base_pid_rot"))
  {
    sdf::ElementPtr basePidRot = this->sdf->GetElement("base_pid_rot");
    double pVal, iVal, dVal, cmdMaxVal, cmdMinVal;
    basePidRot->GetAttribute("p")->Get(pVal);
    basePidRot->GetAttribute("i")->Get(iVal);
    basePidRot->GetAttribute("d")->Get(dVal);
    basePidRot->GetAttribute("cmd_max")->Get(cmdMaxVal);
    basePidRot->GetAttribute("cmd_min")->Get(cmdMinVal);
    this->rotPid.Init(pVal, iVal, 0, 0, 0, cmdMaxVal, cmdMinVal);
    baseJointImplicitDamping = std::max(dVal, baseJointImplicitDamping);
  }
  else
  {
    gzwarn << "no <base_pid_rot> block, using defaults.\n";
    this->rotPid.Init(10000, 0, 0, 0, 0, 10000, -10000);
  }

  // d-gain is enforced implicitly
  this->baseJoint->SetParam("erp", 0, 0.0);
  const double dampTol = 1.0e-6;
  if (baseJointImplicitDamping < dampTol)
  {
    gzwarn << "truncating arm base joint damping at " << dampTol << ".\n";
    baseJointImplicitDamping = dampTol;
  }
  this->baseJoint->SetParam("cfm", 0, 1.0/baseJointImplicitDamping);
  // same implicit damping for revolute joint stops
  this->baseJoint->SetParam("stop_erp", 0, 0.0);
  this->baseJoint->SetParam("stop_cfm", 0, 1.0/baseJointImplicitDamping);

  this->optitrackArm = gazebo::math::Pose::Zero;
  this->monitorOptitrackFrame = gazebo::math::Pose::Zero;

  this->optitrackWorldArmRot = gazebo::math::Pose(
                               gazebo::math::Vector3(0, 0, 0),
                               gazebo::math::Quaternion(M_PI, -M_PI/2, 0));

  this->optitrackArmOffset = gazebo::math::Pose::Zero;
  this->gazeboToOptitrack = gazebo::math::Pose::Zero;
  this->armOffsetInitialized = false;
  this->headOffsetInitialized = false;

  this->headPosFilter.SetFc(0.002, 0.3);
  this->headOriFilter.SetFc(0.002, 0.3);

  // Subscribe to Optitrack update topics: head, arm and origin
  this->optitrackHeadSub = this->gazeboNode->Subscribe
              ("~/optitrack/" + haptix::tracking::Optitrack::headTrackerName,
              &HaptixControlPlugin::OnUpdateOptitrackHead, this);
  this->optitrackArmSub = this->gazeboNode->Subscribe
              ("~/optitrack/" + haptix::tracking::Optitrack::armTrackerName,
              &HaptixControlPlugin::OnUpdateOptitrackArm, this);
  this->optitrackMonitorSub = this->gazeboNode->Subscribe
              ("~/optitrack/" + haptix::tracking::Optitrack::originTrackerName,
              &HaptixControlPlugin::OnUpdateOptitrackMonitor, this);

  this->optitrack.SetWorld(this->world->GetName());

  // Start receiving Optitrack tracking updates.
  this->optitrackThread = std::make_shared<std::thread>(std::thread(&haptix::tracking::Optitrack::StartReception, this->optitrack));
  // initialize polhemus
  this->havePolhemus = false;
  if (!(this->polhemusConn = polhemus_connect_usb(LIBERTY_HS_VENDOR_ID,
                                                  LIBERTY_HS_PRODUCT_ID,
                                                  LIBERTY_HS_WRITE_ENDPOINT,
                                                  LIBERTY_HS_READ_ENDPOINT)))
  {
    fprintf(stderr, "Failed to connect to Polhemus\n");
  }
  else
  {
    if (polhemus_init_comm(this->polhemusConn, 10))
    {
      fprintf(stderr, "Failed to initialize comms with Polhemus\n");
    }
    else
    {
      this->havePolhemus = true;
      msgs::GzString msg;
      msg.set_data("have_polhemus");
      this->gazeboNode->Publish<msgs::GzString>("~/arrange_polhemus", msg);
    }
  }

  // spin up a separate thread to get polhemus sensor data
  // update target pose if using polhemus
  if (this->havePolhemus)
    this->polhemusThread = boost::thread(
      boost::bind(&HaptixControlPlugin::UpdatePolhemus, this));
  else
    gzwarn << "No usable polhemus setup detected.\n";

  this->haveKeyboard = false;

  this->haveKeyboard = this->LoadKeyboard();

  this->LoadHandControl();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HaptixControlPlugin::GazeboUpdateStates, this));

  this->PublishHaptixControlStatus();
}

/////////////////////////////////////////////////
// Open spacenav
void HaptixControlPlugin::LoadHandControl()
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  int id;

  // Get joint names and insert id/name pair into map
  sdf::ElementPtr jointName = this->sdf->GetElement("joint");
  while (jointName)
  {
    jointName->GetAttribute("id")->Get(id);
    this->jointNames[id] = jointName->Get<std::string>();
    // get next sdf
    // gzdbg << "getting joint name [" << this->jointNames[id] << "]\n";
    jointName = jointName->GetNextElement("joint");
  }

  // Set initial haptixJoints size to match jointNames
  // this is incremented with fake joints from motor specifications
  this->haptixJoints.resize(this->jointNames.size());
  // Get gazebo joint pointers to joints
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
  {
    /// \TODO: this assumes id's for joints are consecutive, starting with 0
    physics::JointPtr joint = this->model->GetJoint(this->jointNames[i]);
    if (joint)
    {
      // gzdbg << "setting gazebo joint [" << joint->GetName() << "]\n";
      this->haptixJoints[i] = new JointHelper();
      this->haptixJoints[i]->SetJoint(joint);
      this->haptixJoints[i]->SetJointName(this->jointNames[i]);
    }
    else
      gzerr << "jointName [" << this->jointNames[i] << "] not found.\n";
  }

  // Get motor names and insert id/name pair into MotorInfo map
  sdf::ElementPtr motorSDF = this->sdf->GetElement("motor");
  while (motorSDF)
  {
    std::string motorName;
    motorSDF->GetAttribute("id")->Get(id);
    motorSDF->GetAttribute("name")->Get(motorName);
    // gzdbg << "getting motor [" << id << "]: [" << motorName << "]\n";
    this->motorInfos[id].name = motorName;

    // get joint name associated with this motor
    sdf::ElementPtr poweredMotorJointSDF =
      motorSDF->GetElement("powered_motor_joint");
    this->motorInfos[id].jointName = poweredMotorJointSDF->Get<std::string>();
    // gzdbg << "  joint [" << this->motorInfos[id].jointName << "]\n";

    // get gear ratio associated with this motor
    sdf::ElementPtr gearRatioSDF =
      motorSDF->GetElement("gear_ratio");
    this->motorInfos[id].gearRatio = gearRatioSDF->Get<double>();
    // gzdbg << "  gear [" << this->motorInfos[id].gearRatio << "]\n";

    // get joint offset associated with this motor
    sdf::ElementPtr encoderOffsetSDF =
      motorSDF->GetElement("encoder_offset");
    this->motorInfos[id].encoderOffset = encoderOffsetSDF->Get<double>();
    // gzdbg << "  enc offset [" << this->motorInfos[id].encoderOffset << "]\n";

    // get max [continuous] motor torque associated with this motor
    sdf::ElementPtr motorTorqueSDF =
      motorSDF->GetElement("motor_torque");
    this->motorInfos[id].motorTorque = motorTorqueSDF->Get<double>();
    // gzdbg << "  torque [" << this->motorInfos[id].motorTorque << "]\n";

    // this should return index of the joint in this->haptixJoints
    // where this->jointNames matches motorInfos[id].jointName.
    /// \TODO: there must be a faster way of doing this?
    this->motorInfos[id].index = -1;
    for (unsigned int j = 0; j < this->jointNames.size(); ++j)
    {
      if (this->jointNames[j] == this->motorInfos[id].jointName)
      {
        this->motorInfos[id].index = j;
        break;
      }
    }

    // if fail to find a joint name matching one specified in motor block
    if (this->motorInfos[id].index == -1)
    {
      // create a fake joint, append it to the end
      int j1 = this->haptixJoints.size();
      JointHelper *hj = new JointHelper();
      hj->SetJointName(this->motorInfos[id].jointName);
      this->haptixJoints.push_back(hj);
      this->motorInfos[id].index = j1;
    }

    // get coupled joints from <gearbox> blocks
    if (motorSDF->HasElement("gearbox"))
    {
      sdf::ElementPtr gearboxSDF = motorSDF->GetElement("gearbox");
      while (gearboxSDF)
      {
        // get joint, offset and multiplier1 and multiplier2
        sdf::ElementPtr jointSDF = gearboxSDF->GetElement("joint");
        sdf::ElementPtr offsetSDF = gearboxSDF->GetElement("offset");
        sdf::ElementPtr multiplier1SDF = gearboxSDF->GetElement("multiplier1");
        sdf::ElementPtr multiplier2SDF = gearboxSDF->GetElement("multiplier2");
        MotorInfo::GearBox g;

        // find index in this->haptixJoints that matches
        // jointSDF->Get<std::string>();
        g.index = -1;
        for (unsigned int k = 0; k < this->jointNames.size(); ++k)
        {
          if (this->jointNames[k] == jointSDF->Get<std::string>())
          {
            g.index = k;
            break;
          }
        }
        if (g.index == -1)
          gzerr <<  "failed to find joint for gearbox\n";

        g.offset = offsetSDF->Get<double>();
        g.multiplier1 = multiplier1SDF->Get<double>();
        g.multiplier2 = multiplier2SDF->Get<double>();
        this->motorInfos[id].gearboxes.push_back(g);

        gearboxSDF = gearboxSDF->GetNextElement("gearbox");
      }
    }

    // get next sdf
    motorSDF = motorSDF->GetNextElement("motor");
  }

  // Get pid gains and insert into pid
  this->pids.resize(this->haptixJoints.size());
  sdf::ElementPtr pid = this->sdf->GetElement("pid");
  // gzdbg << "getting pid\n";
  while (pid)
  {
    pid->GetAttribute("id")->Get(id);
    // gzdbg << "getting pid [" << id << "]\n";
    double pVal, iVal, dVal, cmdMaxVal, cmdMinVal;
    pid->GetAttribute("p")->Get(pVal);
    pid->GetAttribute("i")->Get(iVal);
    pid->GetAttribute("d")->Get(dVal);
    pid->GetAttribute("cmd_max")->Get(cmdMaxVal);
    pid->GetAttribute("cmd_min")->Get(cmdMinVal);
    this->pids[id] = common::PID(pVal, iVal, dVal, cmdMaxVal, cmdMinVal);

    // get next sdf
    pid = pid->GetNextElement("pid");
  }

  // Adjust max/min torque commands based on <motor_torque>,
  // <gear_ratio> and <gearbox> params.
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    int m = this->motorInfos[i].index;
    // gzdbg << m << " : " << this->simRobotCommands[m].ref_pos << "\n";
    double jointTorque = this->motorInfos[i].gearRatio *
                         this->motorInfos[i].motorTorque;
    this->pids[m].SetCmdMax(jointTorque);
    this->pids[m].SetCmdMin(-jointTorque);
    // gzdbg << " motor torque [" << m
    //       << "] : " << jointTorque << "\n";

    /// \TODO: contemplate about using Joint::SetEffortLimit()
    /// instead of PID::SetCmdMax() and PID::SetCmdMin()

    // set torque command limits through <gearbox> coupling params.
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
    {
      int n = this->motorInfos[i].gearboxes[j].index;
      /// \TODO Use multiplier1 for reporting, but this value changes now
      /// as a function of joint angle. Implement later.
      /// See issue #60.
      double coupledJointTorque = jointTorque /
        this->motorInfos[i].gearboxes[j].multiplier1;
      this->pids[n].SetCmdMax(coupledJointTorque);
      this->pids[n].SetCmdMin(-coupledJointTorque);
      // gzdbg << "   coupled motor torque [" << n
      //       << "] : " << coupledJointTorque << "\n";
    }
  }

  // get sensor manager from gazebo
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
  while (!mgr->SensorsInitialized())
  {
    gzdbg << "waiting for SensorManager to initialize.\n";
    usleep(100000);
  }

  // Get contact sensor names and insert id/name pair into map
  sdf::ElementPtr contactSensorSDF = this->sdf->GetElement("contactSensor");
  while (contactSensorSDF)
  {
    contactSensorSDF->GetAttribute("id")->Get(id);
    // gzdbg << "getting contact sensor [" << id << "]\n";
    this->contactSensorNames[id] = contactSensorSDF->Get<std::string>();

    // Get a pointer to the contact sensor
    sensors::SensorPtr sensor = mgr->GetSensor(this->contactSensorNames[id]);

    sensors::ContactSensorPtr contactSensor =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (contactSensor)
    {
      ContactSensorInfo info;
      info.sensor = sensor;

      info.connection = sensor->ConnectUpdated(
        boost::bind(&HaptixControlPlugin::OnContactSensorUpdate, this, id));

      // keep vector of all contact sensors
      this->contactSensorInfos.push_back(info);
    }
    else
    {
      gzerr << "Contact Sensor [" << this->contactSensorNames[id]
            << "] not found.\n";
    }

    // get next sdf
    contactSensorSDF = contactSensorSDF->GetNextElement("contactSensor");
  }

  // Get imuSensor names and insert id/name pair into map
  sdf::ElementPtr imuSensor = this->sdf->GetElement("imuSensor");
  while (imuSensor)
  {
    imuSensor->GetAttribute("id")->Get(id);
    this->imuSensorNames[id] = imuSensor->Get<std::string>();
    // gzdbg << "getting imuSensor [" << id << "]\n";

    // Get a pointer to the imu sensor
    sensors::ImuSensorPtr sensor =
        boost::dynamic_pointer_cast<sensors::ImuSensor>
        (mgr->GetSensor(this->imuSensorNames[id]));
    if (sensor)
    {
      /// \TODO: assume id in order
      this->imuSensors.push_back(sensor);
    }
    else
    {
      gzerr << "Imu Sensor [" << this->imuSensorNames[id]
            << "] not found.\n";
    }

    // get next sdf
    imuSensor = imuSensor->GetNextElement("imuSensor");
  }

  // Get predefined grasp poses
  sdf::ElementPtr grasp = this->sdf->GetElement("grasp");
  while(grasp)
  {
    std::string name;
    grasp->GetAttribute("name")->Get(name);
    std::string graspBuffer;
    grasp->GetValue()->Get(graspBuffer);
    std::istringstream iss(graspBuffer);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    for(unsigned int i = 0; i < tokens.size(); i++)
      this->grasps[name].push_back(stof(tokens[i]));
    grasp = grasp->GetNextElement("grasp");
  }

  this->graspMode = false;

  // Allocate memory for all the protobuf fields.
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    // motor states
    this->robotState.add_motor_pos(0);
    this->robotState.add_motor_vel(0);
    this->robotState.add_motor_torque(0);

    // initialize position command.
    this->robotCommand.add_ref_pos(0.0);
    this->robotCommand.add_ref_vel_max(0.0);

    // default position and velocity gains
    this->robotCommand.add_gain_pos(1.0);
    this->robotCommand.add_gain_vel(0.0);
  }
  // initialize to control mode, not gain mode
  this->robotCommand.set_ref_pos_enabled(true);
  this->robotCommand.set_ref_vel_max_enabled(false);
  this->robotCommand.set_gain_pos_enabled(false);
  this->robotCommand.set_gain_vel_enabled(false);

  for (unsigned int i = 0; i < this->haptixJoints.size(); ++i)
  {
    if (this->haptixJoints[i]->HasJoint())
    {
      this->robotState.add_joint_pos(0);
      this->robotState.add_joint_vel(0);

      // internal command of all joints controller here (not just motors)
      SimRobotCommand c;
      c.ref_pos = 0.0;
      c.ref_vel_max = 0.0;
      c.gain_pos = 1.0;
      c.gain_vel = 0.0;
      this->simRobotCommands.push_back(c);
    }
  }

  // add a robot state contact per contactSensorInfo
  for (unsigned int i = 0; i < this->contactSensorInfos.size(); ++i)
  {
    this->robotState.add_contact(0);
  }

  for (unsigned int i = 0; i < imuSensors.size(); ++i)
  {
    haptix::comm::msgs::imu *linacc = this->robotState.add_imu_linear_acc();
    linacc->set_x(0);
    linacc->set_y(0);
    linacc->set_z(0);
    haptix::comm::msgs::imu *angvel = this->robotState.add_imu_angular_vel();
    angvel->set_x(0);
    angvel->set_y(0);
    angvel->set_z(0);
    haptix::comm::msgs::quaternion *orientation =
      this->robotState.add_imu_orientation();
    orientation->set_x(0);
    orientation->set_y(0);
    orientation->set_z(0);
    orientation->set_w(1);
  }

  this->robotState.mutable_time_stamp()->set_sec(0);
  this->robotState.mutable_time_stamp()->set_nsec(0);
}

/////////////////////////////////////////////////
void HaptixControlPlugin::Reset()
{
  this->targetBaseLinkPose = this->initialBaseLinkPose;

  std::vector<SimRobotCommand>::iterator iter;
  for (iter = this->simRobotCommands.begin();
      iter != this->simRobotCommands.end(); ++iter)
  {
    iter->ref_pos = 0.0;
    iter->ref_vel_max = 0.0;
  }
}

/////////////////////////////////////////////////
// Open keyboard commands
void HaptixControlPlugin::SetKeyboardPose(const std::string &/*_topic*/,
                     const msgs::Pose &_pose)
{
  math::Pose inputPose(msgs::Convert(_pose));

  this->keyboardPose.pos += inputPose.pos;
  this->keyboardPose.rot = inputPose.rot * this->keyboardPose.rot;

  // Add pose to our keyboardPose object
  this->staleKeyboardPose = false;
}

/////////////////////////////////////////////////
bool HaptixControlPlugin::LoadKeyboard()
{
  this->keyboardPose = this->initialBaseLinkPose;
  this->staleKeyboardPose = true;
  if (ignNode.Subscribe("/haptix/arm_pose_inc",
        &HaptixControlPlugin::SetKeyboardPose, this))
  {
    printf("Successfully connected to keyboard node\n");
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::UpdateSpacenav(double _dt)
{
  if (!this->newJoystickMessage)
  {
    return;
  }

  msgs::Joystick joy;
  {
    boost::mutex::scoped_lock lock(this->joystickMessageMutex);
    joy = this->latestJoystickMessage;
    this->newJoystickMessage = false;
  }

  math::Vector3 posRate, rotRate;

  if (joy.has_translation())
  {
    posRate = msgs::Convert(joy.translation());
  }

  if (joy.has_rotation())
  {
    rotRate = msgs::Convert(joy.rotation());
  }

  // rotate posRate into camera frame
  math::Pose camPose;
  {
    boost::mutex::scoped_lock lock(this->userCameraPoseMessageMutex);
    camPose = this->userCameraPose;
  }
  posRate = camPose.rot.RotateVector(posRate);
  rotRate = camPose.rot.RotateVector(rotRate);

  {
    const double posScale = 2.0;
    const double rotScale = 5.0;

    boost::mutex::scoped_lock lock(this->baseLinkMutex);
    math::Pose targetSpacenavPose = this->baseLinktoSpacenavPose
                                  + this->targetBaseLinkPose;
    targetSpacenavPose.pos += _dt * posScale * posRate;
    targetSpacenavPose.rot =
      targetSpacenavPose.rot.Integrate(rotScale * rotRate, _dt);

    // apply inverse transform from spacenav reference point
    // back to base link pose
    this->targetBaseLinkPose = this->baseLinktoSpacenavPose.GetInverse()
                             + targetSpacenavPose;
  }
}

/////////////////////////////////////////////////
// Update targetBaseLinkPose using Polhemus
void HaptixControlPlugin::UpdatePolhemus()
{
  // Wait for the user camera pose to be valid, to avoid possible race
  // condition on startup.
  while (!this->userCameraPoseValid)
    usleep(1000);
  // Get current pose from Polhemus
  polhemus_pose_t poses[8];
  while (true)
  {
    int numPoses = 8;  // fill with max poses to read, returns actual poses
    if (!polhemus_get_poses(this->polhemusConn, poses, &numPoses, 100))
    {
      boost::mutex::scoped_lock pauseLock(this->pauseMutex);

      int armId = 0;  // some number between 0 and numPoses
      if (armId < numPoses)
      {
        // lock in case we are receiving a pause polhemus message over
        // gz transport
        math::Pose armSensorPose = this->convertPolhemusToPose(poses[armId]);
        if (this->pauseTracking)
        {
          // calibration mode, update offset
          this->sourceWorldPoseArmOffset =
            (armSensorPose.GetInverse() + this->baseLinkToArmSensor +
             this->targetBaseLinkPose) - this->sourceWorldPose;
        }
        else
        {
          boost::mutex::scoped_lock lock(this->baseLinkMutex);
          this->targetBaseLinkPose = this->baseLinkToArmSensor.GetInverse()
            + armSensorPose
            + (this->sourceWorldPoseArmOffset + this->sourceWorldPose);
        }
      }

      int headId = 1;
      if (headId < numPoses)
      {
        math::Pose headSensorPose = this->convertPolhemusToPose(poses[headId]);
        if (this->pauseTracking)
        {
          boost::mutex::scoped_lock lock(this->userCameraPoseMessageMutex);
          // calibration mode, update offset
          this->sourceWorldPoseHeadOffset =
            (headSensorPose.GetInverse() + this->cameraToHeadSensor +
             this->userCameraPose) - this->sourceWorldPose;
        }
        else
        {
          math::Pose targetCameraPose = this->cameraToHeadSensor.GetInverse()
            + headSensorPose
            + (this->sourceWorldPoseHeadOffset + this->sourceWorldPose);

          gazebo::msgs::Set(&this->joyMsg, targetCameraPose);
          this->viewpointJoyPub->Publish(this->joyMsg);
        }
      }
    }
    else
    {
      gzerr << "polhemus_get_pose() failed\n";
      /*
      // test reconnect?
      if(!(this->polhemusConn = polhemus_connect_usb(LIBERTY_HS_VENDOR_ID,
                                                 LIBERTY_HS_PRODUCT_ID,
                                                 LIBERTY_HS_WRITE_ENDPOINT,
                                                 LIBERTY_HS_READ_ENDPOINT)))
      {
        fprintf(stderr, "Failed to connect to Polhemus\n");
      }
      else
      {
        if(polhemus_init_comm(this->polhemusConn, 10))
        {
          fprintf(stderr, "Failed to initialize comms with Polhemus\n");
        }
        else
          this->havePolhemus = true;
      }
      */
    }
    usleep(1000);
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::UpdateKeyboard(double /*_dt*/)
{
  boost::mutex::scoped_lock lock(this->baseLinkMutex);
  if (!this->staleKeyboardPose)
  {
    this->targetBaseLinkPose = this->keyboardPose;
    this->staleKeyboardPose = true;
  }
  else
  {
    this->keyboardPose = this->targetBaseLinkPose;
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::UpdateBaseLink(double _dt)
{
  math::Pose pose;
  {
    boost::mutex::scoped_lock lock(this->baseLinkMutex);
    pose = this->targetBaseLinkPose;
  }

  math::Pose baseLinkPose = this->baseLink->GetWorldPose();

  math::Vector3 errorPos = baseLinkPose.pos - pose.pos;

  math::Vector3 errorRot =
    (baseLinkPose.rot * pose.rot.GetInverse()).GetAsEuler();

  this->wrench.force.x = this->posPid.Update(errorPos.x, _dt);
  this->wrench.force.y = this->posPid.Update(errorPos.y, _dt);
  this->wrench.force.z = this->posPid.Update(errorPos.z, _dt);
  this->wrench.torque.x = this->rotPid.Update(errorRot.x, _dt);
  this->wrench.torque.y = this->rotPid.Update(errorRot.y, _dt);
  this->wrench.torque.z = this->rotPid.Update(errorRot.z, _dt);
  this->baseLink->SetForce(this->wrench.force);
  this->baseLink->SetTorque(this->wrench.torque);
  // std::cout << "current pose: " << baseLinkPose << std::endl;
  // std::cout << "target pose: " << pose << std::endl;
  // std::cout << "wrench pos: " << this->wrench.force
  //           << " rot: " << this->wrench.torque << std::endl;
}

/////////////////////////////////////////////////
void HaptixControlPlugin::GetHandControlFromClient()
{
  // copy command from hxCommand for motors to list of all joints
  // commanded by this plugin.
  // also account for joint coupling here based on <gearbox> params
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    int m = this->motorInfos[i].index;
    // gzdbg << m << " : " << this->simRobotCommands[m].ref_pos << "\n";
    // If we're in grasp mode, then take commands from elsewhere
    unsigned int numWristMotors = 3;
    if (this->graspMode && i >= numWristMotors)
    {
      if (this->robotCommand.ref_pos_enabled())
      {
        this->simRobotCommands[m].ref_pos = this->graspPositions[i];
      }
      if (this->robotCommand.ref_vel_max_enabled())
      {
        this->simRobotCommands[m].ref_vel_max = 0.0;
      }
    }
    else
    {
      if (this->robotCommand.ref_pos_enabled())
      {
        this->simRobotCommands[m].ref_pos = this->robotCommand.ref_pos(i);
      }
      if (this->robotCommand.ref_vel_max_enabled())
      {
        this->simRobotCommands[m].ref_vel_max =
          this->robotCommand.ref_vel_max(i);
      }
    }
    if (this->robotCommand.gain_pos_enabled())
      this->simRobotCommands[m].gain_pos = this->robotCommand.gain_pos(i);
    if (this->robotCommand.gain_vel_enabled())
      this->simRobotCommands[m].gain_vel = this->robotCommand.gain_vel(i);

    // Use motor commands to set set joint command
    // based on coupling specified in <gearbox> params.

    // First, find minimum offset from all gearboxes:
    double minOffset = 1e16;
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
      minOffset = std::min(minOffset, this->motorInfos[i].gearboxes[j].offset);

    // Next, apply transmission as defined:
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
    {
      int n = this->motorInfos[i].gearboxes[j].index;
      // gzdbg << " " << n
      //       << " : " << this->simRobotCommands[n].ref_pos
      //       << " : " << this->robotCommand.ref_pos_enabled() << "\n";

      // See transmission specification in issue #60,
      // If motor angle commanded is less than offset
      // use multiplier1, otherwise use multiplier2
      if (this->simRobotCommands[m].ref_pos < minOffset)
      {
        if (this->robotCommand.ref_pos_enabled())
        {
          this->simRobotCommands[n].ref_pos =
            this->simRobotCommands[m].ref_pos
            * this->motorInfos[i].gearboxes[j].multiplier1;
        }
        if (this->robotCommand.ref_vel_max_enabled())
        {
          this->simRobotCommands[n].ref_vel_max =
            this->simRobotCommands[m].ref_vel_max
            / this->motorInfos[i].gearboxes[j].multiplier1;
        }
      }
      else
      {
        if (this->robotCommand.ref_pos_enabled())
        {
          this->simRobotCommands[n].ref_pos =
            (this->simRobotCommands[m].ref_pos -
             this->motorInfos[i].gearboxes[j].offset)
            * this->motorInfos[i].gearboxes[j].multiplier2
            + this->motorInfos[i].gearboxes[j].offset
            * this->motorInfos[i].gearboxes[j].multiplier2;
        }
        if (this->robotCommand.ref_vel_max_enabled())
        {
          this->simRobotCommands[n].ref_vel_max =
            this->simRobotCommands[m].ref_vel_max
            / this->motorInfos[i].gearboxes[j].multiplier2;
        }
      }
      // set gains
      if (this->robotCommand.gain_pos_enabled())
        this->simRobotCommands[n].gain_pos = this->robotCommand.gain_pos(i);
      if (this->robotCommand.gain_vel_enabled())
        this->simRobotCommands[n].gain_vel = this->robotCommand.gain_vel(i);
    }
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::UpdateHandControl(double _dt)
{
  // command all joints
  for(unsigned int i = 0; i < this->haptixJoints.size(); ++i)
  {
    // get joint positions and velocities
    double position = this->haptixJoints[i]->GetAngle(0).Radian();
    double velocity = this->haptixJoints[i]->GetVelocity(0);

    // compute target joint position and velocity error in gazebo
    double errorPos = position - this->simRobotCommands[i].ref_pos;
    double errorVel = velocity - this->simRobotCommands[i].ref_vel_max;

    // compute overall error
    double error = this->simRobotCommands[i].gain_pos * errorPos
                 + this->simRobotCommands[i].gain_vel * errorVel;

    // compute force needed
    double force = this->pids[i].Update(error, _dt);

    // this->robotState.set_motor_torque(i, force);
    if (!this->haptixJoints[i]->SetForce(0, force))
    {
      // not a real gazebo joint, set target directly
      this->haptixJoints[i]->SetPosition(position);
      /// \TODO: something about velocity commands
      // this->haptixJoints[i]->SetVelocity(velocity);
    }
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::OnContactSensorUpdate(int _i)
{
  // how do we know which sensor triggered this update?
  // gzerr << "contactSensorInfos " << this->contactSensorInfos.size() << "\n";
  if (_i >= static_cast<int>(this->contactSensorInfos.size()))
  {
    gzerr << "sensor [" << _i
          << "] is out of range of contactSensorInfos["
          << this->contactSensorInfos.size() << "].\n";
    return;
  }
  sensors::ContactSensorPtr contactSensor =
    boost::dynamic_pointer_cast<sensors::ContactSensor>(
    this->contactSensorInfos[_i].sensor);

  if (!contactSensor)
  {
    gzerr << "sensor [" << this->contactSensorInfos[_i].sensor->GetName()
          << "] is not a ContactSensor.\n";
    return;
  }
  msgs::Contacts contacts = contactSensor->GetContacts();
  // contact sensor report contact between pairs of bodies
  // if (contacts.contact().size() > 0)
  //   gzerr << "  name " << contactSensor->GetName()
  //         << " contacts " << contacts.contact().size() << "\n";

  // reset aggregate forces and torques if contacts detected
  this->contactSensorInfos[_i].contactForce = math::Vector3();
  this->contactSensorInfos[_i].contactTorque = math::Vector3();

  for (int j = 0; j < contacts.contact().size(); ++j)
  {
    msgs::Contact contact = contacts.contact(j);
    // each contact can have multiple wrenches
    // if (contact.wrench().size() > 0)
    //   gzerr << "    wrenches " << contact.wrench().size() << "\n";
    for (int k = 0; k < contact.wrench().size(); ++k)
    {
      msgs::JointWrench wrenchMsg = contact.wrench(k);

      // sum up all wrenches from body_1 or body_2
      // check with contact corresponds to the arm and which to the arm
      // compare body_1_name and body_2_name with model name

      if (strncmp(this->model->GetName().c_str(),
                  wrenchMsg.body_1_name().c_str(),
                  this->model->GetName().size()) == 0)
      {
        this->contactSensorInfos[_i].contactForce +=
          msgs::Convert(wrenchMsg.body_1_wrench().force());
        this->contactSensorInfos[_i].contactTorque +=
          msgs::Convert(wrenchMsg.body_1_wrench().torque());
      }
      else if (strncmp(this->model->GetName().c_str(),
                       wrenchMsg.body_2_name().c_str(),
                       this->model->GetName().size()) == 0)
      {
        this->contactSensorInfos[_i].contactForce +=
          msgs::Convert(wrenchMsg.body_2_wrench().force());
        this->contactSensorInfos[_i].contactTorque +=
          msgs::Convert(wrenchMsg.body_2_wrench().torque());
      }
      else
      {
        gzerr << "collision name does not match model name. This should "
               << "never happen." << std::endl;
        return;
      }

      // gzerr << "        contact [" << _i << ", " << j
      //       << ", " << k << "] : [" << contactForce << "]\n";
    }
  }
  // gzerr << "contact [" << _i
  //       << "]: [" << this->contactSensorInfos[_i].contactForce
  //       << "]\n";
}

/////////////////////////////////////////////////
void HaptixControlPlugin::PublishHaptixControlStatus()
{
  // finished loading arm? send status
  this->haptixStatusPub =
    this->gazeboNode->Advertise<gazebo::msgs::Int>("~/haptix_load");
  gazebo::msgs::Int loadStat;
  loadStat.set_data(1);
  this->haptixStatusPub->Publish(loadStat);
}

/////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::GetRobotStateFromSim()
{
  // fill robot state motor_pos, motor_vel, motor_torque
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    int m = motorInfos[i].index;
    double jointPosition = this->haptixJoints[m]->GetAngle(0).Radian();
    double jointVelocity = this->haptixJoints[m]->GetVelocity(0);
    double jointTorque = this->haptixJoints[m]->GetForce(0);
    // convert joint angle and velocities into motor using gear_ratio
    double motorPosition = jointPosition * this->motorInfos[i].gearRatio
      - this->motorInfos[i].encoderOffset;
    double motorVelocity = jointVelocity / this->motorInfos[i].gearRatio;
    double motorTorque = jointTorque / this->motorInfos[i].gearRatio;
    // write to struct
    this->robotState.set_motor_pos(i, motorPosition);
    this->robotState.set_motor_vel(i, motorVelocity);
    this->robotState.set_motor_torque(i, motorTorque);
  }

  // fill robot state joint_pos and joint_vel
  unsigned int count = 0;
  for (unsigned int i = 0; i < this->haptixJoints.size(); ++i)
  {
    if (this->haptixJoints[i]->HasJoint())
    {
      this->robotState.set_joint_pos(count,
        this->haptixJoints[i]->GetAngle(0).Radian());
      this->robotState.set_joint_vel(count,
        this->haptixJoints[i]->GetVelocity(0));
      ++count;
    }
  }

  // copy contact forces
  // gzerr << "contactSensorInfos " << this->contactSensorInfos.size() << "\n";
  for (unsigned int i = 0; i < this->contactSensorInfos.size(); ++i)
  {
    // get summed force from contactSensorInfos
    double force = this->contactSensorInfos[i].contactForce.GetLength();
    // return summed force
    this->robotState.set_contact(i, force);
  }

  for (unsigned int i = 0; i < this->imuSensors.size(); ++i)
  {
    haptix::comm::msgs::imu *linacc =
        this->robotState.mutable_imu_linear_acc(i);
    math::Vector3 acc = this->imuSensors[i]->GetLinearAcceleration();
    math::Vector3 vel = this->imuSensors[i]->GetAngularVelocity();
    linacc->set_x(acc.x);
    linacc->set_y(acc.y);
    linacc->set_z(acc.z);
    haptix::comm::msgs::imu *angvel =
        this->robotState.mutable_imu_angular_vel(i);
    angvel->set_x(vel.x);
    angvel->set_y(vel.y);
    angvel->set_z(vel.z);
    // Orientation not yet supported
    haptix::comm::msgs::quaternion *orientation =
        this->robotState.mutable_imu_orientation(i);
    orientation->set_x(0);
    orientation->set_y(0);
    orientation->set_z(0);
    orientation->set_w(1);
  }

  common::Time curTime = this->world->GetSimTime();
  this->robotState.mutable_time_stamp()->set_sec(curTime.sec);
  this->robotState.mutable_time_stamp()->set_nsec(curTime.nsec);
}

/////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::GazeboUpdateStates()
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  common::Time curTime = this->world->GetSimTime();
  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    // update target pose with spacenav
    this->UpdateSpacenav(dt);

    if (this->haveKeyboard)
      this->UpdateKeyboard(dt);

    // compute wrench needed
    this->UpdateBaseLink(dt);

    // Update based on updateRate.
    if (curTime - this->lastSimTimeForControlThrottling >=
        1.0/this->updateRate)
    {
      // Uncomment this to see the update rate.
      // gzdbg << 1.0/(common::Time::GetSimTime() -
      //               this->lastSimTime).Double() << std::endl;

      // Get robot state from simulation
      this->GetRobotStateFromSim();

      // Get simulation control from client
      this->GetHandControlFromClient();

      this->lastSimTimeForControlThrottling = curTime;
    }

    // control finger joints
    this->UpdateHandControl(dt);

    // report back if motion tracking system is paused
    if (this->gotPauseRequest)
    {
      // signal pause completion
      msgs::Int res;
      res.set_data(this->pauseTracking);
      this->pausePub->Publish(res);
      // reset flag
      this->gotPauseRequest = false;
    }

    this->lastTime = curTime;
  }
  else if (dt < 0)
  {
    // has time been reset?
    this->lastTime = curTime;
  }
}

/////////////////////////////////////////////////
// Play the trajectory, update states
math::Pose HaptixControlPlugin::convertPolhemusToPose(double x, double y,
  double z, double roll, double pitch, double yaw)
{
  // const double M_PER_INCH = 2.54e-2;
  const double M_PER_CM = 1e-2;
  const double RAD_PER_DEG = M_PI/180.0;
  return math::Pose(x*M_PER_CM, y*M_PER_CM, z*M_PER_CM,
                    roll*RAD_PER_DEG, pitch*RAD_PER_DEG, yaw*RAD_PER_DEG);
}

/////////////////////////////////////////////////
// Play the trajectory, update states
math::Pose HaptixControlPlugin::convertPolhemusToPose(
  const polhemus_pose_t &_pose)
{
  // (-x,y,z,yaw,-pitch,-roll) seems to do the right thing;
  // original
  // return this->convertPolhemusToPose(
  //   -_pose.x, _pose.y, _pose.z, _pose.yaw, -_pose.pitch, -_pose.roll);
  // test
  return this->convertPolhemusToPose(
    _pose.x, _pose.y, _pose.z, _pose.yaw, _pose.pitch, _pose.roll);
}

//////////////////////////////////////////////////
void HaptixControlPlugin::HaptixGetRobotInfoCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxRobot &/*_req*/,
      haptix::comm::msgs::hxRobot &_rep, bool &_result)
{
  // is this needed?
  // if (_service != deviceInfoTopic)
  //   _result = false;

  _rep.set_motor_count(this->motorInfos.size());
  _rep.set_joint_count(this->jointNames.size());
  _rep.set_contact_sensor_count(this->contactSensorInfos.size());
  _rep.set_imu_count(this->imuSensors.size());

  for (unsigned int i = 0; i < this->haptixJoints.size(); ++i)
  {
    if (this->haptixJoints[i]->HasJoint())
    {
      haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
      joint->set_minimum(this->haptixJoints[i]->GetLowerLimit(0).Radian());
      joint->set_maximum(this->haptixJoints[i]->GetUpperLimit(0).Radian());
    }
  }

  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    int m = this->motorInfos[i].index;
    haptix::comm::msgs::hxRobot::hxLimit *motor = _rep.add_motor_limit();
    // compute the motor limits from joint limits
    double jointMin = this->haptixJoints[m]->GetLowerLimit(0).Radian();
    double jointMax = this->haptixJoints[m]->GetUpperLimit(0).Radian();
    double motorMin = jointMin * this->motorInfos[i].gearRatio
      - this->motorInfos[i].encoderOffset;
    double motorMax = jointMax * this->motorInfos[i].gearRatio
      - this->motorInfos[i].encoderOffset;
    if (this->motorInfos[i].gearRatio < 0)
    {
      // flip if gearRatio is negative
      motor->set_maximum(motorMin);
      motor->set_minimum(motorMax);
    }
    else
    {
      motor->set_minimum(motorMin);
      motor->set_maximum(motorMax);
    }
    // gzerr << motorMin << " : " << motorMax << "\n";
  }

  _rep.set_update_rate(this->updateRate);

  _result = true;
}

//////////////////////////////////////////////////
/// using haptix-comm service callback
void HaptixControlPlugin::HaptixUpdateCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxCommand &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  // Read the request parameters.
  // Debug output.
  /*std::cout << "Received a new command:" << std::endl;
  for (unsigned int i = 0; i < this->haptixJoints.size(); ++i)
  {
    std::cout << "\tMotor " << i << ":" << std::endl;
    std::cout << "\t\t" << _req.ref_pos(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel_max(i) << std::endl;
    std::cout << "\t\t" << _req.gain_pos(i) << std::endl;
    std::cout << "\t\t" << _req.gain_vel(i) << std::endl;
  }
  std::cout << "\tref_pos_enabled\t"
            << _req.ref_pos_enabled()     << std::endl;
  std::cout << "\tref_vel_max_enabled\t"
            << _req.ref_vel_max_enabled() << std::endl;
  std::cout << "\tgain_pos_enabled\t"
            << _req.gain_pos_enabled()    << std::endl;
  std::cout << "\tgain_vel_enabled\t"
            << _req.gain_vel_enabled()    << std::endl;
  */

  this->robotCommand = _req;

  _rep.Clear();

  _rep = this->robotState;

  _result = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUserCameraPose(ConstPosePtr &_msg)
{
  boost::mutex::scoped_lock lock(this->userCameraPoseMessageMutex);
  this->userCameraPose = math::Pose(msgs::Convert(*_msg));
  this->userCameraPoseValid = true;
}

//////////////////////////////////////////////////
/// using ign-transport service, out of band from haptix_comm
void HaptixControlPlugin::HaptixGraspCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxGrasp &_req,
      haptix::comm::msgs::hxCommand &_rep, bool &_result)
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  if (this->graspPositions.size() != this->motorInfos.size())
    this->graspPositions.resize(this->motorInfos.size());

  for (unsigned int j = 0; j < this->graspPositions.size(); ++j)
  {
    this->graspPositions[j] = 0.0;
    _rep.add_ref_pos(0.0);
  }

  // gzerr << "ref_pos_enabled: "
  //       << this->robotCommand.ref_pos_enabled() << "\n";

  _rep.set_ref_pos_enabled(this->robotCommand.ref_pos_enabled());
  _rep.set_ref_vel_max_enabled(this->robotCommand.ref_vel_max_enabled());
  _rep.set_gain_pos_enabled(this->robotCommand.gain_pos_enabled());
  _rep.set_gain_vel_enabled(this->robotCommand.gain_vel_enabled());

  for (int i=0; i < _req.grasps_size(); ++i)
  {
    std::string name = _req.grasps(i).grasp_name();
    std::map<std::string, std::vector<float> >::const_iterator g =
      this->grasps.find(name);
    if (g != this->grasps.end())
    {
      for (unsigned int j=0;
          j < g->second.size() && j < this->graspPositions.size(); ++j)
      {
        float value = _req.grasps(i).grasp_value();
        if (value < 0.0)
          value = 0.0;
        if (value > 1.0)
          value = 1.0;
        // This superposition logic could use a lot of thought.  But it should
        // at least work for the case of a single type of grasp.
        this->graspPositions[j] += value * g->second[j] / _req.grasps_size();
        _rep.set_ref_pos(j, this->graspPositions[j]);
      }
    }
  }

  // An empty request puts us back in single-finger control mode
  if (_req.grasps_size() == 0)
    this->graspMode = false;
  else
    this->graspMode = true;

  _result = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::HaptixReadCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxSensor &/*_req*/,
      haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  boost::mutex::scoped_lock lock(this->updateMutex);
  _rep.Clear();

  _rep = this->robotState;

  _result = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnHydra(ConstHydraPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->hydraMessageMutex);
  this->haveHydra = true;
  this->hydraPose = math::Pose(msgs::Convert(_msg->right().pose()));

  math::Pose armSensorPose = this->hydraPose;
  if (this->pauseTracking)
  {
    // calibration mode, update offset
    this->sourceWorldPoseArmOffset =
      (armSensorPose.GetInverse() + this->baseLinkToHydraSensor +
       this->targetBaseLinkPose) - this->sourceWorldPose;
  }
  else
  {
    boost::mutex::scoped_lock baseLock(this->baseLinkMutex);
    this->targetBaseLinkPose = this->baseLinkToHydraSensor.GetInverse()
      + armSensorPose
      + (this->sourceWorldPoseArmOffset + this->sourceWorldPose);
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnJoy(ConstJoystickPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->joystickMessageMutex);
  this->latestJoystickMessage = *_msg;
  this->newJoystickMessage = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnPause(ConstIntPtr &_msg)
{
  boost::mutex::scoped_lock pauseLock(this->pauseMutex);
  if (!this->havePolhemus)
  {
    gzdbg << "no polhemus, but responding to pause request\n";
  }

  // gzerr << "got " << _msg->data() << "\n";
  if (_msg->data() == 0)
  {
    this->pauseTracking = false;
  }
  else
  {
    this->pauseTracking = true;
  }
  // set request flag
  this->gotPauseRequest = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUpdateOptitrackHead(ConstPosePtr &_msg)
{
  /// The pose transformation math traverses the following transform tree.

  /// G ---------> O --------> M ---> C
  /// | \          \__________^ ^     ^
  /// |  \______________________|     |
  /// \_______________________________/

  /// G: Gazebo origin
  /// O: Optitrack origin
  /// M: Head marker
  /// C: Gazebo camera
  /// We will denote the transform from B to A in B's frame as A_B

  /// rawMarkerPose is the pose of the marker in the Optitrack frame: M_O
  gazebo::math::Pose rawMarkerPose = gazebo::msgs::Convert(*_msg);
  rawMarkerPose.pos = this->headPosFilter.Process(rawMarkerPose.pos);
  rawMarkerPose.rot = this->headOriFilter.Process(rawMarkerPose.rot);
  std::unique_lock<std::mutex> view_lock(this->viewpointRotationsMutex,
      std::try_to_lock);

  boost::mutex::scoped_lock lock(this->userCameraPoseMessageMutex);

  /// If we're paused, or if we haven't calculated an offset yet...
  if (this->pauseTracking || !this->headOffsetInitialized)
  {
    if (this->userCameraPoseValid)
    {
      /// gazeboToOptitrack is transform from gazebo to Optitrack: O_G
      /// O_G = O_M + M_C + C_G
      this->gazeboToOptitrack = -rawMarkerPose
          + this->cameraToOptitrackHeadMarker + this->userCameraPose;
      this->headOffsetInitialized = true;
    }
  }
  else
  {
    if (this->userCameraPoseValid)
    {
      /// C_G = C_M + M_O + O_G
      gazebo::math::Pose targetCameraPose = -this->cameraToOptitrackHeadMarker
          + rawMarkerPose + this->gazeboToOptitrack;
      if (!this->viewpointRotationsEnabled)
      {
        targetCameraPose.rot = this->userCameraPose.rot;
      }
      gazebo::msgs::Set(&this->joyMsg, targetCameraPose);
      this->viewpointJoyPub->Publish(this->joyMsg);
    }
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUpdateOptitrackArm(ConstPosePtr &_msg)
{
  boost::mutex::scoped_lock lock(this->baseLinkMutex);

  gazebo::math::Pose pose = this->optitrackWorldArmRot +
      gazebo::msgs::Convert(*_msg);

  this->optitrackArm = pose + this->optitrackArmOffset;

  // If we're paused, or if we haven't calculated an offset yet...
  if (this->pauseTracking || !this->armOffsetInitialized)
  {
    this->optitrackArmOffset = -pose + this->targetBaseLinkPose;
    this->armOffsetInitialized = true;
  }
  else
  {
    this->targetBaseLinkPose = this->optitrackArm;
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUpdateOptitrackMonitor(ConstPosePtr &_msg)
{
  this->monitorOptitrackFrame = gazebo::msgs::Convert(*_msg);
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnToggleViewpointRotations(ConstIntPtr &_msg)
{
  std::unique_lock<std::mutex> lock(this->viewpointRotationsMutex,
      std::try_to_lock);
  this->viewpointRotationsEnabled = _msg->data() == 0 ? false : true;
}

GZ_REGISTER_MODEL_PLUGIN(HaptixControlPlugin)
}
