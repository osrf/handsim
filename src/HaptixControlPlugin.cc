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

#include <gazebo/util/Diagnostics.hh>
#include <gazebo/common/Assert.hh>
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
  this->robotCommandTime = -1;
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
  if (this->polhemusThread.joinable())
    this->polhemusThread.join();

  this->optitrack.Stop();
  if (this->optitrackThread)
    this->optitrackThread->join();

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
  physics::PhysicsEnginePtr physics = this->world->GetPhysicsEngine();
  physics->SetParam("contact_sor_scale", 1.0);
  physics->SetParam("thread_position_correction", true);
  /* experimental
  physics->SetParam("warm_start_factor", 1.0);
  physics->SetParam("contact_residual_smoothing", 0.2);
  physics->SetParam("inertia_ratio_reduction", true);
  physics->SetParam("extra_friction_iterations", 10.0);
  */

  // start a transport node for polhemus head pose view point control
  this->gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  fprintf(stderr, "world name: [%s]\n", this->world->GetName().c_str());
  this->gazeboNode->Init(this->world->GetName());
  this->viewpointJoyPub =
    this->gazeboNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

  this->pausePub = this->gazeboNode->Advertise<gazebo::msgs::Int>
        ("~/motion_tracking/pause_response");

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
  if (_sdf->HasElement("base_link_to_arm_sensor_pose"))
  {
    this->baseLinkToArmSensor =
      _sdf->Get<math::Pose>("base_link_to_arm_sensor_pose");
  }
  // transform from polhemus sensor orientation to camera frame
  // 10cm to the right of the sensor is roughly where the eyes are
  // -0.3 rad pitch up: sensor is usually tilted upwards when worn on head
  this->cameraToHeadSensor = math::Pose(0, 0.10, 0, 0.0, -0.3, 0.0);

  // transformation from camera to marker in camera frame (optitrack)
  this->headMarker = math::Pose(0, 0, 0, M_PI/2, 0, 0);
  if (_sdf->HasElement("optitrack_head_to_marker_offset"))
  {
    this->headMarker =
      _sdf->Get<math::Pose>("optitrack_head_to_marker_offset");
  }


  this->viewpointRotationsSub = this->gazeboNode->Subscribe(
      "~/motion_tracking/viewpoint_rotations",
          &HaptixControlPlugin::OnToggleViewpointRotations, this);

  // hydra sensor offset
  this->baseLinkToHydraSensor = math::Pose(0, -0.3, 0, 0, 1.0*M_PI, -0.5*M_PI);
  if (_sdf->HasElement("hydra_control_point_offset"))
  {
    this->baseLinkToHydraSensor =
      _sdf->Get<math::Pose>("hydra_control_point_offset");
  }

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

  this->monitorScreen = gazebo::math::Pose::Zero;

  // Position of the screen in the world frame
  this->worldScreen = gazebo::math::Pose(0, -0.65, 1.5, 0, 0, 0);
  if (this->sdf->HasElement("world_screen_location"))
  {
    this->worldScreen = sdf->Get<math::Pose>("world_screen_location");
  }

  this->optitrackHeadOffset = gazebo::math::Pose::Zero;
  this->optitrackArmOffset = gazebo::math::Pose::Zero;
  // Transform from elbow of simulated arm to marker
  this->elbowMarker = math::Pose(gazebo::math::Vector3(0.0, -0.20, 0.07),
                                 gazebo::math::Quaternion(M_PI, -M_PI/2, 0));
  if (this->sdf->HasElement("base_link_sensor_offset"))
  {
    sdf::ElementPtr elbowMarkerSdf =
        this->sdf->GetElement("base_link_sensor_offset");
    this->elbowMarker = elbowMarkerSdf->Get<math::Pose>();
  }
  this->elbowMarkerCorrected = elbowMarker;

  this->armOffsetInitialized = false;
  this->headOffsetInitialized = false;

  this->headPosFilter.SetFc(0.002, 0.3);
  this->headOriFilter.SetFc(0.002, 0.3);
  this->torqueFilter.SetFc(1.0, 1000.0);

  this->polhemusGraspFilter.SetFc(0.01, 4);

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
  this->optitrackThread = std::make_shared<std::thread>(std::thread(
      &haptix::tracking::Optitrack::StartReception, std::ref(this->optitrack)));
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

  // DEMO hard coded
  this->currentPolhemusGrasp = "FinePinchOpen";
  this->arrangeSub = this->gazeboNode->Subscribe("~/arrange",
      &HaptixControlPlugin::OnArrange, this);

  // spin up a separate thread to get polhemus sensor data
  // update target pose if using polhemus
  if (this->havePolhemus)
  {
    this->polhemusThread = boost::thread(
      boost::bind(&HaptixControlPlugin::UpdatePolhemus, this));
  }
  else
    gzwarn << "No usable polhemus setup detected.\n";

  this->haveKeyboard = false;

  this->haveKeyboard = this->LoadKeyboard();

  if (!this->ignNode.Subscribe("/haptix/arm_model_pose",
        &HaptixControlPlugin::SetWorldPose, this))
  {
    gzerr << "setting arm pose subscriber failed, will not be"
          << " able to set arm pose directly.\n";
  }

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
  std::map<unsigned int, int> simOnly;

  // Get joint names and insert id/name pair into map
  sdf::ElementPtr jointName = this->sdf->GetElement("joint");
  while (jointName)
  {
    jointName->GetAttribute("id")->Get(id);
    int so = 0;
    if (jointName->HasAttribute("sim_only"))
    {
      jointName->GetAttribute("sim_only")->Get(so);
    }
    simOnly[id] = so;
    this->jointNames[id] = jointName->Get<std::string>();
    // get next sdf
    // gzdbg << "getting joint name [" << this->jointNames[id] << "]\n";
    jointName = jointName->GetNextElement("joint");
  }

  // Set initial simJoints size to match jointNames
  // this is incremented with fake joints from motor specifications
  this->simJoints.resize(this->jointNames.size());
  // Get gazebo joint pointers to joints
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
  {
    /// \TODO: this assumes id's for joints are consecutive, starting with 0
    physics::JointPtr joint = this->model->GetJoint(this->jointNames[i]);
    if (joint)
    {
      // gzdbg << "setting gazebo joint [" << joint->GetName() << "]\n";
      this->simJoints[i] = new JointHelper();
      this->simJoints[i]->SetJoint(joint);
      this->simJoints[i]->SetJointName(this->jointNames[i]);
      this->simJoints[i]->simOnly = simOnly[i];
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

    // get joint name associated with this motor
    if (motorSDF->HasElement("clutch"))
    {
      this->motorInfos[id].clutch = motorSDF->Get<bool>("clutch");
    }
    else
    {
      this->motorInfos[id].clutch = false;
    }
    // gzdbg << "  joint [" << this->motorInfos[id].jointName << "]\n";

    // initialize effort differential multiplier for parent to 1.0,
    // subtract multiplier from this value for each child effort differential
    this->motorInfos[id].effortDifferentialMultiplier = 1.0;

    // this should return index of the joint in this->simJoints
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
      int j1 = this->simJoints.size();
      JointHelper *hj = new JointHelper();
      hj->SetJointName(this->motorInfos[id].jointName);
      this->simJoints.push_back(hj);
      this->motorInfos[id].index = j1;
    }

    // get differential joints from <effort_differential> blocks
    if (motorSDF->HasElement("effort_differential"))
    {
      sdf::ElementPtr effortDifferentialSDF =
        motorSDF->GetElement("effort_differential");
      while (effortDifferentialSDF)
      {
        // get joint, offset and multiplier1 and multiplier2
        sdf::ElementPtr jointSDF = effortDifferentialSDF->GetElement("joint");
        sdf::ElementPtr multiplierSDF =
          effortDifferentialSDF->GetElement("multiplier");
        MotorInfo::EffortDifferential e;

        // find index in this->simJoints that matches
        // jointSDF->Get<std::string>();
        e.index = -1;
        for (unsigned int k = 0; k < this->jointNames.size(); ++k)
        {
          if (this->jointNames[k] == jointSDF->Get<std::string>())
          {
            e.index = k;
            break;
          }
        }
        if (e.index == -1)
        {
          gzwarn << "failed to find joint [" << jointSDF->Get<std::string>()
                 << "] for effort_differential, this"
                 << " joint will not be controlled.\n";
        }

        e.multiplier = multiplierSDF->Get<double>();
        this->motorInfos[id].effortDifferentials.push_back(e);

        // subtrace effort multiplier from main actuator torque
        this->motorInfos[id].effortDifferentialMultiplier -= e.multiplier;

        // parse spring
        if (effortDifferentialSDF->HasElement("spring"))
        {
          sdf::ElementPtr springSDF =
            effortDifferentialSDF->GetElement("spring");

          double preload = 0;
          if (springSDF->HasElement("preload"))
          {
            preload = springSDF->Get<double>("preload");
            gzdbg << "preload: " << preload << "\n";
          }
          double stiffness = 0;
          if (springSDF->HasElement("stiffness"))
          {
            stiffness = springSDF->Get<double>("stiffness");
            gzdbg << "stiffness: " << stiffness << "\n";
          }

          // assign values to gazebo joint directly
          if (e.index > -1)
          {
            double reference = 0;
            if (fabs(stiffness) > 0.0)
              reference = preload / stiffness;
            gzdbg << "reference " << reference << "\n";
            this->simJoints[e.index]->GetRealJoint()->SetStiffnessDamping(
              0, stiffness,
              this->simJoints[e.index]->GetRealJoint()->GetDamping(0),
              reference);
          }
        }

        effortDifferentialSDF =
          effortDifferentialSDF->GetNextElement("effort_differential");
      }
    }

    // get coupled joints from <gearbox> blocks
    if (motorSDF->HasElement("gearbox"))
    {
      sdf::ElementPtr gearboxSDF = motorSDF->GetElement("gearbox");
      while (gearboxSDF)
      {
        // get joint, offset and multiplier1 and multiplier2
        sdf::ElementPtr jointSDF = gearboxSDF->GetElement("joint");
        sdf::ElementPtr referenceJointSDF;
        if (gearboxSDF->HasElement("reference_joint"))
          referenceJointSDF = gearboxSDF->GetElement("reference_joint");
        sdf::ElementPtr offsetSDF = gearboxSDF->GetElement("offset");
        sdf::ElementPtr multiplier1SDF = gearboxSDF->GetElement("multiplier1");
        sdf::ElementPtr multiplier2SDF = gearboxSDF->GetElement("multiplier2");
        MotorInfo::GearBox g;

        // find index in this->simJoints that matches
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
        {
          gzwarn << "failed to find joint [" << jointSDF->Get<std::string>()
                 << "] for gearbox, this joint will not be controlled.\n";
        }

        g.referenceIndex = -1;
        if (referenceJointSDF)
        {
          for (unsigned int k = 0; k < this->jointNames.size(); ++k)
          {
            if (this->jointNames[k] == referenceJointSDF->Get<std::string>())
            {
              g.referenceIndex = k;
              break;
            }
          }
          if (g.referenceIndex == -1)
          {
            gzwarn << "failed to find reference joint ["
                   << referenceJointSDF->Get<std::string>()
                   << "] using parent motor joint for reference.\n";
          }
        }

        g.offset = offsetSDF->Get<double>();
        g.multiplier1 = multiplier1SDF->Get<double>();
        g.multiplier2 = multiplier2SDF->Get<double>();
        this->motorInfos[id].gearboxes.push_back(g);

        // parse spring and set joint spring
        if (gearboxSDF->HasElement("spring"))
        {
          sdf::ElementPtr springSDF =
            gearboxSDF->GetElement("spring");

          double preload = 0;
          if (springSDF->HasElement("preload"))
            preload = springSDF->Get<double>("preload");
          double stiffness = 0;
          if (springSDF->HasElement("stiffness"))
            stiffness = springSDF->Get<double>("stiffness");

          // assign values to gazebo joint directly
          if (g.index > -1)
          {
            double reference = 0;
            if (fabs(stiffness) > 0.0)
              reference = preload / stiffness;
            this->simJoints[g.index]->GetRealJoint()->SetStiffnessDamping(
             0, stiffness,
             this->simJoints[g.index]->GetRealJoint()->GetDamping(0),
             reference);
          }
        }

        gearboxSDF = gearboxSDF->GetNextElement("gearbox");
      }
    }

    // get next sdf
    motorSDF = motorSDF->GetNextElement("motor");
  }

  // Get pid gains and insert into pid
  this->pids.resize(this->simJoints.size());
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

  // SET MAX/MIN HAPTIX JOINT TORQUE LIMIT BASED ON
  //   - parent <motor_torque> and differential joint <multiplier>
  //   - parent <motor_torque> and gearbox <multiplier[1|2]>
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    int m = this->motorInfos[i].index;
    // gzdbg << m << " : " << this->simJointCommands[m].ref_pos << "\n";
    double jointTorque = this->motorInfos[i].gearRatio *
                         this->motorInfos[i].motorTorque;
    this->pids[m].SetCmdMax(jointTorque);
    this->pids[m].SetCmdMin(-jointTorque);

    /// also set joint effort limit directly, gazebo pid limits
    /// broken (see gazebo issue #1534)
    this->simJoints[m]->SetEffortLimit(0, jointTorque);

    /// \TODO: contemplate about using Joint::SetEffortLimit()
    /// instead of PID::SetCmdMax() and PID::SetCmdMin()

    /// parse EffortDifferentials to set effort limits
    /// \TODO: note: power distribution is not accurate.
    /// we should distribute the power between a effort differential
    /// with its children joints, which are currently PID position
    /// controlled. If we switch to physics gearbox joint constraint
    /// effort distribution will be automatically enforced.
    for (unsigned int j = 0;
         j < this->motorInfos[i].effortDifferentials.size(); ++j)
    {
      int n = this->motorInfos[i].effortDifferentials[j].index;
      double differentialJointTorque = jointTorque *
        this->motorInfos[i].effortDifferentials[j].multiplier;

      /// set joint effort limit
      this->simJoints[n]->SetEffortLimit(0, differentialJointTorque);

      // gzdbg << "   differential motor torque [" << n
      //       << "] : " << differentialJointTorque << "\n";
    }

    // parse gearboxes to
    // set torque command limits through <gearbox> coupling params.
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
    {
      int n = this->motorInfos[i].gearboxes[j].index;
      // Use the maximum of multiplier1 and multiplier2
      // for bounding joint torque command.
      const double minMultiplierLimit = 1.0;
      double minMultiplier = std::max(minMultiplierLimit, std::min(
        fabs(this->motorInfos[i].gearboxes[j].multiplier1),
        fabs(this->motorInfos[i].gearboxes[j].multiplier2)));
      // if a joint is geared with a multiplier,
      // it should have higher torque if multiplier < 1 (mechanical reduction)
      // it should have lower torque if multiplier > 1
      double maxJointTorque = jointTorque / minMultiplier;
      double minJointTorque = -jointTorque / minMultiplier;
      this->pids[n].SetCmdMax(maxJointTorque);
      this->pids[n].SetCmdMin(minJointTorque);

      double maxAbsJointTorque = std::max(
        maxJointTorque, minJointTorque);
      /// also set joint effort limit directly, gazebo pid limits
      /// broken (see gazebo issue #1534)
      this->simJoints[n]->SetEffortLimit(0, maxAbsJointTorque);

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
      std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (contactSensor)
    {
      ContactSensorInfo info;
      info.sensor = sensor;

      // initialize timestamp to a negative value
      info.timestamp = -1;

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
        std::dynamic_pointer_cast<sensors::ImuSensor>
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
  while (grasp)
  {
    std::string name;
    grasp->GetAttribute("name")->Get(name);
    std::vector<GraspPoint> graspPoints;
    if (grasp->HasElement("point"))
    {
      sdf::ElementPtr point = grasp->GetElement("point");
      while (point)
      {
        std::string graspInputsBuffer;
        point->GetElement("inputs")->GetValue()->Get(graspInputsBuffer);
        std::istringstream issInputs(graspInputsBuffer);
        std::vector<std::string> tokensInputs{
          std::istream_iterator<std::string>{issInputs},
          std::istream_iterator<std::string>{}};

        std::string graspMotorsBuffer;
        point->GetElement("motors")->GetValue()->Get(graspMotorsBuffer);
        std::istringstream issMotors(graspMotorsBuffer);
        std::vector<std::string> tokensMotors{
          std::istream_iterator<std::string>{issMotors},
          std::istream_iterator<std::string>{}};

        if (tokensInputs.size() == tokensMotors.size())
        {
          GraspPoint p;
          for (unsigned int i = 0; i < tokensInputs.size(); ++i)
          {
            // for old style trajectory, target input is 1
            p.inputs.push_back(stof(tokensInputs[i]));
            p.motors.push_back(stof(tokensMotors[i]));
          }
          graspPoints.push_back(p);
        }
        else
        {
          gzerr << "<inputs> vector must equal length of <motors>\n";
          gzerr << "grasp name=" << name << "\n";
          gzerr << "<inputs> len=" << tokensInputs.size() << "\n";
          gzerr << "<motors> len=" << tokensMotors.size() << "\n";
        }
        point = point->GetNextElement("point");
      }
    }
    else
    {
      // old style specification with just a single vector of final values
      // e.g.
      //   <grasp name="pinch">0 0 1 1 1</grasp>
      // we can set this as single point with inputs being 1's
      std::string graspBuffer;
      grasp->GetValue()->Get(graspBuffer);
      std::istringstream iss(graspBuffer);
      std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                      std::istream_iterator<std::string>{}};
      GraspPoint p;
      for (unsigned int i = 0; i < tokens.size(); ++i)
      {
        // for old style trajectory, target input is 1
        p.inputs.push_back(1.0);
        p.motors.push_back(stof(tokens[i]));
      }
      graspPoints.push_back(p);
    }
    this->grasps[name] = graspPoints;
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
    this->robotCommand.add_ref_vel(0.0);
    this->robotCommand.add_ref_vel_max(0.0);

    // default position and velocity gains
    this->robotCommand.add_gain_pos(1.0);
    this->robotCommand.add_gain_vel(0.0);
  }
  // initialize to control mode, not gain mode
  this->robotCommand.set_ref_pos_enabled(true);
  this->robotCommand.set_ref_vel_enabled(false);
  this->robotCommand.set_ref_vel_max_enabled(false);
  this->robotCommand.set_gain_pos_enabled(false);
  this->robotCommand.set_gain_vel_enabled(false);

  for (unsigned int i = 0; i < this->simJoints.size(); ++i)
  {
    if (this->simJoints[i]->HasJoint())
    {
      if (!this->simJoints[i]->simOnly)
      {
        this->robotState.add_joint_pos(0);
        this->robotState.add_joint_vel(0);
      }

      // internal command of all joints controller here (not just motors)
      SimRobotCommand c;
      c.ref_pos = 0.0;
      c.ref_vel = 0.0;
      c.ref_vel_max = 0.0;
      c.gain_pos = 1.0;
      c.gain_vel = 0.0;
      this->simJointCommands.push_back(c);
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

  // save joint limits for simulation clutch via joint limit pinching
  this->clutchEngaged.resize(this->simJoints.size());
  this->simJointUpperLimits.resize(this->simJoints.size());
  this->simJointLowerLimits.resize(this->simJoints.size());
  for (unsigned int i = 0; i < this->simJoints.size(); ++i)
  {
    if (this->simJoints[i]->HasJoint())
    {
      this->simJointLowerLimits[i] =
        this->simJoints[i]->GetRealJoint()->GetLowerLimit(0).Radian();
      this->simJointUpperLimits[i] =
        this->simJoints[i]->GetRealJoint()->GetUpperLimit(0).Radian();
      this->clutchEngaged[i] = 0;
    }
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::Reset()
{
  this->keyboardPose = this->initialBaseLinkPose;
  this->staleKeyboardPose = true;
  this->targetBaseLinkPose = this->initialBaseLinkPose;

  std::vector<SimRobotCommand>::iterator iter;
  for (iter = this->simJointCommands.begin();
      iter != this->simJointCommands.end(); ++iter)
  {
    iter->ref_pos = 0.0;
    iter->ref_vel = 0.0;
    iter->ref_vel_max = 0.0;
  }

  this->targetHandDist = 0;
  this->previousHandDist = 0;
}

/////////////////////////////////////////////////
// Allow users to set model pose directly
void HaptixControlPlugin::SetWorldPose(const msgs::Pose &_pose)
{
  boost::mutex::scoped_lock lock(this->baseLinkMutex);
  math::Pose inputPose(msgs::ConvertIgn(_pose));
  this->model->SetWorldPose(inputPose);
  this->targetBaseLinkPose = this->baseLink->GetRelativePose() + inputPose;

  // complete hack
  this->targetHandDist = 0;
  this->previousHandDist = 0;
}

/////////////////////////////////////////////////
// Open keyboard commands
void HaptixControlPlugin::SetKeyboardPose(const msgs::Pose &_pose)
{
  math::Pose inputPose(msgs::ConvertIgn(_pose));

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
  if (this->ignNode.Subscribe("/haptix/arm_pose_inc",
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
    posRate = msgs::ConvertIgn(joy.translation());
  }

  if (joy.has_rotation())
  {
    rotRate = msgs::ConvertIgn(joy.rotation());
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
    const double posScale = 1.5;
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

          gazebo::msgs::Set(&this->joyMsg, targetCameraPose.Ign());
          this->viewpointJoyPub->Publish(this->joyMsg);
        }
      }

      int thumbId = 2;
      math::Pose thumbSensorPose;
      if (thumbId < numPoses)
      {
        thumbSensorPose = this->convertPolhemusToPose(poses[thumbId]);
      }

      int fingersId = 3;
      math::Pose fingersSensorPose;
      if (fingersId < numPoses)
      {
        fingersSensorPose = this->convertPolhemusToPose(poses[fingersId]);
      }

      if (thumbSensorPose != math::Pose::Zero &&
          fingersSensorPose != math::Pose::Zero)
      {
        // distance between fingers
        double dist = (fingersSensorPose.pos - thumbSensorPose.pos).GetLength();
        // subtract the distance in which hand considered completely closed
        dist -= 0.04;
        if (dist < 0)
        {
          dist = 1e-3;
        }
        if (this->pauseTracking)
        {
          // calibration mode, update offset which represents hand fully open
          this->sourceDistHandOffset = dist / (1 - this->previousHandDist);
        }
        else
        {
          // Normalized distance: 0 fully open, 1 fully closed
          if (dist > this->sourceDistHandOffset)
          {
            dist = this->sourceDistHandOffset;
          }
          dist = this->polhemusGraspFilter.Process(dist);
          this->targetHandDist = 1 - dist / this->sourceDistHandOffset;
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
// DEMO hard coded
void HaptixControlPlugin::OnArrange(ConstGzStringPtr &_arrangement)
{
  this->arrangement = _arrangement->data();
  if (this->arrangement == "pyramid")
  {
    // demo hardcoded for luke hand
    this->currentPolhemusGrasp = "Chuck";
  }
  else if (this->arrangement == "hanoi")
  {
    // demo hardcoded for luke hand
    this->currentPolhemusGrasp = "FinePinchOpen";
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::UpdateBaseLink(double _dt)
{
  math::Pose pose;
  double dist;
  {
    boost::mutex::scoped_lock lock(this->baseLinkMutex);
    pose = this->targetBaseLinkPose;
    dist = this->targetHandDist;
    this->previousHandDist = dist;
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

/* old code hack for demo
  // limit wrench values for hanoi case
  double maxForce = 40;
  double maxTorque = 80;
  if (this->arrangement == "hanoi" || this->arrangement == "")
  {
    maxForce = 13.0;
    maxTorque = 30.0;
  }

  this->wrench.force.x =
    gazebo::math::clamp(this->posPid.Update(errorPos.x, _dt),
      -maxForce, maxForce);
  this->wrench.force.y =
    gazebo::math::clamp(this->posPid.Update(errorPos.y, _dt),
      -maxForce, maxForce);
  this->wrench.force.z =
    gazebo::math::clamp(this->posPid.Update(errorPos.z, _dt),
      -maxForce, maxForce);
  this->wrench.torque.x =
    gazebo::math::clamp(this->rotPid.Update(errorRot.x, _dt),
      -maxTorque, maxTorque);
  this->wrench.torque.y =
    gazebo::math::clamp(this->rotPid.Update(errorRot.y, _dt),
      -maxTorque, maxTorque);
  this->wrench.torque.z =
    gazebo::math::clamp(this->rotPid.Update(errorRot.z, _dt),
      -maxTorque, maxTorque);

  //std::cout << "Apply wrench to arm: " << this->wrench.force << ", " << this->wrench.torque << std::endl;
*/

  this->baseLink->SetForce(this->wrench.force);
  this->baseLink->SetTorque(this->wrench.torque);
  // std::cout << "current pose: " << baseLinkPose << std::endl;
  // std::cout << "target pose: " << pose << std::endl;
  // std::cout << "wrench pos: " << this->wrench.force
  //           << " rot: " << this->wrench.torque << std::endl;

  // DEMO hardcode grasp call
  // This is probably a horrible way and place to be doing this, especially
  // since I had to turn off a mutex :)
  haptix::comm::msgs::hxGrasp graspTmp;
  haptix::comm::msgs::hxGrasp::hxGraspValue* gv = graspTmp.add_grasps();
  gv->set_grasp_name(currentPolhemusGrasp);
  gv->set_grasp_value(dist);
  haptix::comm::msgs::hxCommand resp;
  bool result;
  this->HaptixGraspCallback(graspTmp, resp, result);
  if (!result)
  {
    gzerr << "ERROR: HaptixGraspCallback could not call grasp service" << std::endl;
  }
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

    // get motor joint target commands from motor ref_pos commands
    if (this->graspMode)
    {
      // If we're in grasp mode, then take commands from elsewhere
      if (this->robotCommand.ref_pos_enabled())
      {
        /// \TODO: check if gearRatio = 0
        this->simJointCommands[m].ref_pos =
          (this->graspPositions[i] + this->motorInfos[i].encoderOffset)
          / this->motorInfos[i].gearRatio;
      }
      if (this->robotCommand.ref_vel_enabled())
      {
        this->simJointCommands[m].ref_vel = 0.0;
      }
      if (this->robotCommand.ref_vel_max_enabled())
      {
        this->simJointCommands[m].ref_vel_max = 0.0;
      }
    }
    else
    {
      // command joint position targets for corresponding motor joints
      if (this->robotCommand.ref_pos_enabled())
      {
        // convert motor position to joint position
        this->ConvertMotorPositionToJointPosition(
          this->motorInfos[i], this->robotCommand.ref_pos(i),
          this->simJointCommands[m].ref_pos);
      }
      if (this->robotCommand.ref_vel_enabled())
      {
        // convert motor velocity to joint velocity
        this->ConvertMotorVelocityToJointVelocity(
          this->motorInfos[i], this->robotCommand.ref_vel(i),
          this->simJointCommands[m].ref_vel);
      }
      if (this->robotCommand.ref_vel_max_enabled())
      {
        // convert motor velocity to joint velocity
        this->ConvertMotorVelocityToJointVelocity(
          this->motorInfos[i], this->robotCommand.ref_vel_max(i),
          this->simJointCommands[m].ref_vel_max);
      }
    }
    if (this->robotCommand.gain_pos_enabled())
      this->simJointCommands[m].gain_pos = this->robotCommand.gain_pos(i);
    if (this->robotCommand.gain_vel_enabled())
      this->simJointCommands[m].gain_vel = this->robotCommand.gain_vel(i);

    // Use motor commands to set set joint command
    // based on coupling specified in <gearbox> params.

    // First, find minimum offset from all gearboxes:
    double minOffset = 1e16;
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
      minOffset = std::min(minOffset, this->motorInfos[i].gearboxes[j].offset);

    // APPLY JOINT GEARBOX RATIO TO GET COUPLED JOINTS TARGET FROM
    // MOTOR JOINT COMMANDS.
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
    {
      int n = this->motorInfos[i].gearboxes[j].index;
      // gzdbg << " " << n
      //       << " : " << this->simJointCommands[n].ref_pos
      //       << " : " << this->robotCommand.ref_pos_enabled() << "\n";

      // See transmission specification in issue #60,
      // If motor angle commanded is less than offset
      // use multiplier1, otherwise use multiplier2
      if (this->simJointCommands[m].ref_pos < minOffset)
      {
        if (this->robotCommand.ref_pos_enabled())
        {
          // use reference joint if specified
          if (this->motorInfos[i].gearboxes[j].referenceIndex >= 0)
          {
            m = this->motorInfos[i].gearboxes[j].referenceIndex;
            this->simJointCommands[n].ref_pos =
              this->simJoints[m]->GetAngle(0).Radian()
              * this->motorInfos[i].gearboxes[j].multiplier1;
          }
          else
          {
            this->simJointCommands[n].ref_pos =
              this->simJointCommands[m].ref_pos
              * this->motorInfos[i].gearboxes[j].multiplier1;
          }
        }
        if (this->robotCommand.ref_vel_enabled())
        {
          if (this->motorInfos[i].gearboxes[j].referenceIndex >= 0)
          {
            m = this->motorInfos[i].gearboxes[j].referenceIndex;
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier1, 0.0))
            {
              this->simJointCommands[n].ref_vel = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel =
                this->simJoints[m]->GetVelocity(0)
                / this->motorInfos[i].gearboxes[j].multiplier1;
            }
          }
          else
          {
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier1, 0.0))
            {
              this->simJointCommands[n].ref_vel = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel =
                this->simJointCommands[m].ref_vel
                / this->motorInfos[i].gearboxes[j].multiplier1;
            }
          }
        }
        if (this->robotCommand.ref_vel_max_enabled())
        {
          if (this->motorInfos[i].gearboxes[j].referenceIndex >= 0)
          {
            m = this->motorInfos[i].gearboxes[j].referenceIndex;
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier1, 0.0))
            {
              this->simJointCommands[n].ref_vel_max = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel_max =
                this->simJoints[m]->GetVelocity(0)
                / this->motorInfos[i].gearboxes[j].multiplier1;
            }
          }
          else
          {
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier1, 0.0))
            {
              this->simJointCommands[n].ref_vel_max = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel_max =
                this->simJointCommands[m].ref_vel_max
                / this->motorInfos[i].gearboxes[j].multiplier1;
            }
          }
        }
      }
      else
      {
        if (this->robotCommand.ref_pos_enabled())
        {
          if (this->motorInfos[i].gearboxes[j].referenceIndex >= 0)
          {
            m = this->motorInfos[i].gearboxes[j].referenceIndex;
            this->simJointCommands[n].ref_pos =
              (this->simJoints[m]->GetAngle(0).Radian() -
               this->motorInfos[i].gearboxes[j].offset)
              * this->motorInfos[i].gearboxes[j].multiplier2
              + this->motorInfos[i].gearboxes[j].offset
              * this->motorInfos[i].gearboxes[j].multiplier2;
          }
          else
          {
            this->simJointCommands[n].ref_pos =
              (this->simJointCommands[m].ref_pos -
               this->motorInfos[i].gearboxes[j].offset)
              * this->motorInfos[i].gearboxes[j].multiplier2
              + this->motorInfos[i].gearboxes[j].offset
              * this->motorInfos[i].gearboxes[j].multiplier2;
          }
        }
        if (this->robotCommand.ref_vel_enabled())
        {
          if (this->motorInfos[i].gearboxes[j].referenceIndex >= 0)
          {
            m = this->motorInfos[i].gearboxes[j].referenceIndex;
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier2, 0.0))
            {
              this->simJointCommands[n].ref_vel = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel =
                this->simJoints[m]->GetVelocity(0)
                / this->motorInfos[i].gearboxes[j].multiplier2;
            }
          }
          else
          {
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier2, 0.0))
            {
              this->simJointCommands[n].ref_vel = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel =
                this->simJointCommands[m].ref_vel
                / this->motorInfos[i].gearboxes[j].multiplier2;
            }
          }
        }
        if (this->robotCommand.ref_vel_max_enabled())
        {
          if (this->motorInfos[i].gearboxes[j].referenceIndex >= 0)
          {
            m = this->motorInfos[i].gearboxes[j].referenceIndex;
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier2, 0.0))
            {
              this->simJointCommands[n].ref_vel_max = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel_max =
                this->simJoints[m]->GetVelocity(0)
                / this->motorInfos[i].gearboxes[j].multiplier2;
            }
          }
          else
          {
            if (math::equal(this->motorInfos[i].gearboxes[j].multiplier2, 0.0))
            {
              this->simJointCommands[n].ref_vel_max = 0.0;
            }
            else
            {
              this->simJointCommands[n].ref_vel_max =
                this->simJointCommands[m].ref_vel_max
                / this->motorInfos[i].gearboxes[j].multiplier2;
            }
          }
        }
      }
      // set gains
      if (this->robotCommand.gain_pos_enabled())
        this->simJointCommands[n].gain_pos = this->robotCommand.gain_pos(i);
      if (this->robotCommand.gain_vel_enabled())
        this->simJointCommands[n].gain_vel = this->robotCommand.gain_vel(i);
    }
  }
}

/////////////////////////////////////////////////
double HaptixControlPlugin::ApplySimJointPositionPIDCommand(int _index,
  double _dt)
{
  // get joint positions and velocities
  double position = this->simJoints[_index]->GetAngle(0).Radian();
  double velocity = this->simJoints[_index]->GetVelocity(0);

  // compute target joint position and velocity error in gazebo
  double errorPos = position - this->simJointCommands[_index].ref_pos;
  double errorVel = velocity - this->simJointCommands[_index].ref_vel;
  double errorVelMax = velocity - this->simJointCommands[_index].ref_vel_max;

  // compute overall error
  double error = this->simJointCommands[_index].gain_pos * errorPos
               + this->simJointCommands[_index].gain_vel * errorVel
               + this->simJointCommands[_index].gain_vel * errorVelMax;

  // compute force needed
  double force = this->pids[_index].Update(error, _dt);

  return force;
}

/////////////////////////////////////////////////
void HaptixControlPlugin::ApplyJointForce(int _index, double _force)
{
  // command joint effort
  if (!this->simJoints[_index]->SetForce(0, _force))
  {
    // not a real gazebo joint, set target directly
    this->simJoints[_index]->SetPosition(
      this->simJointCommands[_index].ref_pos);

    /// \TODO: something about velocity commands
    // this->simJoints[_index]->SetVelocity(
    //   this->simJointCommands[_index].ref_vel);
    // this->simJoints[_index]->SetVelocity(
    //   this->simJointCommands[_index].ref_vel_max);
    /// \TODO: for issue #86 motor velocity will be zero
    /// unless we:  1) compute torque from transmissioned joints, or
    /// 2) implement actual motor joint dynamics and servo the joint.
    /// For example, 1) could be:

    /// \TODO: for issue #86: torque for fake joint will always be zero
    /// unless we:  1) compute torque from transmissioned joints, or
    /// 2) implement actual motor joint dynamics and servo the joint.
    /// For example, 1) could be:
    // double force2 = computed from gearboxed joints
    // this->simJoints[i]->SetForce(0, force2);
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::UpdateHandControl(double _dt)
{
  // clutch are optional for motor actuated joints
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    if (this->motorInfos[i].clutch)
    {
      int m = this->motorInfos[i].index;
      double pe, ie, de;
      this->pids[m].GetErrors(pe, ie, de);
      double cmd = this->pids[m].GetCmd();

      if (this->simJoints[m]->HasJoint())
      {
        double pos = this->simJoints[m]->GetRealJoint()->GetAngle(0).Radian();

        // 2 degrees deadband for pos violation detection
        const double tol = 2.0/180.0*M_PI;

        // check if lolimit needs to be engaged
        bool handPushedOpen = (pos < this->simJointCommands[m].ref_pos - tol);
        if (handPushedOpen)
        {
          if (this->clutchEngaged[m] != -1)
          {
            gzdbg << "engage lo ["
                  << this->simJoints[m]->GetRealJoint()->GetName()
                  << "] m: " << m
                  << " pos: " << pos
                  << " ref_pos: " << this->simJointCommands[m].ref_pos
                  << " cmd: " << cmd
                  << " pe: " << pe
                  << " ie: " << ie
                  << " de: " << de
                  << " lo: " << this->simJointLowerLimits[m]
                  << " hi: " << this->simJointUpperLimits[m]
                  << "\n";
            double posClamp = math::clamp(pos,
                  this->simJointLowerLimits[m], this->simJointUpperLimits[m]);
            // lo clutch enabled
            this->simJoints[m]->GetRealJoint()->SetLowStop(0, posClamp);
            // hi clutch disabled
            this->simJoints[m]->GetRealJoint()->SetHighStop(0,
              this->simJointUpperLimits[m]);
            this->clutchEngaged[m] = -1;
          }
        }

        // check if we should disengage lo

        // debug:
        // bool loClutchEngaged =
        //   (this->simJoints[m]->GetRealJoint()->GetParam("lo_stop", 0) >
        //    this->simJointLowerLimits[m]);
        // if (this->simJoints[m]->GetRealJoint()->GetName() ==
        //     "index_proximal_flex" && loClutchEngaged)
        // {
        //   gzerr << this->simJoints[m]->GetRealJoint()->GetParam("lo_stop", 0)
        //         << "\n";
        // }
        // if (loClutchEngaged && !handPushedOpen)
        if (this->clutchEngaged[m] == -1 && !handPushedOpen)
        {
            gzdbg << "disengage lo ["
                  << this->simJoints[m]->GetRealJoint()->GetName()
                  << "] m: " << m
                  << " pos: " << pos
                  << " ref_pos: " << this->simJointCommands[m].ref_pos
                  << " lo_limit: " << this->simJointLowerLimits[m]
                  << " cmd: " << cmd
                  << " pe: " << pe
                  << " ie: " << ie
                  << " de: " << de
                  << " lo: " << this->simJointLowerLimits[m]
                  << " hi: " << this->simJointUpperLimits[m]
                  << "\n";
          // lo clutch disabled
          this->simJoints[m]->GetRealJoint()->SetLowStop(0,
            this->simJointLowerLimits[m]);
          this->clutchEngaged[m] = 0;
        }

        // check if hilimit needs to be engaged
        bool handPushedClose = (pos > this->simJointCommands[m].ref_pos + tol);
        if (handPushedClose)
        {
          if (this->clutchEngaged[m] != 1)
          {
            gzdbg << "engage hi ["
                  << this->simJoints[m]->GetRealJoint()->GetName()
                  << "] m: " << m
                  << " pos: " << pos
                  << " ref_pos: " << this->simJointCommands[m].ref_pos
                  << " cmd: " << cmd
                  << " pe: " << pe
                  << " ie: " << ie
                  << " de: " << de
                  << " lo: " << this->simJointLowerLimits[m]
                  << " hi: " << this->simJointUpperLimits[m]
                  << "\n";
            double posClamp = math::clamp(pos,
                  this->simJointLowerLimits[m], this->simJointUpperLimits[m]);
            // hi clutch engaged
            this->simJoints[m]->GetRealJoint()->SetHighStop(0, posClamp);
            // lo clutch disabled
            this->simJoints[m]->GetRealJoint()->SetLowStop(0,
              this->simJointLowerLimits[m]);
            this->clutchEngaged[m] = 1;
          }
        }

        // check if we should disengage hi
        if (this->clutchEngaged[m] == 1 && !handPushedClose)
        {
            gzdbg << "disengage hi ["
                  << this->simJoints[m]->GetRealJoint()->GetName()
                  << "] m: " << m
                  << " pos: " << pos
                  << " ref_pos: " << this->simJointCommands[m].ref_pos
                  << " cmd: " << cmd
                  << " pe: " << pe
                  << " ie: " << ie
                  << " de: " << de
                  << " lo: " << this->simJointLowerLimits[m]
                  << " hi: " << this->simJointUpperLimits[m]
                  << "\n";
          // hi clutch disabled
          this->simJoints[m]->GetRealJoint()->SetHighStop(0,
            this->simJointUpperLimits[m]);
          this->clutchEngaged[m] = 0;
        }
      }
    }
  }

  // command all joints by walking through all the motors
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    // set force on actuator joint
    int m = this->motorInfos[i].index;
    double actForce = this->ApplySimJointPositionPIDCommand(m, _dt);
    this->ApplyJointForce(m,
      this->motorInfos[i].effortDifferentialMultiplier * actForce);
    double actForceFiltered = this->torqueFilter.Process(actForce);
    // gzerr << actForce << " : " << actForceFiltered << "\n";

    // set force on effort differential joints
    for (unsigned int j = 0;
         j < this->motorInfos[i].effortDifferentials.size(); ++j)
    {
      // test: do not apply torque if torque is small
      // const double torqueTol = 0.1;
      // if (fabs(actForceFiltered) > torqueTol)
      {
        int n = this->motorInfos[i].effortDifferentials[j].index;
        double multiplier =
          this->motorInfos[i].effortDifferentials[j].multiplier;
        this->ApplyJointForce(n, multiplier*actForceFiltered);
        // this->ApplyJointForce(n, multiplier*actForce);
        // gzerr << m << " : " << n
        //       << " : " << multiplier << " : " << actForce << "\n";
      }
    }

    // set force on geared joints
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
    {
      int n = this->motorInfos[i].gearboxes[j].index;
      double force = this->ApplySimJointPositionPIDCommand(n, _dt);
      this->ApplyJointForce(n, force);
    }
  }
}

/////////////////////////////////////////////////
void HaptixControlPlugin::OnContactSensorUpdate(int _i)
{
  boost::mutex::scoped_lock lock(this->updateMutex);

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
    std::dynamic_pointer_cast<sensors::ContactSensor>(
    this->contactSensorInfos[_i].sensor);

  if (!contactSensor)
  {
    gzerr << "sensor [" << this->contactSensorInfos[_i].sensor->Name()
          << "] is not a ContactSensor.\n";
    return;
  }
  msgs::Contacts contacts = contactSensor->Contacts();
  // contact sensor report contact between pairs of bodies

  // we must start at the end of the contacts.contact() array,
  // go backwards and aggregate all the wrenches that have the same
  // timestamp and same link name
  int numContacts = contacts.contact().size();

  // initial timestamp with a negative value, initialize with the timestap of
  // last one in the buffer array (contacts.contact()), then
  // only aggregate the ones that have the same timestamp.
  double timestamp = -1;

  // clear contact info
  if (numContacts > 0)
  {
    this->contactSensorInfos[_i].contactForce = math::Vector3();
    this->contactSensorInfos[_i].contactTorque = math::Vector3();
  }

  while (numContacts > 0)
  {
    --numContacts;
    msgs::Contact contact = contacts.contact(numContacts);

    // if (contact.wrench().size() > 0)
    //   gzerr << "  name " << contactSensor->GetName() << "\n";

    // each contact can have multiple wrenches
    for (int k = 0; k < contact.wrench().size(); ++k)
    {
      msgs::JointWrench wrenchMsg = contact.wrench(k);

      double t = contact.time().sec() + 1.0e-9*contact.time().nsec();
      if (timestamp < 0)
      {
        // timestamp uninitialized
        timestamp = t;
      }

      if (!math::equal(t, timestamp))
      {
        // got an older timestamp, stop the while loop
        numContacts = 0;
      }
      else
      {
        // gzerr << "    contacts [" << numContacts
        //       << "] t ["  << timestamp
        //       << "] wrench size ["  << contact.wrench().size()
        //       << "] body 1 ["  << wrenchMsg.body_1_name()
        //       << "] body 2 ["  << wrenchMsg.body_2_name()
        //       << "]\n";

        // sum up all wrenches from body_1 or body_2
        // check with contact corresponds to the arm and which to the arm
        // compare body_1_name and body_2_name with model name

        gazebo::msgs::Wrench w;
        gazebo::physics::CollisionPtr collision;
        gazebo::physics::LinkPtr link;
        if (strncmp(this->model->GetName().c_str(),
                    wrenchMsg.body_1_name().c_str(),
                    this->model->GetName().size()) == 0)
        {
          w = wrenchMsg.body_1_wrench();
          collision = this->model->GetChildCollision(wrenchMsg.body_1_name());
        }
        else if (strncmp(this->model->GetName().c_str(),
                         wrenchMsg.body_2_name().c_str(),
                         this->model->GetName().size()) == 0)
        {
          w = wrenchMsg.body_2_wrench();
          collision = this->model->GetChildCollision(wrenchMsg.body_2_name());
        }
        else
        {
          gzerr << "collision name does not match model name. This should "
                 << "never happen." << std::endl;
          return;
        }

        link = boost::dynamic_pointer_cast<gazebo::physics::Link>(
          collision->GetParent());
        // gzerr << link->GetName() << "\n";
        ignition::math::Pose3<double> fPose = link->GetWorldPose().Ign();
        ignition::math::Vector3d fPos = fPose.Pos();
        ignition::math::Quaternion<double> fRot = fPose.Rot();

        // force and torque in Link frame
        ignition::math::Vector3d force =
          gazebo::msgs::ConvertIgn(w.force());
        ignition::math::Vector3d torque =
          gazebo::msgs::ConvertIgn(w.torque());

        // force and torque in inertial frame at Link origin
        ignition::math::Vector3d forceI = fRot.RotateVector(force);
        ignition::math::Vector3d torqueI = fRot.RotateVector(torque);

        // position and normal in inertial frame
        ignition::math::Vector3d position =
          gazebo::msgs::ConvertIgn(contact.position(k));
        ignition::math::Vector3d normal =
          gazebo::msgs::ConvertIgn(contact.normal(k));

        // force moment arm in inertial frame
        ignition::math::Vector3d forceArm = position - fPos;

        // compute force at contact in inertial frame
        ignition::math::Vector3d forceAtContact =
          forceI + torqueI.Cross(forceArm);

        // compute normal force at contact in inertial frame
        // negative sign so force is possitive pushing down on the surface
        ignition::math::Vector3d fn = -forceAtContact.Dot(normal) * normal;

        // compute normal force at the point of contact
        this->contactSensorInfos[_i].contactForce += fn;

        // compute torsional friction at point of contact
        this->contactSensorInfos[_i].contactTorque +=
          torqueI.Dot(normal) * normal;

        // store time at which sensor was updated
        this->contactSensorInfos[_i].timestamp = timestamp;

        // if (contact.wrench().size() > 1)
        //   gzerr << "        sensor [" << _i << "] buffer [" << numContacts
        //         << "] contact [" << k << "] force ["
        //         << this->contactSensorInfos[_i].contactForce
        //         << "] sub-sum ["
        //         << this->contactSensorInfos[_i].contactForce.GetLength()
        //         << "]\n";
      }
    }
  }
  // gzerr << " sensor [" << _i
  //       << "] sum [" << this->contactSensorInfos[_i].contactForce
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
// convert joint position to motor position
// convert joint velocity to motor velocity
// convert joint torque to motor torque
void HaptixControlPlugin::ConvertJointDataToMotorData(
  const MotorInfo &_motorInfo,
  double &_motorPosition, double &_motorVelocity, double &_motorTorque)
{
  int m = _motorInfo.index;
  double jointPosition = this->simJoints[m]->GetAngle(0).Radian();
  double jointVelocity = this->simJoints[m]->GetVelocity(0);
  double jointTorque = this->simJoints[m]->GetForce(0);
  // convert joint angle and velocities into motor using gear_ratio
  _motorPosition = jointPosition * _motorInfo.gearRatio
    - _motorInfo.encoderOffset;
  _motorVelocity = jointVelocity / _motorInfo.gearRatio;
  _motorTorque = jointTorque / _motorInfo.gearRatio;
}

/////////////////////////////////////////////////
// convert motor position to joint position
void HaptixControlPlugin::ConvertMotorPositionToJointPosition(
  const MotorInfo &_motorInfo, const double _motorPosition,
  double &_jointPosition)
{
  _jointPosition = (_motorPosition + _motorInfo.encoderOffset)
    / _motorInfo.gearRatio;
}

/////////////////////////////////////////////////
// convert motor velocity to joint velocity
void HaptixControlPlugin::ConvertMotorVelocityToJointVelocity(
  const MotorInfo &_motorInfo, const double _motorVelocity,
  double &_jointVelocity)
{
  if (!math::equal(_motorInfo.gearRatio, 0.0))
  {
    _jointVelocity = _motorVelocity / _motorInfo.gearRatio;
  }
  else
  {
    gzwarn << "zero motor gear ratio, not right. Setting joint vel to 0.\n";
    _jointVelocity = 0;
  }
}

/////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::GetRobotStateFromSim()
{
  common::Time curTime = this->world->GetSimTime();

  // fill motor state from joint state
  double motorPosition, motorVelocity, motorTorque;
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    this->ConvertJointDataToMotorData(this->motorInfos[i],
      motorPosition, motorVelocity, motorTorque);
    // write to struct
    this->robotState.set_motor_pos(i, motorPosition);
    if (this->model->GetName() == "luke_hand_description")
    {
      // HACK: mimic special case when using luke hand
      this->robotState.set_motor_vel(i, 0);
      this->robotState.set_motor_torque(i, 0);
    }
    else
    {
      this->robotState.set_motor_vel(i, motorVelocity);
      this->robotState.set_motor_torque(i, motorTorque);
    }
  }

  // fill robot state joint_pos and joint_vel
  unsigned int count = 0;
  for (unsigned int i = 0; i < this->simJoints.size(); ++i)
  {
    if (this->simJoints[i]->HasJoint() && !this->simJoints[i]->simOnly)
    {
      this->robotState.set_joint_pos(count,
        this->simJoints[i]->GetAngle(0).Radian());
      if (this->model->GetName() == "luke_hand_description")
      {
        // HACK: mimic special case when using luke hand
        this->robotState.set_joint_vel(count, 0);
      }
      else
      {
        this->robotState.set_joint_vel(count,
          this->simJoints[i]->GetVelocity(0));
      }
      ++count;
    }
  }

  // copy contact forces
  // gzerr << "contactSensorInfos " << this->contactSensorInfos.size() << "\n";
  for (unsigned int i = 0; i < this->contactSensorInfos.size(); ++i)
  {
    // get summed force from contactSensorInfos
    double force = 0;
    const double timeout = 0.01;
    if (curTime.Double() - this->contactSensorInfos[i].timestamp < timeout)
      force = this->contactSensorInfos[i].contactForce.GetLength();
    // return summed force
    this->robotState.set_contact(i, force);
  }

  for (unsigned int i = 0; i < this->imuSensors.size(); ++i)
  {
    haptix::comm::msgs::imu *linacc =
        this->robotState.mutable_imu_linear_acc(i);
    math::Vector3 acc = this->imuSensors[i]->LinearAcceleration();
    math::Vector3 vel = this->imuSensors[i]->AngularVelocity();
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

  this->robotState.mutable_time_stamp()->set_sec(curTime.sec);
  this->robotState.mutable_time_stamp()->set_nsec(curTime.nsec);
}

/////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::GazeboUpdateStates()
{
{
  DIAG_TIMER_START("HaptixControlPlugin::GazeboUpdateStates");
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
      // gzdbg << 1.0/(curTime -
      //   this->lastSimTimeForControlThrottling).Double() << std::endl;

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
    this->lastSimTimeForControlThrottling = curTime;
  }
  DIAG_TIMER_STOP("HaptixControlPlugin::GazeboUpdateStates");

  std::vector<std::string> services;
  this->ignNode.ServiceList(services);

  if (std::find(services.begin(), services.end(), "/haptix/luke/Update") !=
        services.end())
  {
    // Forward the current command to the real hand.
    haptix::comm::msgs::hxCommand cmd;
    cmd.set_ref_pos_enabled(true);
    cmd.set_ref_vel_enabled(false);
    cmd.set_ref_vel_max_enabled(false);
    cmd.set_gain_pos_enabled(false);
    cmd.set_gain_vel_enabled(false);

    if (this->graspMode &&
        this->graspPositions.size() == this->motorInfos.size())
    {
      for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
        cmd.add_ref_pos(this->graspPositions[i]);
    }
    else
    {
      for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
        cmd.add_ref_pos(this->robotCommand.ref_pos(i));
    }
    haptix::comm::msgs::hxSensor rep;
    bool res;
    this->ignNode.Request("/haptix/luke/Update", cmd, 10, rep, res);
  }
}

  // demo hard code to send to CAN bus driver
  if (this->graspMode &&
      this->graspPositions.size() == this->motorInfos.size())
  {
    bool result;
    haptix::comm::msgs::hxCommand demoReq;
    haptix::comm::msgs::hxSensor demoRep;
    demoReq.set_ref_pos_enabled(true);
    demoReq.set_ref_vel_enabled(false);
    demoReq.set_ref_vel_max_enabled(false);
    demoReq.set_gain_pos_enabled(false);
    demoReq.set_gain_vel_enabled(false);
    for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
    {
      // gzerr << i << " : " << this->graspPositions[i] << "\n";
      demoReq.add_ref_pos(this->graspPositions[i]);
    }
    /*if (!this->ignNode.Request("/haptix/luke/Update",
                              demoReq,
                              30,
                              demoRep,
                              result) || !result)
    {
      gzerr << "Failed to call demo request" << std::endl;
    }*/
  }
  else
  {
    bool result;
    haptix::comm::msgs::hxCommand demoReq;
    haptix::comm::msgs::hxSensor demoRep;
    demoReq.set_ref_pos_enabled(true);
    demoReq.set_ref_vel_enabled(false);
    demoReq.set_ref_vel_max_enabled(false);
    demoReq.set_gain_pos_enabled(false);
    demoReq.set_gain_vel_enabled(false);
    for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
    {
      // gzerr << i << " : " << this->robotCommand.ref_pos(i) << "\n";
      demoReq.add_ref_pos(this->robotCommand.ref_pos(i));
    }
    if (!this->ignNode.Request("/haptix/luke/Update",
                              demoReq,
                              30,
                              demoRep,
                              result) || !result)
    {
      gzerr << "Failed to call demo request" << std::endl;
    }
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
      const haptix::comm::msgs::hxRobot &/*_req*/,
      haptix::comm::msgs::hxRobot &_rep, bool &_result)
{
  _rep.set_motor_count(this->motorInfos.size());
  _rep.set_joint_count(this->robotState.joint_pos().size());
  _rep.set_contact_sensor_count(this->contactSensorInfos.size());
  _rep.set_imu_count(this->imuSensors.size());

  for (unsigned int i = 0; i < this->simJoints.size(); ++i)
  {
    if (this->simJoints[i]->HasJoint() && !this->simJoints[i]->simOnly)
    {
      haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
      joint->set_minimum(this->simJoints[i]->GetLowerLimit(0).Radian());
      joint->set_maximum(this->simJoints[i]->GetUpperLimit(0).Radian());
    }
  }

  // RETURN JOINT INFO (motor max / min limits).
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    int m = this->motorInfos[i].index;
    haptix::comm::msgs::hxRobot::hxLimit *motor = _rep.add_motor_limit();

    /// \TODO for issue #86, compute joint limits for fake joints as well
    if (!this->simJoints[m]->HasJoint())
    {
      // fake joint, limit is not set in sdf, so they are +/-1e16
      // go through all gearboxes and compute a joint limit based
      // on joint limits of gearboxed joints.
      double motorMin = -1e16;
      double motorMax = 1e16;
      for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
      {
        int n = this->motorInfos[i].gearboxes[j].index;
        double hi = this->simJoints[n]->GetUpperLimit(0).Radian();
        double lo = this->simJoints[n]->GetLowerLimit(0).Radian();

        // which multiplier to use
        // See transmission specification in issue #60,
        // If motor angle commanded is less than offset
        // use multiplier1, otherwise use multiplier2
        // Assumption here is that offset is >= 0, otherwise
        // joint motion is discontinuous.

        // THE GOAL HERE IS TO COMPUTE MOTOR POSITION RANGE BASED
        // ON CHILD GEARBOXED JOINT LIMITS IN THE MODEL.
        // given:
        // motor_position = actuator_joint_position * gear_ratio - offset
        // the correct approach is to compute the range of motor positions
        // based on actuator joint position limits for the segment from:
        //   - lowest motor angle to encoder offset angle (using multiplier1)
        //   - encoder offset angle to highest motor angle (using multiplier2)

        /* improved transmission, work in progress
        // compute actuator joint position with limits using multiplier1
        double actuatorJoint1a =
          lo * this->motorInfos[i].gearboxes[j].multiplier1;
        double actuatorJoint1b =
          hi * this->motorInfos[i].gearboxes[j].multiplier1;
        // compute motor position limits based on actuator joint positions
        double motor1a =
          actuatorJoint1a * this->motorInfos[i].gearboxes[j].gearRatio
          - this->motorInfos[i].gearboxes[j].encoderOffset;
        double motor1b =
          actuatorJoint1b * this->motorInfos[i].gearboxes[j].gearRatio
          - this->motorInfos[i].gearboxes[j].encoderOffset;
        // take max/min of motor position limits
        double motor1Lo = std::min(motor1a, motor1b);
        double motor1Hi = std::max(motor1a, motor1b);
        // motor high limit is limited by the encoder offset
        motor1Hi = std::min(motor1Hi,
          this->motorInfos[i].gearboxes[j].encoderOffset);
        // take the segment
        // double m1Lo = lo * this->motorInfos[i].gearboxes[j].multiplier1;
        // double m1Hi = this->motorInfos[i].gearboxes[j].encoderOffset
        //   * this->motorInfos[i].gearboxes[j].multiplier1;
        */


        // using multiplier1:
        // note: this is wrong
        motorMin = std::max(motorMin,
          lo * this->motorInfos[i].gearboxes[j].multiplier1
             * this->motorInfos[i].gearRatio
             - this->motorInfos[i].encoderOffset);

        // use multiplier2 for computing upper limit
        // take the smallest of the max
        // note: this is wrong
        motorMax = std::min(motorMax,
          hi * this->motorInfos[i].gearboxes[j].multiplier2
             * this->motorInfos[i].gearRatio
             - this->motorInfos[i].encoderOffset);
        // gzdbg << " lo: " << lo
        //       << " hi: " << hi
        //       << " min: " << motorMin
        //       << " max: " << motorMax
        //       << "\n";
      }

      // gzdbg << m
      //       << " : " << motorMin
      //       << " : " << motorMax
      //       << " min: " << motorMin
      //       << " max: " << motorMax
      //       << " gr: " << this->motorInfos[i].gearRatio
      //       << "\n";
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
    }
    else
    {
      // there's a real joint with valid limits,
      // compute the motor limits from joint limits.
      double jointMin = this->simJoints[m]->GetLowerLimit(0).Radian();
      double jointMax = this->simJoints[m]->GetUpperLimit(0).Radian();
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
      // gzerr << m << " : " << motorMin << " : " << motorMax << "\n";
    }
  }

  _rep.set_update_rate(this->updateRate);

  _result = true;
}

//////////////////////////////////////////////////
/// using haptix-comm service callback
void HaptixControlPlugin::HaptixUpdateCallback(
      const haptix::comm::msgs::hxCommand &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  // Read the request parameters.
  // Debug output.
  /*std::cout << "Received a new command:" << std::endl;
  for (unsigned int i = 0; i < this->simJoints.size(); ++i)
  {
    std::cout << "\tMotor " << i << ":" << std::endl;
    std::cout << "\t\t" << _req.ref_pos(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel_max(i) << std::endl;
    std::cout << "\t\t" << _req.gain_pos(i) << std::endl;
    std::cout << "\t\t" << _req.gain_vel(i) << std::endl;
  }
  std::cout << "\tref_pos_enabled\t"
            << _req.ref_pos_enabled()     << std::endl;
  std::cout << "\tref_vel_enabled\t"
            << _req.ref_vel_enabled() << std::endl;
  std::cout << "\tref_vel_max_enabled\t"
            << _req.ref_vel_max_enabled() << std::endl;
  std::cout << "\tgain_pos_enabled\t"
            << _req.gain_pos_enabled()    << std::endl;
  std::cout << "\tgain_vel_enabled\t"
            << _req.gain_vel_enabled()    << std::endl;
  */

  this->robotCommand = _req;

  // for clutch timing
  this->robotCommandTime = this->world->GetSimTime();

  _rep.Clear();

  _rep = this->robotState;

  _result = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUserCameraPose(ConstPosePtr &_msg)
{
  boost::mutex::scoped_lock lock(this->userCameraPoseMessageMutex);
  this->userCameraPose = math::Pose(msgs::ConvertIgn(*_msg));
  this->userCameraPoseValid = true;
}

//////////////////////////////////////////////////
/// using ign-transport service, out of band from haptix_comm
void HaptixControlPlugin::HaptixGraspCallback(
      const haptix::comm::msgs::hxGrasp &_req,
      haptix::comm::msgs::hxCommand &_rep, bool &_result)
{
  // unlock for demo because HaptixGraspCallback is called
  // from UpdateBaseLink to set grasp control using polhemus fingers 
  // boost::mutex::scoped_lock lock(this->updateMutex);

  // for clutch timing
  this->robotCommandTime = this->world->GetSimTime();

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
  _rep.set_ref_vel_enabled(this->robotCommand.ref_vel_enabled());
  _rep.set_ref_vel_max_enabled(this->robotCommand.ref_vel_max_enabled());
  _rep.set_gain_pos_enabled(this->robotCommand.gain_pos_enabled());
  _rep.set_gain_vel_enabled(this->robotCommand.gain_vel_enabled());

  for (int i = 0; i < _req.grasps_size(); ++i)
  {
    std::string name = _req.grasps(i).grasp_name();
    std::map<std::string, std::vector<GraspPoint> >::const_iterator g =
      this->grasps.find(name);
    if (g != this->grasps.end())
    {
      float value = _req.grasps(i).grasp_value();
      if (value < 0.0)
        value = 0.0;
      if (value > 1.0)
        value = 1.0;
      for (unsigned int j = 0;
          j < this->graspPositions.size(); ++j)
      {
        // This superposition logic could use a lot of thought.  But it should
        // at least work for the case of a single type of grasp.
        std::vector<GraspPoint> points = g->second;
        // loop through points to see where the value falls in grasp trajectory
        float input = 0;
        float output = 0;
        float pos = 0;
        for (std::vector<GraspPoint>::iterator p = points.begin();
             p != points.end(); ++p)
        {
          float lastInput = input;
          float lastOutput = output;
          input = p->inputs[j];
          output = p->motors[j];
          // check to see if this value falls in this interval of input
          if (value > input)
          {
            // update output command
            pos = output;
            // go to next point
          }
          else
          {
            // update output command
            if (!math::equal(input, lastInput))
            {
              pos += (value - lastInput) / (input - lastInput) *
                     (output - lastOutput);
            }
            p = --points.end();
          }
        }
        // superimpose multiple _req.grasps together
        this->graspPositions[j] += pos / _req.grasps_size();
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
  this->hydraPose = math::Pose(msgs::ConvertIgn(_msg->right().pose()));

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
  boost::mutex::scoped_lock lock(this->userCameraPoseMessageMutex);
  std::lock_guard<std::mutex> monitorLock(this->optitrackMonitorMutex);

  gazebo::math::Pose cameraMarker = gazebo::msgs::ConvertIgn(*_msg);

  if ((this->pauseTracking || !this->headOffsetInitialized)
       && this->userCameraPoseValid)
  {
    // T_CC' = (-T_CH - T_MS + T_WS) - (-T_C'Marker - T_HMarker + T_WH)
    //
    // compute optitrackHeadOffset
    //
    if (this->viewpointRotationsEnabled)
    {
      this->optitrackHeadOffset =
                                  this->headMarker
                                + this->userCameraPose
                                + this->worldScreen.GetInverse()
                                + this->monitorScreen
                                + this->cameraMonitor
                                + cameraMarker.GetInverse();
    }
    else
    {
      // for the case rotation not enabled
      this->optitrackHeadOffset =
                                  this->monitorScreen
                                + this->cameraMonitor
                                + cameraMarker.GetInverse()
                                + this->headMarker
                                + this->userCameraPose
                                + this->worldScreen.GetInverse();

      // force a no-rotation offset
      this->optitrackHeadOffset.rot = math::Quaternion();

      // compute the erroroneous target pose from above
      gazebo::math::Pose targetCameraWrong =
                                  this->headMarker.GetInverse()
                                + cameraMarker
                                + this->cameraMonitor.GetInverse()
                                + this->monitorScreen.GetInverse()
                                + this->optitrackHeadOffset
                                + this->worldScreen;

      // correct error from forcing rotation to zero by
      // adding user camera frame error from previous step
      // (userCameraPose.pos - targetCameraWrong.pos) to create a final
      // target pose.pos that will not introduce user
      // camera frame offset
      this->optitrackHeadOffset.pos = this->optitrackHeadOffset.pos
        + (this->userCameraPose.pos - targetCameraWrong.pos);
    }

    this->headOffsetInitialized = true;
  }
  else if (this->headOffsetInitialized)
  {
    //
    // List of frames
    //
    //   Frame           Description
    // --------------------------------------------------------
    //   world           world or inertial frame in simulation.
    //   screen          upper right corner of physical screen
    //                   z-up, x-forward, z-left.
    //   monitor         frame fored by its 3 optitrack markers.
    //   camera          frame of the camera.
    //
    // The chain of transforms (from world to user camera):
    //
    //   Frame           Transform              Frame
    // --------------------------------------------------------
    //   world           worldScreen            screen
    //   screen          monitorScreen.Inv      monitor
    //   monitor         cameraMonitor.Inv      camera
    //   camera          cameraMarker           marker
    //   screen          optitrackHeadOffset    screen offset
    //   marker          headMarker.Inv         head
    //   world           targetCamera           head (user camera)
    //
    // T_WH = T_HMarker + T_C'Marker + T_CC' -T_CM -T_MS + T_WS
    gazebo::math::Pose targetCamera;
    if (this->viewpointRotationsEnabled)
    {
      targetCamera =
                  this->headMarker.GetInverse()
                + this->optitrackHeadOffset
                + cameraMarker
                + this->cameraMonitor.GetInverse()
                + this->monitorScreen.GetInverse()
                + this->worldScreen;
    }
    else
    {
      targetCamera =
                  this->headMarker.GetInverse()
                + cameraMarker
                + this->cameraMonitor.GetInverse()
                + this->monitorScreen.GetInverse()
                + this->optitrackHeadOffset
                + this->worldScreen;
      targetCamera.rot = this->userCameraPose.rot;
    }
    gazebo::msgs::Set(&this->joyMsg, targetCamera.Ign());
    this->viewpointJoyPub->Publish(this->joyMsg);
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUpdateOptitrackArm(ConstPosePtr &_msg)
{
  // By Gazebo pose math,
  // If A = T_OP and B = T_PQ, B+A = T_OQ

  boost::mutex::scoped_lock lock(this->baseLinkMutex);
  std::lock_guard<std::mutex> monitorLock(this->optitrackMonitorMutex);

  gazebo::math::Pose cameraMarker = gazebo::msgs::ConvertIgn(*_msg);

  if (this->pauseTracking || !this->armOffsetInitialized)
  {
    math::Pose worldArm = this->elbowMarkerCorrected
      + this->targetBaseLinkPose;

    // goal: arm sensor orientation should be the same as the screen rotation
    // worldScreen.rot is the screen orientation in the world frame
    // worldArm.rot is the arm orientation in the world frame, set them equal
    worldArm.rot = (this->elbowMarker + this->targetBaseLinkPose).rot;
    // solve for elbowMarkerCorrected.rot based on new worldArm
    this->elbowMarkerCorrected.rot =
      (worldArm - this->targetBaseLinkPose).rot;

    // T_CC' = (-T_CM - T_MS + T_WS) - (-T_C'A - T_AE + T_WE)
    this->optitrackArmOffset =  cameraMarker.GetInverse()
      + this->elbowMarkerCorrected
      + this->targetBaseLinkPose + this->worldScreen.GetInverse() +
        this->monitorScreen + this->cameraMonitor;

/*
    // testing offset frame between cameraMarker and elbowMarker
    this->optitrackArmOffset =
                                this->elbowMarker
                              + this->targetBaseLinkPose
                              + this->worldScreen.GetInverse()
                              + this->monitorScreen
                              + this->cameraMonitor
                              + cameraMarker.GetInverse()
                              ;
*/
/*
    // testing something similar to head tracking without rotation
    // offset frame between worldScreen and monitorScreen
    this->optitrackArmOffset =
                                this->monitorScreen
                              + this->cameraMonitor
                              + cameraMarker.GetInverse()
                              + this->elbowMarker
                              + this->targetBaseLinkPose
                              + this->worldScreen.GetInverse()
                              ;
    // force a no-rotation offset
    this->optitrackArmOffset.rot = math::Quaternion();

    // compute the erroroneous target pose from above
    gazebo::math::Pose targetPoseWrong =
                                this->elbowMarker.GetInverse()
                              + cameraMarker
                              + this->cameraMonitor.GetInverse()
                              + this->monitorScreen.GetInverse()
                              + this->optitrackArmOffset
                              + this->worldScreen
                              ;

    // correct error from forcing rotation to zero by
    // adding target pose error from previous step
    // (current target.pos - targetPoseWrong.pos) to create a final
    // target pose.pos that will not introduce frame offset
    this->optitrackArmOffset.pos = this->optitrackArmOffset.pos
      + (this->targetBaseLinkPose.pos - targetPoseWrong.pos);
*/

    this->armOffsetInitialized = true;
  }
  else if (this->armOffsetInitialized)
  {
    //
    // List of frames
    //
    //   Frame           Description
    // --------------------------------------------------------
    //   world           world or inertial frame in simulation.
    //   screen          upper right corner of physical screen
    //                   z-up, x-forward, z-left.
    //   monitor         frame fored by its 3 optitrack markers.
    //   camera          frame of the camera.
    //   elbow           targetBaseLink's frame
    //
    // The chain of transforms (from world to user camera):
    //
    //   Frame           Transform              Frame
    // --------------------------------------------------------
    //   world           worldScreen            screen
    //   screen          monitorScreen.Inv      monitor
    //   monitor         cameraMonitor.Inv      camera
    //   camera          cameraMarker           marker
    //   screen          optitrackHeadOffset    screen offset
    //   marker          elbowMarker.Inv        elbow
    //   world           targetBaseLink         elbow
    //

    // T_WE = T_AE + T_C'A + T_CC' -T_CM -T_MS + T_WS
    this->targetBaseLinkPose = this->elbowMarkerCorrected.GetInverse()
        + cameraMarker
        + this->optitrackArmOffset + this->cameraMonitor.GetInverse() +
            this->monitorScreen.GetInverse() + this->worldScreen;

/*
    // testing offset frame between cameraMarker and elbowMarker
    this->targetBaseLinkPose =
                  this->elbowMarker.GetInverse()
                + this->optitrackArmOffset
                + cameraMarker
                + this->cameraMonitor.GetInverse()
                + this->monitorScreen.GetInverse()
                + this->worldScreen
                ;
*/
/*
    // offset frame between worldScreen and monitorScreen
    this->targetBaseLinkPose =
                  this->elbowMarker.GetInverse()
                + cameraMarker
                + this->cameraMonitor.GetInverse()
                + this->monitorScreen.GetInverse()
                + this->optitrackArmOffset
                + this->worldScreen
                ;
*/
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnUpdateOptitrackMonitor(ConstPointCloudPtr &_msg)
{
  if (_msg == NULL)
  {
    gzerr << "Message was NULL!" << std::endl;
    return;
  }
  if (_msg->points_size() != 3)
  {
    gzerr << "Message had the wrong number of points!" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> monitorLock(this->optitrackMonitorMutex);

  std::vector<gazebo::math::Vector3> points;
  for (int i = 0; i < _msg->points_size(); ++i)
  {
    points.push_back(msgs::ConvertIgn(_msg->points(i)));
  }

  double maxLength = 0;
  int originPointId = -1;
  int shortPointId = -1;
  int longPointId = -1;
  // Find side with maximum length, choose the "origin" as the opposite point
  for (int i = 0; i < 3; ++i)
  {
    int i1 = (i + 1) % 3;
    int i2 = (i + 2) % 3;
    double length = (points[i1] - points[i2]).GetLength();
    if (length > maxLength)
    {
      maxLength = length;
      originPointId = i;
      longPointId = i1;
      shortPointId = i2;
    }
  }

  if ((points[originPointId]-points[longPointId]).GetLength() <
      (points[originPointId]-points[shortPointId]).GetLength())
  {
    int tmp = longPointId;
    longPointId = shortPointId;
    shortPointId = tmp;
  }

  // X is the longer vector, z is the shorter one
  // X points to the right of the screen, Z points up
  // Y points in
  gazebo::math::Vector3 gx = points[originPointId] - points[longPointId];
  gazebo::math::Vector3 gz = points[originPointId] - points[shortPointId];

  if (gx.GetLength() < gz.GetLength())
  {
    gzerr << "Swapping gx and gz, this should never happen" << std::endl;
    gazebo::math::Vector3 tmp = gx;
    gx = gz;
    gz = tmp;
  }
  gx = gx.Normalize();
  gz = gz.Normalize();

  // gy = gz X gx
  gazebo::math::Vector3 gy = gz.Cross(gx).Normalize();

  gx = gy.Cross(gz).Normalize();

  if (abs(gx.Dot(gy)) > 1e-6 || abs(gx.Dot(gz)) > 1e-6
      || abs(gz.Dot(gy)) > 1e-6)
  {
    gzerr << "Basis vectors are not orthogonal!" << std::endl;
    return;
  }

  // The rotational matrix from the camera (monitor) frame to Gazebo (Screen)
  // frame can be represented with gx, gy, gz as its column vectors
  // Convert from this matrix to a quaternion:
  // http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

  math::Matrix3 G;
  G.SetFromAxes(gx, gy, gz);

  if (G[0][0] + G[1][1] + G[2][2] + 1 <= 0)
  {
    // TODO: Integrate more robust conversion method in Gazebo
    gzerr << "Trace of rotation matrix was invalid!" << std::endl;
    return;
  }
  double qw = sqrt(1 + G[0][0] + G[1][1] + G[2][2])/2;
  GZ_ASSERT(qw > 1e-12, "stop before division by 0");
  double qx = (G[2][1] - G[1][2])/(4*qw);
  double qy = (G[0][2] - G[2][0])/(4*qw);
  double qz = (G[1][0] - G[0][1])/(4*qw);

  gazebo::math::Quaternion monitorRot(qw, qx, qy, qz);

  this->cameraMonitor.pos = points[1];

  // Test: Assert that the inverse of the rotation we calculated rotates
  // gx, gy and gz to their respective unit vectors.
  gazebo::math::Quaternion optitrackToMonitor = monitorRot.GetInverse();
  double xdiff = (gazebo::math::Vector3(1, 0, 0) -
      optitrackToMonitor.RotateVector(gx)).GetLength();
  double ydiff = (gazebo::math::Vector3(0, 1, 0) -
      optitrackToMonitor.RotateVector(gy)).GetLength();
  double zdiff = (gazebo::math::Vector3(0, 0, 1) -
      optitrackToMonitor.RotateVector(gz)).GetLength();

  if (xdiff > 1e-6 || ydiff > 1e-6 || zdiff > 1e-6)
  {
    gzerr << "Calculated transform was wrong. Are all the monitor markers "
          << "visible to the Optitrack?" << std::endl;
    return;
  }

  this->monitorScreen.rot = monitorRot;
  // Assume screen and monitor axes share the same origin
  this->monitorScreen.pos = math::Vector3();
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnToggleViewpointRotations(ConstIntPtr &_msg)
{
  this->viewpointRotationsEnabled = _msg->data() == 0 ? false : true;

  // force re-compute offsets
  this->headOffsetInitialized = false;
}

GZ_REGISTER_MODEL_PLUGIN(HaptixControlPlugin)
}
