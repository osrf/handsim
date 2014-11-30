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

/// Hydra:
#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <linux/hidraw.h>
#include <linux/input.h>
#include <linux/types.h>
#include <cstring>

#include "haptix_gazebo_plugins/HaptixControlPlugin.hh"

// Hydra: Loosely adapted from the following
// https://github.com/ros-drivers/razer_hydra/blob/groovy-devel/src/hydra.cpp
// Ugly hack to work around failing compilation on systems that don't
// yet populate new version of hidraw.h to userspace.
//
// If you need this, please have your distro update the kernel headers.
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif
// Eventually crawl hidraw file system using this:
// http://www.signal11.us/oss/udev/
#define HYDRA_RIGHT_BUMPER 7
#define HYDRA_RIGHT_1 8
#define HYDRA_RIGHT_2 9
#define HYDRA_RIGHT_3 10
#define HYDRA_RIGHT_4 11
#define HYDRA_RIGHT_CENTER 12
#define HYDRA_RIGHT_JOY 13

#define HYDRA_LEFT_LB 0
#define HYDRA_LEFT_1 1
#define HYDRA_LEFT_2 2
#define HYDRA_LEFT_3 3
#define HYDRA_LEFT_4 4
#define HYDRA_LEFT_CENTER 5
#define HYDRA_LEFT_JOY 6

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
HaptixControlPlugin::HaptixControlPlugin()
{
  this->pausePolhemus = true;
  this->gotPausePolhemusRequest = false;

  this->haveHydra = false;

  // Advertise haptix services.
  this->ignNode.Advertise("/haptix/gazebo/GetDeviceInfo",
    &HaptixControlPlugin::HaptixGetDeviceInfoCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/Update",
    &HaptixControlPlugin::HaptixUpdateCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/Grasp",
    &HaptixControlPlugin::HaptixGraspCallback, this);

  // Hydra:
  this->stopHydra = false;
  this->lastCycleStart = common::Time::GetWallTime();
  // magic number for 50% mix at each step
  this->periodEstimate.SetFc(0.11, 1.0);
  this->periodEstimate.SetValue(0.004);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
HaptixControlPlugin::~HaptixControlPlugin()
{
  this->polhemusThread.join();
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  event::Events::DisconnectWorldUpdateEnd(this->updateConnectionEnd);

  // Hydra:
  this->stopHydra = true;
  this->pollHydraThread->join();
}

////////////////////////////////////////////////////////////////////////////////
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
  this->polhemusJoyPub =
    this->gazeboNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

  this->pausePolhemusPub =
    this->gazeboNode->Advertise<gazebo::msgs::Int>("~/polhemus/pause_response");

  this->keySub =
    this->gazeboNode->Subscribe("~/qtKeyEvent",
      &HaptixControlPlugin::OnKey, this);

  this->joySub =
    this->gazeboNode->Subscribe("~/user_camera/joy_twist",
      &HaptixControlPlugin::OnJoy, this);

  this->pausePolhemusSub =
    this->gazeboNode->Subscribe("~/polhemus/pause_request",
      &HaptixControlPlugin::OnPausePolhemus, this);

  this->userCameraPoseValid = false;
  this->userCameraPoseSub =
    this->gazeboNode->Subscribe("~/user_camera/pose",
      &HaptixControlPlugin::OnUserCameraPose, this);

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
  if (this->sdf->HasElement("base_link_to_spacenav_control_point"))
  {
    sdf::ElementPtr bl2scp =
      this->sdf->GetElement("base_link_to_spacenav_control_point");
    this->baseLinktoSpacenavPose = bl2scp->Get<math::Pose>();
  }
  else
  {
    gzdbg << "using defaults for apl_hand.\n";
    this->baseLinktoSpacenavPose = math::Pose(0, -0.4, 0, 0, 0, 0);
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

  // hydra sensor offset
  if (this->sdf->HasElement("base_link_to_hydra_control_point"))
  {
    sdf::ElementPtr bl2hcp =
      this->sdf->GetElement("base_link_to_hydra_control_point");
    this->baseLinkToHydraSensor = bl2hcp->Get<math::Pose>();
  }
  else
  {
    gzdbg << "using defaults for apl_hand.\n";
    this->baseLinkToHydraSensor = math::Pose(0, -0.3, 0, 0, M_PI, -0.5*M_PI);
  }

  // for controller time control
  this->lastTime = this->world->GetSimTime();

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
      this->havePolhemus = true;
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

  // Hydrs:
  int res;
  uint8_t buf[256];
  struct hidraw_report_descriptor rptDesc;
  struct hidraw_devinfo info;

  // Find the Razer device.
  std::string device;
  for (int i = 0; i < 6 && device.empty(); ++i)
  {
    std::ostringstream stream;
    stream << "/sys/class/hidraw/hidraw" << i << "/device/uevent";
    std::ifstream fileIn(stream.str().c_str());
    if (fileIn.is_open())
    {
      std::string line;
      while (std::getline(fileIn, line) && device.empty())
      {
        if (line.find("HID_NAME=Razer Razer Hydra") != std::string::npos)
          device = "/dev/hidraw" + boost::lexical_cast<std::string>(i);
      }
    }
  }

  if (device.empty())
  {
    gzerr << "Unable to find Razer device\n";
    this->haveHydra = false;
  }
  else
  {
    this->hidrawFd = open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (this->hidrawFd < 0)
    {
      gzerr << "couldn't open hidraw device[" << device << "]\n";
      this->haveHydra = false;
    }
    else
    {
      this->haveHydra = true;
    }
  }

  if (this->haveHydra)
  {
    memset(&rptDesc, 0x0, sizeof(rptDesc));
    memset(&info, 0x0, sizeof(info));
    memset(buf, 0x0, sizeof(buf));

    // Get Raw Name
    res = ioctl(this->hidrawFd, HIDIOCGRAWNAME(256), buf);
    if (res < 0)
      gzerr << "Hydro ioctl error HIDIOCGRAWNAME: " << strerror(errno) << "\n";

    // set feature to start it streaming
    memset(buf, 0x0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[9] = 3;
    buf[89] = 6;

    int attempt = 0;
    for (attempt = 0; attempt < 50; ++attempt)
    {
      res = ioctl(this->hidrawFd, HIDIOCSFEATURE(91), buf);
      if (res < 0)
      {
        gzerr << "Unable to start streaming. HIDIOCSFEATURE: "
              << strerror(errno) << "\n";
        common::Time::MSleep(500);
      }
      else
      {
        break;
      }
    }

    if (attempt >= 60)
    {
      gzerr << "Failed to load hydra\n";
      this->haveHydra = false;
    }
  }

  if (this->haveHydra)
  {
    this->pollHydraThread = new boost::thread(
      boost::bind(&HaptixControlPlugin::RunHydra, this));
  }

  this->PublishHaptixControlStatus();
}

////////////////////////////////////////////////////////////////////////////////
// Open spacenav
void HaptixControlPlugin::LoadHandControl()
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  int id;

  // Get joint names and insert id/name pair into map
  sdf::ElementPtr joint = this->sdf->GetElement("joint");
  while (joint)
  {
    joint->GetAttribute("id")->Get(id);
    this->jointNames[id] = joint->Get<std::string>();
    // get next sdf
    // gzdbg << "getting joint name [" << this->jointNames[id] << "]\n";
    joint = joint->GetNextElement("joint");
  }

  // Get pointers to joints
  this->joints.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    /// \TODO: this assumes id's for joints are consecutive, starting with 0
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    // gzdbg << "got gazebo joint [" << this->joints[i]->GetName() << "]\n";
  }

  // Get pid gains and insert into pid
  this->pids.resize(this->joints.size());
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

    // get max [continuous] motor torque associated with this motor
    sdf::ElementPtr motorTorqueSDF =
      motorSDF->GetElement("motor_torque");
    this->motorInfos[id].motorTorque = motorTorqueSDF->Get<double>();
    // gzdbg << "  torque [" << this->motorInfos[id].motorTorque << "]\n";

    // this should return index of the joint in this->joints
    // where this->jointNames matches motorInfos[id].jointName.
    /// \TODO: there must be a faster way of doing this?
    unsigned int j;
    for (j = 0; j < this->jointNames.size(); ++j)
    {
      if (this->jointNames[j] == this->motorInfos[id].jointName)
      {
        this->motorInfos[id].index = j;
        break;
      }
    }

    // get coupled joints from <gearbox> blocks
    if (motorSDF->HasElement("gearbox"))
    {
      sdf::ElementPtr gearboxSDF = motorSDF->GetElement("gearbox");
      while (gearboxSDF)
      {
        // get joint, offset and multiplier
        sdf::ElementPtr jointSDF = gearboxSDF->GetElement("joint");
        sdf::ElementPtr offsetSDF = gearboxSDF->GetElement("offset");
        sdf::ElementPtr multiplierSDF = gearboxSDF->GetElement("multiplier");
        MotorInfo::GearBox g;

        // find index in this->joints that matches jointSDF->Get<std::string>();
        unsigned int k;
        for (k = 0; k < this->jointNames.size(); ++k)
        {
          if (this->jointNames[k] == jointSDF->Get<std::string>())
          {
            g.index = k;
            break;
          }
        }
        g.offset = offsetSDF->Get<double>();
        g.multiplier = multiplierSDF->Get<double>();
        this->motorInfos[id].gearboxes.push_back(g);

        gearboxSDF = gearboxSDF->GetNextElement("gearbox");
      }
    }

    // Adjust max/min torque commands based on <motor_torque>,
    // <gear_ratio> and <gearbox> params.
    for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
    {
      unsigned int m = this->motorInfos[i].index;
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
        unsigned int n = this->motorInfos[i].gearboxes[j].index;
        double coupledJointTorque = jointTorque *
          this->motorInfos[i].gearboxes[j].multiplier;
        this->pids[n].SetCmdMax(coupledJointTorque);
        this->pids[n].SetCmdMin(-coupledJointTorque);
        // gzdbg << " motor torque [" << n
        //       << "] : " << coupledJointTorque << "\n";
      }
    }

    // get next sdf
    motorSDF = motorSDF->GetNextElement("motor");
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
    this->robotCommand.add_ref_vel(0.0);

    // default position and velocity gains
    this->robotCommand.add_gain_pos(1.0);
    this->robotCommand.add_gain_vel(0.0);
  }

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->robotState.add_joint_pos(0);
    this->robotState.add_joint_vel(0);

    // internal command of all joints controller here (not just motors)
    SimRobotCommand c;
    c.ref_pos = 0.0;
    c.ref_vel = 0.0;
    c.gain_pos = 1.0;
    c.gain_vel = 0.0;
    this->simRobotCommands.push_back(c);

     // get gazebo joint handles
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    // if (!this->joints[i])
    //   fprintf(stderr, "joint[%d]=[%s] bad\n",
    //           i, this->jointNames[i].c_str());
  }

  // add a robot state contact per contactSensorInfo
  for (int i = 0; i < this->contactSensorInfos.size(); ++i)
  {
    this->robotState.add_contact(0);
  }

  for (int i = 0; i < imuSensors.size(); ++i)
  {
    haptix::comm::msgs::imu *linacc = this->robotState.add_imu_linacc();
    linacc->set_x(0);
    linacc->set_y(0);
    linacc->set_z(0);
    haptix::comm::msgs::imu *angvel = this->robotState.add_imu_angvel();
    angvel->set_x(0);
    angvel->set_y(0);
    angvel->set_z(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
void HaptixControlPlugin::Reset()
{
  this->targetBaseLinkPose = this->initialBaseLinkPose;

  std::vector<SimRobotCommand>::iterator iter;
  for (iter = this->simRobotCommands.begin();
      iter != this->simRobotCommands.end(); ++iter)
  {
    iter->ref_pos = 0.0;
    iter->ref_vel = 0.0;
  }
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

  const double posScale = 2.0;
  const double rotScale = 5.0;
  {
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

////////////////////////////////////////////////////////////////////////////////
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
      boost::mutex::scoped_lock pauseLock(this->pausePolhemusMutex);

      int armId = 0;  // some number between 0 and numPoses
      if (armId < numPoses)
      {
        // lock in case we are receiving a pause polhemus message over
        // gz transport
        math::Pose armSensorPose = this->convertPolhemusToPose(poses[armId]);
        if (this->pausePolhemus)
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
        if (this->pausePolhemus)
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
          this->polhemusJoyPub->Publish(this->joyMsg);
        }
      }

      // report back if polhemus is paused
      if (this->gotPausePolhemusRequest)
      {
        gzdbg << "have polhemus, responding to pause request\n";
        // signal pause completion
        msgs::Int res;
        res.set_data(1);
        this->pausePolhemusPub->Publish(res);
        // reset flag
        this->gotPausePolhemusRequest = false;
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

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

  if (1)
  {
    this->baseLink->SetWorldPose(pose);
  }
  else
  {
    this->wrench.force.x = this->posPid.Update(errorPos.x, _dt);
    this->wrench.force.y = this->posPid.Update(errorPos.y, _dt);
    this->wrench.force.z = this->posPid.Update(errorPos.z, _dt);
    this->wrench.torque.x = this->rotPid.Update(errorRot.x, _dt);
    this->wrench.torque.y = this->rotPid.Update(errorRot.y, _dt);
    this->wrench.torque.z = this->rotPid.Update(errorRot.z, _dt);
    this->baseLink->SetForce(this->wrench.force);
    this->baseLink->SetTorque(this->wrench.torque);
  }
  // std::cout << "current pose: " << baseLinkPose << std::endl;
  // std::cout << "target pose: " << pose << std::endl;
  // std::cout << "wrench pos: " << this->wrench.force
  //           << " rot: " << this->wrench.torque << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void HaptixControlPlugin::UpdateHandControl(double _dt)
{
  // copy command from hxCommand for motors to list of all joints
  // commanded by this plugin.
  // also account for joint coupling here based on <gearbox> params
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    unsigned int m = this->motorInfos[i].index;
    // gzdbg << m << " : " << this->simRobotCommands[m].ref_pos << "\n";
    // If we're in grasp mode, then take commands from elsewhere
    unsigned int numWristMotors = 3;
    if (this->graspMode && i >= numWristMotors)
    {
      this->simRobotCommands[m].ref_pos = this->graspPositions[i];
      this->simRobotCommands[m].ref_vel = 0.0;
    }
    else
    {
      this->simRobotCommands[m].ref_pos = this->robotCommand.ref_pos(i);
      this->simRobotCommands[m].ref_vel = this->robotCommand.ref_vel(i);
    }
    /// \TODO: fix by implementing better models
    // this->simRobotCommands[m].gain_pos = this->robotCommand.gain_pos(i);
    // this->simRobotCommands[m].gain_vel = this->robotCommand.gain_vel(i);

    // set joint command using coupling specified in <gearbox> params.
    for (unsigned int j = 0; j < this->motorInfos[i].gearboxes.size(); ++j)
    {
      unsigned int n = this->motorInfos[i].gearboxes[j].index;
      // gzdbg << " " << n
      //       << " : " << this->simRobotCommands[n].ref_pos << "\n";
      this->simRobotCommands[n].ref_pos =
        (this->simRobotCommands[m].ref_pos +
         this->motorInfos[i].gearboxes[j].offset)
        * this->motorInfos[i].gearboxes[j].multiplier;
      this->simRobotCommands[n].ref_vel =
        (this->simRobotCommands[m].ref_vel +
         this->motorInfos[i].gearboxes[j].offset)
        * this->motorInfos[i].gearboxes[j].multiplier;

      /// \TODO: fix by implementing better models
      // this->simRobotCommands[n].gain_pos = this->robotCommand.gain_pos(i);
      // this->simRobotCommands[n].gain_vel = this->robotCommand.gain_vel(i);
    }
  }

  // command all joints
  for(unsigned int i = 0; i < this->joints.size(); ++i)
  {
    // get joint positions and velocities
    double position = this->joints[i]->GetAngle(0).Radian();
    double velocity = this->joints[i]->GetVelocity(0);

    // compute position and velocity error
    double errorPos = position - this->simRobotCommands[i].ref_pos;
    double errorVel = velocity - this->simRobotCommands[i].ref_vel;

    // compute overall error
    double error = this->simRobotCommands[i].gain_pos * errorPos
                 + this->simRobotCommands[i].gain_vel * errorVel;

    // compute force needed
    double force = this->pids[i].Update(error, _dt);

    // this->robotState.set_motor_torque(i, force);
    if (this->joints[i])
      this->joints[i]->SetForce(0, force);
    else
      gzerr << "joint [" << this->jointNames[i] << "] bad.\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
void HaptixControlPlugin::OnContactSensorUpdate(int _i)
{
  // how do we know which sensor triggered this update?
  // gzerr << "contactSensorInfos " << this->contactSensorInfos.size() << "\n";
  if (_i < this->contactSensorInfos.size())
  {
    sensors::ContactSensorPtr contactSensor =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(
      this->contactSensorInfos[_i].sensor);

    if (contactSensor)
    {
      msgs::Contacts contacts = contactSensor->GetContacts();
      // contact sensor report contact between pairs of bodies
      // if (contacts.contact().size() > 0)
      //   gzerr << "  name " << contactSensor->GetName()
      //         << " contacts " << contacts.contact().size() << "\n";

      // reset aggregate forces and torques if contacts detected
      this->contactSensorInfos[_i].contactForce = math::Vector3();
      this->contactSensorInfos[_i].contactTorque = math::Vector3();
    
      for (unsigned int j = 0; j < contacts.contact().size(); ++j)
      {
        msgs::Contact contact = contacts.contact(j);
        // each contact can have multiple wrenches
        // if (contact.wrench().size() > 0)
        //   gzerr << "    wrenches " << contact.wrench().size() << "\n";
        for (unsigned int k = 0; k < contact.wrench().size(); ++k)
        {
          msgs::JointWrench wrench = contact.wrench(k);

          // sum up all wrenches from body_1 or body_2
          // check with contact corresponds to the arm and which to the arm
          // compare body_1_name and body_2_name with model name
          if (strncmp(this->model->GetName().c_str(),
                      wrench.body_1_name().c_str(),
                      this->model->GetName().size()) == 0)
          {
            this->contactSensorInfos[_i].contactForce +=
              msgs::Convert(wrench.body_1_wrench().force());
            this->contactSensorInfos[_i].contactTorque +=
              msgs::Convert(wrench.body_1_wrench().torque());
          }
          else if (strncmp(this->model->GetName().c_str(),
                           wrench.body_2_name().c_str(),
                           this->model->GetName().size()) == 0)
          {
            this->contactSensorInfos[_i].contactForce +=
              msgs::Convert(wrench.body_2_wrench().force());
            this->contactSensorInfos[_i].contactTorque +=
              msgs::Convert(wrench.body_2_wrench().torque());
          }
          else
          {
            gzwarn << "collision name does not match model name, averaging.\n";
            this->contactSensorInfos[_i].contactForce +=
              0.5*(msgs::Convert(wrench.body_2_wrench().force()) +
                   msgs::Convert(wrench.body_2_wrench().force()));
            this->contactSensorInfos[_i].contactTorque +=
              0.5*(msgs::Convert(wrench.body_2_wrench().torque()) +
                   msgs::Convert(wrench.body_2_wrench().torque()));
          }

          // gzerr << "        contact [" << _i << ", " << j
          //       << ", " << k << "] : [" << contactForce << "]\n";
        }
      }
      // gzerr << "contact [" << _i
      //       << "]: [" << this->contactSensorInfos[_i].contactForce
      //       << "]\n";
    }
    else
    {
      gzerr << "sensor [" << this->contactSensorInfos[_i].sensor->GetName()
            << "] is not a ContactSensor.\n";
    }
  }
  else
  {
    gzerr << "sensor [" << _i
          << "] is out of range of contactSensorInfos["
          << this->contactSensorInfos.size() << "].\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
void HaptixControlPlugin::PublishHaptixControlStatus()
{
  // finished loading arm? send status
  this->haptixStatusPub =
    this->gazeboNode->Advertise<gazebo::msgs::Int>("~/haptix_load");
  gazebo::msgs::Int loadStat;
  loadStat.set_data(1);
  this->haptixStatusPub->Publish(loadStat);
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::GetRobotStateFromSim()
{
  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    unsigned int m = motorInfos[i].index;
    this->robotState.set_motor_pos(i, this->joints[m]->GetAngle(0).Radian());
    this->robotState.set_motor_vel(i, this->joints[m]->GetVelocity(0));
    this->robotState.set_motor_torque(i, this->joints[m]->GetForce(0));
  }

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->robotState.set_joint_pos(i, this->joints[i]->GetAngle(0).Radian());
    this->robotState.set_joint_vel(i, this->joints[i]->GetVelocity(0));
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
    haptix::comm::msgs::imu *linacc = this->robotState.mutable_imu_linacc(i);
    math::Vector3 acc = this->imuSensors[i]->GetLinearAcceleration();
    math::Vector3 vel = this->imuSensors[i]->GetAngularVelocity();
    linacc->set_x(acc.x);
    linacc->set_y(acc.y);
    linacc->set_z(acc.z);
    haptix::comm::msgs::imu *angvel = this->robotState.mutable_imu_angvel(i);
    angvel->set_x(vel.x);
    angvel->set_y(vel.y);
    angvel->set_z(vel.z);
  }
}

////////////////////////////////////////////////////////////////////////////////
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

    if (this->haveHydra)
      this->UpdateHydra();

    // compute wrench needed
    this->UpdateBaseLink(dt);

    // Get robot state from simulation
    this->GetRobotStateFromSim();

    // control finger joints
    this->UpdateHandControl(dt);

    this->lastTime = curTime;
  }
  else if (dt < 0)
  {
    // has time been reset?
    this->lastTime = curTime;
  }
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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
void HaptixControlPlugin::HaptixGetDeviceInfoCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxDevice &/*_req*/,
      haptix::comm::msgs::hxDevice &_rep, bool &_result)
{
  // is this needed?
  // if (_service != deviceInfoTopic)
  //   _result = false;

  _rep.set_nmotor(this->motorInfos.size());
  _rep.set_njoint(this->joints.size());
  _rep.set_ncontactsensor(this->contactSensorInfos.size());
  _rep.set_nimu(this->imuSensors.size());

  for (unsigned int i = 0; i < this->motorInfos.size(); ++i)
  {
    haptix::comm::msgs::hxJointAngle *joint = _rep.add_limit();
    unsigned int m = this->motorInfos[i].index;
    joint->set_minimum(this->joints[m]->GetLowerLimit(0).Radian());
    joint->set_maximum(this->joints[m]->GetUpperLimit(0).Radian());
  }

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
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    std::cout << "\tMotor " << i << ":" << std::endl;
    std::cout << "\t\t" << _req.ref_pos(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel(i) << std::endl;
    std::cout << "\t\t" << _req.gain_pos(i) << std::endl;
    std::cout << "\t\t" << _req.gain_vel(i) << std::endl;
  }*/

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

  for (unsigned int i=0; i < _req.grasps_size(); ++i)
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
void HaptixControlPlugin::OnJoy(ConstJoystickPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->joystickMessageMutex);
  this->latestJoystickMessage = *_msg;
  this->newJoystickMessage = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnPausePolhemus(ConstIntPtr &_msg)
{
  boost::mutex::scoped_lock pauseLock(this->pausePolhemusMutex);
  if (this->havePolhemus)
  {
    gzerr << "got " << _msg->data() << "\n";
    if (_msg->data() == 0)
    {
      this->pausePolhemus = false;
    }
    else
    {
      this->pausePolhemus = true;
    }
    // set request flag
    this->gotPausePolhemusRequest = true;
    gzdbg << "got request, set flag " << this->gotPausePolhemusRequest << "\n";
  }
  else
  {
    // if there's no polhemus
    gzdbg << "no polhemus, but responding to pause request\n";
    // signal pause completion
    msgs::Int res;
    res.set_data(0);
    this->pausePolhemusPub->Publish(res);
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnKey(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->baseLinkMutex);
  // gzdbg << "got key [" << _msg->data()
  //           << "] press [" << _msg->dbl_data() << "]\n";
  // char key = _msg->data().c_str()[0];
  // if (strcmp(&key, &this->lastKeyPressed) != 0)
  if (_msg->dbl_data() > 0.0)
  {
    // pressed "p" or spacebar?
    if (strcmp(_msg->data().c_str(), "p") == 0 ||
        strcmp(_msg->data().c_str(), " ") == 0)
      this->pausePolhemus = !this->pausePolhemus;
    gzdbg << " pausing polhemus [" << this->pausePolhemus << "]\n";
  }
  else
  {
    // clear
    // gzdbg << " released key [" << this->keyPressed << "]\n";
  }
}

/////////////////////////////////////////////////
bool HaptixControlPlugin::PollHydra(float _lowPassCornerHz)
{
  if (this->hidrawFd < 0)
  {
    gzerr << "hidraw device is not open, couldn't poll.\n";
    return false;
  }

  if (_lowPassCornerHz <= std::numeric_limits<float>::epsilon())
  {
    gzerr << "Corner frequency for low-pass filter must be greater than 0."
      << "Using a default value of 2.5Hz.\n";
    // Set a default value if the value is incorrect.
    _lowPassCornerHz = 2.5;
  }

  uint8_t buf[64];
  ssize_t nread = read(this->hidrawFd, buf, sizeof(buf));

  // No updates.
  if (nread <= 0)
    return false;


  static bool firstTime = true;

  // Update average read period
  if (!firstTime)
  {
    this->periodEstimate.Process(
      (common::Time::GetWallTime() - this->lastCycleStart).Double());
  }

  this->lastCycleStart = common::Time::GetWallTime();

  if (firstTime)
    firstTime = false;

  // Update filter frequencies
  float fs = 1.0 / this->periodEstimate.GetValue();
  float fc = _lowPassCornerHz;

  for (int i = 0; i < 2; ++i)
  {
    this->hydraFilterPos[i].SetFc(fc, fs);
    this->hydraFilterQuat[i].SetFc(fc, fs);
  }

  // Read data
  this->rawPos[0] = *(reinterpret_cast<int16_t *>(buf+8));
  this->rawPos[1] = *(reinterpret_cast<int16_t *>(buf+10));
  this->rawPos[2] = *(reinterpret_cast<int16_t *>(buf+12));
  this->rawQuat[0] = *(reinterpret_cast<int16_t *>(buf+14));
  this->rawQuat[1] = *(reinterpret_cast<int16_t *>(buf+16));
  this->rawQuat[2] = *(reinterpret_cast<int16_t *>(buf+18));
  this->rawQuat[3] = *(reinterpret_cast<int16_t *>(buf+20));
  this->rawButtons[0] = buf[22] & 0x7f;
  this->rawAnalog[0] = *(reinterpret_cast<int16_t *>(buf+23));
  this->rawAnalog[1] = *(reinterpret_cast<int16_t *>(buf+25));
  this->rawAnalog[2] = buf[27];

  this->rawPos[3] = *(reinterpret_cast<int16_t *>(buf+30));
  this->rawPos[4] = *(reinterpret_cast<int16_t *>(buf+32));
  this->rawPos[5] = *(reinterpret_cast<int16_t *>(buf+34));
  this->rawQuat[4] = *(reinterpret_cast<int16_t *>(buf+36));
  this->rawQuat[5] = *(reinterpret_cast<int16_t *>(buf+38));
  this->rawQuat[6] = *(reinterpret_cast<int16_t *>(buf+40));
  this->rawQuat[7] = *(reinterpret_cast<int16_t *>(buf+42));
  this->rawButtons[1] = buf[44] & 0x7f;
  this->rawAnalog[3] = *(reinterpret_cast<int16_t *>(buf+45));
  this->rawAnalog[4] = *(reinterpret_cast<int16_t *>(buf+47));
  this->rawAnalog[5] = buf[49];

  boost::mutex::scoped_lock lock(this->hydraMutex);
  // Put the raw position and orientation into Gazebo coordinate frame
  for (int i = 0; i < 2; ++i)
  {
    this->hydraPos[i].x = -this->rawPos[3*i+1] * 0.001;
    this->hydraPos[i].y = -this->rawPos[3*i+0] * 0.001;
    this->hydraPos[i].z = -this->rawPos[3*i+2] * 0.001;

    this->hydraQuat[i].w = this->rawQuat[i*4+0] / 32768.0;
    this->hydraQuat[i].x = -this->rawQuat[i*4+2] / 32768.0;
    this->hydraQuat[i].y = -this->rawQuat[i*4+1] / 32768.0;
    this->hydraQuat[i].z = -this->rawQuat[i*4+3] / 32768.0;
  }

  // Apply filters
  for (int i = 0; i < 2; ++i)
  {
    this->hydraQuat[i] = this->hydraFilterQuat[i].Process(this->hydraQuat[i]);
    this->hydraPos[i] = this->hydraFilterPos[i].Process(this->hydraPos[i]);
  }

  this->hydraAnalog[0] = this->rawAnalog[0] / 32768.0;
  this->hydraAnalog[1] = this->rawAnalog[1] / 32768.0;
  this->hydraAnalog[2] = this->rawAnalog[2] / 255.0;
  this->hydraAnalog[3] = this->rawAnalog[3] / 32768.0;
  this->hydraAnalog[4] = this->rawAnalog[4] / 32768.0;
  this->hydraAnalog[5] = this->rawAnalog[5] / 255.0;

  for (int i = 0; i < 2; ++i)
  {
    this->hydraButtons[i*7  ] = (this->rawButtons[i] & 0x01) ? 1 : 0;
    this->hydraButtons[i*7+1] = (this->rawButtons[i] & 0x04) ? 1 : 0;
    this->hydraButtons[i*7+2] = (this->rawButtons[i] & 0x08) ? 1 : 0;
    this->hydraButtons[i*7+3] = (this->rawButtons[i] & 0x02) ? 1 : 0;
    this->hydraButtons[i*7+4] = (this->rawButtons[i] & 0x10) ? 1 : 0;
    this->hydraButtons[i*7+5] = (this->rawButtons[i] & 0x20) ? 1 : 0;
    this->hydraButtons[i*7+6] = (this->rawButtons[i] & 0x40) ? 1 : 0;
  }

  return true;
}

/////////////////////////////////////////////////
void HaptixControlPlugin::RunHydra()
{
  double cornerHz = 200.0;

  while (!this->stopHydra)
  {
    if (!this->PollHydra(cornerHz))
      common::Time::NSleep(250000);
  }

  if (this->hidrawFd >= 0)
  {
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[89] = 5;

    if (ioctl(this->hidrawFd, HIDIOCSFEATURE(91), buf) < 0)
    {
      gzerr << "Unable to stopHydra streaming. HIDIOCSFEATURE: "
            << strerror(errno) << "\n";
    }

    close(this->hidrawFd);
  }
}

//////////////////////////////////////////////////
void HaptixControlPlugin::UpdateHydra()
{
  // copy hydraPos and hydraQuat to 
  boost::mutex::scoped_lock lock(this->hydraMutex);

  /* code used to construct ROS joy message
  math::Pose origRight(this->hydraPos[1], this->hydraQuat[1]);

  math::Pose pivotRight = origRight;
  math::Pose grabRight = origRight;

  pivotRight.pos += origRight.rot * math::Vector3(-0.04, 0, 0);
  grabRight.pos += origRight.rot * math::Vector3(-0.12, 0, 0);

  math::Pose origLeft(this->hydraPos[0], this->hydraQuat[0]);

  math::Pose pivotLeft = origLeft;
  math::Pose grabLeft = origLeft;

  pivotLeft.pos += origLeft.rot.RotateVector(math::Vector3(-0.04, 0, 0));
  grabLeft.pos += origLeft.rot.RotateVector(math::Vector3(-0.12, 0, 0));

  msgs::Hydra msg;
  msgs::Hydra::Paddle *rightPaddle = msg.mutable_right();
  msgs::Hydra::Paddle *leftPaddle = msg.mutable_left();

  // Analog 0: Left right(+) left(-)
  // Analog 1: Left forward(+) back(-)
  // Analog 2: Left trigger(0-1)
  // Analog 3: Right right(+) left(-)
  // Analog 4: Right forward(+) back(-)
  // Analog 5: Right trigger(0-1)
  rightPaddle->set_joy_y(this->hydraAnalog[3]);
  rightPaddle->set_joy_x(this->hydraAnalog[4]);
  rightPaddle->set_trigger(this->hydraAnalog[5]);

  leftPaddle->set_joy_y(this->hydraAnalog[0]);
  leftPaddle->set_joy_x(this->hydraAnalog[1]);
  leftPaddle->set_trigger(this->hydraAnalog[2]);

  leftPaddle->set_button_bumper(this->hydraButtons[0]);
  leftPaddle->set_button_1(this->hydraButtons[1]);
  leftPaddle->set_button_2(this->hydraButtons[2]);
  leftPaddle->set_button_3(this->hydraButtons[3]);
  leftPaddle->set_button_4(this->hydraButtons[4]);

  leftPaddle->set_button_center(this->hydraButtons[5]);
  leftPaddle->set_button_joy(this->hydraButtons[6]);

  rightPaddle->set_button_bumper(this->hydraButtons[7]);
  rightPaddle->set_button_1(this->hydraButtons[8]);
  rightPaddle->set_button_2(this->hydraButtons[9]);
  rightPaddle->set_button_3(this->hydraButtons[10]);
  rightPaddle->set_button_4(this->hydraButtons[11]);
  rightPaddle->set_button_center(this->hydraButtons[12]);
  rightPaddle->set_button_joy(this->hydraButtons[13]);

  msgs::Set(rightPaddle->mutable_pose(), grabRight);
  msgs::Set(leftPaddle->mutable_pose(), grabLeft);

  this->pub->Publish(msg);
  */

  // update target base link pose based on hydra
  {
    // take the right paddle pose
    this->hydraPose = math::Pose(this->hydraPos[1], this->hydraQuat[1]);

    math::Pose armSensorPose = this->hydraPose;
    if (this->pausePolhemus)
    {
      // calibration mode, update offset
      this->sourceWorldPoseArmOffset =
        (armSensorPose.GetInverse() + this->baseLinkToHydraSensor +
         this->targetBaseLinkPose) - this->sourceWorldPose;
    }
    else
    {
      boost::mutex::scoped_lock lock(this->baseLinkMutex);
      this->targetBaseLinkPose = this->baseLinkToHydraSensor.GetInverse()
        + armSensorPose
        + (this->sourceWorldPoseArmOffset + this->sourceWorldPose);
    }
  }

}

GZ_REGISTER_MODEL_PLUGIN(HaptixControlPlugin)
}
