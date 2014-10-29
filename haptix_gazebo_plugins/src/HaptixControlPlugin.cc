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

#include "haptix_gazebo_plugins/HaptixControlPlugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
HaptixControlPlugin::HaptixControlPlugin()
{
  this->keyPressed = static_cast<char>(0);

  // Advertise haptix services.
  this->ignNode.Advertise("/haptix/gazebo/GetDeviceInfo",
    &HaptixControlPlugin::HaptixGetDeviceInfoCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/Update",
    &HaptixControlPlugin::HaptixUpdateCallback, this);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
HaptixControlPlugin::~HaptixControlPlugin()
{
  this->polhemusThread.join();
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
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
    this->gazeboNode->Advertise<gazebo::msgs::Pose>("~/polhemus/joy");
  this->keySub =
    this->gazeboNode->Subscribe("~/qtKeyEvent",
      &HaptixControlPlugin::OnKey, this);
  this->joySub =
    this->gazeboNode->Subscribe("~/user_camera/joy_twist",
      &HaptixControlPlugin::OnJoy, this);

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

  // we should get this from the polhemus sensor, but first approximate
  // it as some constant offset from arm base link.
  // assuming arm is over the table, and the
  this->initialCameraPose = math::Pose();  // + this->baseLink->GetWorldPose();
  this->targetCameraPose = this->initialCameraPose;

  this->polhemusSourceLink =
    this->world->GetModel("polhemus_source")->GetLink("link");
  if (!this->polhemusSourceLink)
  {
    gzerr << "world needs a polhemus_source model with link named link\n";
    return;
  }
  // for tracking polhemus setup, where is the source in the world frame?
  this->sourceWorldPose = this->polhemusSourceLink->GetWorldPose();
  // transform from polhemus sensor orientation to base link frame
  // -0.6 meters towards wrist from elbow
  // -0.7 rad pitch up
  // 90 degrees yaw to the left
  this->baseLinkToArmSensor = math::Pose(0, -0.6, 0, 0, -0.7, -0.5*M_PI);
  // transform from polhemus sensor orientation to camera frame
  // 10cm to the right of the sensor is roughly where the eyes are
  // -0.3 rad pitch up: sensor is usually tilted upwards when worn near wrist
  this->cameraToHeadSensor = math::Pose(0, 0.1, 0, 0.0, -0.3, 0.0);

  // for controller time control
  this->lastTime = this->world->GetSimTime();

  // initialize PID's
  const double pGain = 10000.0;
  const double cmdMax = 10000.0;
  const double cmdMin = -10000.0;
  this->posPid.Init(pGain, 0, 0.0, 0, 0, cmdMax, cmdMin);
  this->rotPid.Init(pGain, 0, 0.0, 0, 0, cmdMax, cmdMin);

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
    std::cerr << "No usable polhemus setup detected.\n";

  this->haveKeyboard = false;

  this->haveKeyboard = this->LoadKeyboard();

  this->LoadHandControl();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HaptixControlPlugin::UpdateStates, this));

  // base joint damping
  const double baseJointDamping = 0.01;
  this->baseJoint->SetParam("erp", 0, 0.0);
  this->baseJoint->SetParam("cfm", 0, baseJointDamping);
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
    /// \TODO: assume id in order
    this->joints.push_back(this->model->GetJoint(this->jointNames[id]));

    // get next sdf
    joint = joint->GetNextElement("joint");
  }

  // Get pid gains and insert into pid
  this->pids.resize(this->joints.size());
  sdf::ElementPtr pid = this->sdf->GetElement("pid");
  std::cerr << "getting pid\n";
  while (pid)
  {
    pid->GetAttribute("id")->Get(id);
    std::cerr << "getting pid [" << id << "]\n";
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

  // Get motor names and insert id/name pair into map
  sdf::ElementPtr motor = this->sdf->GetElement("motor");
  while (motor)
  {
    motor->GetAttribute("id")->Get(id);
    this->motorNames[id] = motor->Get<std::string>();

    /// \TODO: assume id in order
    this->motors.push_back(this->model->GetJoint(this->motorNames[id]));

    // get next sdf
    motor = motor->GetNextElement("motor");
  }

  // Get contact sensor names and insert id/name pair into map
  sdf::ElementPtr contactSensor = this->sdf->GetElement("contactSensor");
  while (contactSensor)
  {
    contactSensor->GetAttribute("id")->Get(id);
    this->contactSensorNames[id] = contactSensor->Get<std::string>();

    // get sensor from gazebo
    sensors::SensorManager *mgr = sensors::SensorManager::Instance();
    // Get a pointer to the contact sensor
    sensors::ContactSensorPtr sensor =
        boost::dynamic_pointer_cast<sensors::ContactSensor>
        (mgr->GetSensor(this->contactSensorNames[id]));
    if (sensor)
    {
      /// \TODO: assume id in order
      this->contactSensors.push_back(sensor);
    }
    else
    {
      gzerr << "Contact Sensor [" << this->contactSensorNames[id]
            << "] not found.\n";
    }

    // get next sdf
    contactSensor = contactSensor->GetNextElement("contactSensor");
  }

  // Get imuSensor names and insert id/name pair into map
  sdf::ElementPtr imuSensor = this->sdf->GetElement("imuSensor");
  while (imuSensor)
  {
    imuSensor->GetAttribute("id")->Get(id);
    this->imuSensorNames[id] = imuSensor->Get<std::string>();

    // get sensor from gazebo
    sensors::SensorManager *mgr = sensors::SensorManager::Instance();
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

  // Allocate memory for all the protobuf fields.
  for (unsigned int i = 0; i < this->motors.size(); ++i)
  {
    this->robotState.add_motor_pos(0);
    this->robotState.add_motor_vel(0);
    this->robotState.add_motor_torque(0);
  }

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->robotState.add_joint_pos(0);
    this->robotState.add_joint_vel(0);

    // initialize position command.
    this->robotCommand.add_ref_pos(0.0);

     // get gazebo joint handles
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    // if (!this->joints[i])
    //   fprintf(stderr, "joint[%d]=[%s] bad\n",
    //           i, this->jointNames[i].c_str());
  }

  const int num_contact_sensors = 0;  /// \TODO: fix me, add sdf
  for (int i = 0; i < num_contact_sensors; ++i)
    this->robotState.add_contact(0);

  const int num_imu_sensors = 0;  /// \TODO: fix me, add sdf
  for (int i = 0; i < num_imu_sensors; ++i)
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
// Open keyboard commands

void HaptixControlPlugin::SetKeyboardPose(const std::string &/*_topic*/,
                     const msgs::Pose &_pose)
{
  math::Pose inputPose(msgs::Convert(_pose));

  this->keyboardPose = inputPose*this->keyboardPose;

  // Add pose to our keyboardPose object

  this->staleKeyboardPose = false;
}

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

  const double posScale = 2.0;
  const double rotScale = 5.0;
  {
    boost::mutex::scoped_lock lock(this->baseLinkMutex);
    this->targetBaseLinkPose.pos += _dt * posScale * posRate;
    this->targetBaseLinkPose.rot =
      this->targetBaseLinkPose.rot.Integrate(rotScale * rotRate, _dt);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update targetBaseLinkPose using Polhemus
void HaptixControlPlugin::UpdatePolhemus()
{
  // Get current pose from Polhemus
  polhemus_pose_t poses[8];
  while (true)
  {
    int numPoses = 8;  // fill with max poses to read, returns actual poses
    if (!polhemus_get_poses(this->polhemusConn, poses, &numPoses, 100))
    {
      int armId = 0;  // some number between 0 and numPoses
      if (armId < numPoses)
      {
        math::Pose armSensorPose = this->convertPolhemusToPose(poses[armId]);
        // std::cout << "arm [" << armSensorPose << "]\n";
        const char *cKey = "c";
        if (strcmp(&this->keyPressed, cKey) == 0)
        {
          // calibration mode, update this->baseLinkToArmSensor
          // withouthis->world->IsPaused())t changing targetBaseLinkPose
          math::Pose baseLinkPose = this->baseLink->GetWorldPose();
          this->baseLinkToArmSensor =
            (armSensorPose + this->sourceWorldPose) - baseLinkPose;
        }
        else
        {
          boost::mutex::scoped_lock lock(this->baseLinkMutex);
          this->targetBaseLinkPose = this->baseLinkToArmSensor.GetInverse()
            + armSensorPose + this->sourceWorldPose;
        }
      }

      int headId = 1;
      if (headId < numPoses)
      {
        math::Pose headSensorPose = this->convertPolhemusToPose(poses[headId]);
        this->targetCameraPose = this->cameraToHeadSensor.GetInverse()
          + headSensorPose + this->sourceWorldPose;

        gazebo::msgs::Set(&this->joyMsg, this->targetCameraPose);
        this->polhemusJoyPub->Publish(this->joyMsg);
      }
    }
    else
    {
      std::cerr << "polhemus_get_pose() failed\n";
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

////////////////////////////////////////////////////////////////////////////////
//
void HaptixControlPlugin::UpdateHandControl(double _dt)
{
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    double position = this->joints[i]->GetAngle(0).Radian();
    // double velocity = this->joints[i]->GetVelocity(0);
    double positionError = position - this->robotCommand.ref_pos(i);
    double force = this->pids[i].Update(positionError, _dt);
    // this->robotState.set_motor_torque(i, force);
    if (this->joints[i])
      this->joints[i]->SetForce(0, force);
    else
      fprintf(stderr, "joint bad\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::GetRobotStateFromSim()
{
  for (unsigned int i = 0; i < this->motors.size(); ++i)
  {
    this->robotState.set_motor_pos(i, i);
    this->robotState.set_motor_vel(i, i + 1);
    this->robotState.set_motor_torque(i, i + 2);
  }

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->robotState.set_joint_pos(i, i);
    this->robotState.set_joint_vel(i, i + 1);
  }

  for (unsigned int i = 0; i < this->contactSensors.size(); ++i)
    this->robotState.set_contact(i, i);

  for (unsigned int i = 0; i < this->imuSensors.size(); ++i)
  {
    haptix::comm::msgs::imu *linacc = this->robotState.mutable_imu_linacc(i);
    linacc->set_x(i);
    linacc->set_y(i + 1);
    linacc->set_z(i + 2);
    haptix::comm::msgs::imu *angvel = this->robotState.mutable_imu_angvel(i);
    angvel->set_x(i + 3);
    angvel->set_y(i + 4);
    angvel->set_z(i + 5);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void HaptixControlPlugin::UpdateStates()
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

  _rep.set_nmotor(this->motors.size());
  _rep.set_njoint(this->joints.size());
  _rep.set_ncontactsensor(this->contactSensors.size());
  _rep.set_nimu(this->imuSensors.size());

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    haptix::comm::msgs::hxJointAngle *joint = _rep.add_limit();
    joint->set_minimum(this->joints[i]->GetLowerLimit(0).Radian());
    joint->set_maximum(this->joints[i]->GetUpperLimit(0).Radian());
  }

  _result = true;
}

//////////////////////////////////////////////////
/// using haptix_comm service callback
void HaptixControlPlugin::HaptixUpdateCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxCommand &_req,
      haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  boost::mutex::scoped_lock lock(this->updateMutex);

  // is this needed?
  // if (_service != updateTopic)
  //   _result = false;

  // Read the request parameters.
  // Debug output.
  std::cout << "Received a new command:" << std::endl;
  /*for (unsigned int i = 0; i < this->joints.size(); ++i)
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
void HaptixControlPlugin::OnJoy(ConstJoystickPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->joystickMessageMutex);
  this->latestJoystickMessage = *_msg;
  this->newJoystickMessage = true;
}

//////////////////////////////////////////////////
void HaptixControlPlugin::OnKey(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->baseLinkMutex);
  // std::cerr << "got key [" << _msg->data()
  //           << "] press [" << _msg->dbl_data() << "]\n";
  if (_msg->dbl_data() > 0.0)
  {
    // pressed
    this->keyPressed = _msg->data().c_str()[0];
    // std::cerr << " pressed key [" << this->keyPressed << "]\n";
  }
  else
  {
    // clear
    this->keyPressed = static_cast<char>(0);
    // std::cerr << " released key [" << this->keyPressed << "]\n";
  }
}

GZ_REGISTER_MODEL_PLUGIN(HaptixControlPlugin)
}
