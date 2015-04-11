/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/gui/GLWidget.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/RenderWidget.hh>
#include <gazebo/gui/RenderWidget.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/util/LogRecord.hh>
#include <gazebo/plugins/TimerGUIPlugin.hh>

#include "haptix/comm/msg/hxCommand.pb.h"
#include "haptix/comm/msg/hxGrasp.pb.h"

#include "handsim/HaptixWorldPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(HaptixWorldPlugin)

/////////////////////////////////////////////////
HaptixWorldPlugin::HaptixWorldPlugin()
{
}

/////////////////////////////////////////////////
HaptixWorldPlugin::~HaptixWorldPlugin()
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::Load(physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->sdf = _sdf;

  // Advertise Gazebo transport topics
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init(_world->GetName());
  // TODO: different timer topics?
  this->timerPublisher =
      this->gzNode->Advertise<msgs::GzString>("~/timer_control");
  this->worldControlPub = this->gzNode->Advertise<gazebo::msgs::WorldControl>
                              ("~/world_control");

  this->pausePub = this->gzNode->Advertise<gazebo::msgs::Int>
                      ("~/motion_tracking/pause_request");

  // Advertise haptix sim services.
  this->ignNode.Advertise("/haptix/gazebo/hxs_siminfo",
    &HaptixWorldPlugin::HaptixSimInfoCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_camera_transform",
    &HaptixWorldPlugin::HaptixCameraTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_camera_transform",
    &HaptixWorldPlugin::HaptixSetCameraTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_contacts",
    &HaptixWorldPlugin::HaptixContactPointsCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_state",
    &HaptixWorldPlugin::HaptixStateCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_add_model",
    &HaptixWorldPlugin::HaptixAddModelCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_remove_model_id",
    &HaptixWorldPlugin::HaptixRemoveModelCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_transform",
    &HaptixWorldPlugin::HaptixModelTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_linear_velocity",
    &HaptixWorldPlugin::HaptixLinearVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_angular_velocity",
    &HaptixWorldPlugin::HaptixAngularVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_force",
    &HaptixWorldPlugin::HaptixForceCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_torque",
    &HaptixWorldPlugin::HaptixTorqueCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_reset",
    &HaptixWorldPlugin::HaptixResetCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_reset_timer",
    &HaptixWorldPlugin::HaptixResetTimerCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_start_timer",
    &HaptixWorldPlugin::HaptixStartTimerCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_stop_timer",
    &HaptixWorldPlugin::HaptixStopTimerCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_timer",
    &HaptixWorldPlugin::HaptixTimerCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_is_logging",
    &HaptixWorldPlugin::HaptixIsLoggingCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_start_logging",
    &HaptixWorldPlugin::HaptixStartLoggingCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_stop_logging",
    &HaptixWorldPlugin::HaptixStopLoggingCallback, this);
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::Init()
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::Reset()
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSimInfoCallback( const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxSimInfo &_rep, bool &_result)
{
  _rep.clear_models();
  // Get models
  physics::Model_V modelVector = this->world->GetModels();
  for (auto &model : modelVector)
  {
    haptix::comm::msgs::hxModel* modelMsg = _rep.add_models();
    if (!HaptixWorldPlugin::ConvertModel(*model, *modelMsg))
    {
      _result = false;
      return;
    }
  }

  rendering::UserCameraPtr camera = gui::get_active_camera();
  if (!camera)
  {
    _result = false;
    return;
  }
  gazebo::math::Pose pose = camera->GetWorldPose();
  if (!HaptixWorldPlugin::ConvertTransform(pose, *_rep.mutable_camera_transform()))
  {
    _result = false;
    return;
  }

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixCameraTransformCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxTransform &_rep, bool &_result)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  if (!camera)
  {
    _result = false;
    return;
  }
  gazebo::math::Pose pose = camera->GetWorldPose();
  if (!HaptixWorldPlugin::ConvertTransform(pose, _rep))
  {
    _result = false;
    return;
  }

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetCameraTransformCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxTransform &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  if (!camera)
  {
    _result = false;
    return;
    // TODO make new user camera?
  }
  gazebo::math::Pose pose;
  if (!HaptixWorldPlugin::ConvertTransform(_req, pose))
  {
    _result = false;
    return;
  }
  camera->SetWorldPose(pose);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixContactPointsCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxContactPoint_V &_rep, bool &_result)
{
  std::string modelName = _req.data();
  std::vector<physics::Contact*> contacts =
      this->world->GetPhysicsEngine()->GetContactManager()->GetContacts();
  for (auto contact: contacts)
  {
    // If contact is not relevant to the requested model name
    if (contact->collision1->GetLink()->GetModel()->GetName() != modelName &&
        contact->collision2->GetLink()->GetModel()->GetName() != modelName)
    {
      continue;
    }
    for (int i = 0; i < contact->count; i++)
    {
      haptix::comm::msgs::hxContactPoint* contactMsg = _rep.add_contacts();

      contactMsg->set_link1(contact->collision1->GetLink()->GetName());
      contactMsg->set_link2(contact->collision2->GetLink()->GetName());

      // TODO !!!!!! Adjust positions from relative to CoM of link to global
      // TODO tangents
      // frame/contact frame
      ConvertVector(contact->positions[i], *contactMsg->mutable_point());
      ConvertVector(contact->normals[i], *contactMsg->mutable_normal());
      // force is always body1 onto body2
      ConvertVector(contact->wrench[i].body2Force, *contactMsg->mutable_force());
      contactMsg->set_distance(contact->depths[i]);
    }
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStateCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxModel &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  // TODO
  /*physics::JointPtr joint = this->world->GetModel(_req.name())->
      GetJoint(jointMsg.name());
  if (!joint)
  {
    _result = false;
    return;
  }
  joint->SetPosition(0, jointMsg.pos());
  joint->SetVelocity(0, jointMsg.vel());
  joint->SetForce(0, jointMsg.torque_motor());*/

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAddModelCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxModel &_rep, bool &_result)
{
  std::string xml = _req.string_value();
  sdf::SDF modelSDF;
  modelSDF.SetFromString(xml);
  sdf::ElementPtr modelElement = modelSDF.Root();
  if (!modelElement->HasAttribute("name"))
  {
    _result = false;
    return;
  }

  // Set name
  modelElement->GetAttribute("name")->Set<std::string>(_req.name());

  // load an SDF element from XML
  this->world->InsertModelSDF(modelSDF);
  // problem with this approach: cannot retrieve model pointer?

  physics::ModelPtr model = this->world->GetModel(_req.name());

  if (!model)
  {
    _result = false;
    return;
  }

  math::Pose pose;
  if (!ConvertTransform(_req.transform(), pose))
  {
    _result = false;
    return;
  }

  // Set pose
  model->SetWorldPose(pose);

  ConvertModel(*model, _rep);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixRemoveModelCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  // RemoveModel blocks, consider own thread?
  physics::ModelPtr model = this->world->GetModel(_req.data());
  if (!model)
  {
    // should this be false?
    _result = true;
    return;
  }
  this->world->RemoveModel(model);

  // If model still exists
  if (this->world->GetModel(_req.data()))
  {
    _result = false;
  }
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelTransformCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    _result = false;
    return;
  }
  math::Pose pose;
  if (!ConvertTransform(_req.transform(), pose))
  {
    _result = false;
    return;
  }
  model->SetWorldPose(pose);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixLinearVelocityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    _result = false;
    return;
  }
  math::Vector3 linvel;
  if (!ConvertVector(_req.pos(), linvel))
  {
    _result = false;
    return;
  }
  model->SetLinearVel(linvel);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAngularVelocityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    _result = false;
    return;
  }
  math::Vector3 angvel;
  if (!ConvertVector(_req.pos(), angvel))
  {
    _result = false;
    return;
  }
  model->SetAngularVel(angvel);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::ForceDurationThread(const physics::LinkPtr _link,
    const math::Vector3 &_force, float _duration)
{
  common::Time interval(1/this->world->GetPhysicsEngine()->GetRealTimeUpdateRate());
  if (_duration < 0)
  {
    // Negative is persistent application
    while (true)
    {
      _link->AddLinkForce(_force);
      // Try to sleep for the length of a physics timestep
      common::Time::Sleep(interval);
    }
    return;
  }

  // TODO: Treat duration as sim time, not wall time
  common::Time startTime = common::Time::GetWallTime();
  while ((common::Time::GetWallTime() - startTime).Float() < _duration)
  {
    _link->AddLinkForce(_force);
    // Try to sleep for the length of a physics timestep
    common::Time::Sleep(interval);
  }
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixForceCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  // TODO: Consider adding application point input argument for AddLinkForce call
  physics::LinkPtr link =
      this->world->GetModel(_req.name())->GetLink(_req.string_value());
  if (!link)
  {
    _result = false;
    return;
  }
  float duration = _req.float_value();
  math::Vector3 force;
  if (!ConvertVector(_req.pos(), force))
  {
    _result = false;
    return;
  }

  if (fabs(duration < 1e-6))
  {
    // 0 is impulse 
    link->AddLinkForce(force);
    _result = true;
    return;
  }

  // start a thread to call Link::AddLinkForce over a duration
  this->threadPool.push_back(std::thread(
    std::bind(&HaptixWorldPlugin::ForceDurationThread, this, link, force, duration)));

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::TorqueDurationThread(const physics::LinkPtr _link,
    const math::Vector3 &_torque, float _duration)
{
  common::Time interval(1/this->world->GetPhysicsEngine()->GetRealTimeUpdateRate());
  if (_duration < 0)
  {
    // Negative is persistent application
    while (true)
    {
      _link->AddTorque(_torque);
      // Try to sleep for the length of a physics timestep
      common::Time::Sleep(interval);
    }
    return;
  }

  // TODO: Treat duration as sim time, not wall time
  common::Time startTime = common::Time::GetWallTime();
  while ((common::Time::GetWallTime() - startTime).Float() < _duration)
  {
    _link->AddTorque(_torque);
    // Try to sleep for the length of a physics timestep
    common::Time::Sleep(interval);
  }
}


/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixTorqueCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  physics::LinkPtr link =
      this->world->GetModel(_req.name())->GetLink(_req.string_value());
  if (!link)
  {
    _result = false;
    return;
  }
  float duration = _req.float_value();
  math::Vector3 torque;
  if (!ConvertVector(_req.pos(), torque))
  {
    _result = false;
    return;
  }

  if (fabs(duration < 1e-6))
  {
    // 0 is impulse 
    link->AddTorque(torque);
    _result = true;
    return;
  }

  // start a thread to call Link::AddTorque over a duration
  this->threadPool.push_back(std::thread(
    std::bind(&HaptixWorldPlugin::TorqueDurationThread, this, link, torque, duration)));

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixResetCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxInt &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  // Signal to WorldControl to reset the world
  gazebo::msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  this->worldControlPub->Publish(msg);

  if (_req.data())
  {
    // Reset wrist and finger posture
    hxCommand command;
    memset(&command, 0, sizeof(command));
    command.ref_pos_enabled = 1;
    hxSensor sensor;
    if (hx_update(&command, &sensor) != ::hxOK)
      gzerr << "hx_update(): Request error.\n" << std::endl;

    gazebo::msgs::Int pause;
    pause.set_data(1);
    this->pausePub->Publish(pause);

    haptix::comm::msgs::hxCommand resp;
    haptix::comm::msgs::hxGrasp grasp;
    memset(&grasp, 0, sizeof(grasp));
    bool result;
    // And zero the grasp, if any.
    // TODO: check if grasp service exists?
    if(!this->ignNode.Request("haptix/gazebo/Grasp",
                              grasp,
                              1000,
                              resp,
                              result) || !result)
    {
      gzerr << "Failed to call gazebo/Grasp service" << std::endl;
    }
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixResetTimerCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!this->timerPublisher)
  {
    _result = false;
    return;
  }
  msgs::GzString msg;
  msg.set_data("reset");
  this->timerPublisher->Publish<msgs::GzString>(msg);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStartTimerCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!this->timerPublisher)
  {
    _result = false;
    return;
  }
  msgs::GzString msg;
  msg.set_data("start");
  this->timerPublisher->Publish<msgs::GzString>(msg);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStopTimerCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!this->timerPublisher)
  {
    _result = false;
    return;
  }
  msgs::GzString msg;
  msg.set_data("stop");
  this->timerPublisher->Publish<msgs::GzString>(msg);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixTimerCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxTime &_rep, bool &_result)
{
  gui::MainWindow *mainWindow = gui::get_main_window();
  if (!mainWindow)
  {
    _result = false;
    return;
  }

  gui::RenderWidget *renderWidget = mainWindow->GetRenderWidget();
  if (!renderWidget)
  {
    _result = false;
    return;
  }

  gui::GLWidget *glWidget = renderWidget->findChild<gui::GLWidget*>("GLWidget");
  if (!glWidget)
  {
    _result = false;
    return;
  }

  /*TimerGUIPlugin *timer = glWidget->findChild<TimerGUIPlugin*>("TimerGUIPlugin");
  if (!timer)
  {
    _result = false;
    return;
  }

  common::Time gzTime = timer->GetCurrentTime();
  _rep.set_sec(gzTime.sec);
  _rep.set_nsec(gzTime.nsec);*/

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStartLoggingCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!util::LogRecord::Instance())
  {
    _result = false;
    return;
  }

  // TODO: Encoding type? review interface with Carlos and Louise
  if (!util::LogRecord::Instance()->Start("zlib", _req.data()))
  {
    _result = false;
    return;
  }

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixIsLoggingCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxInt &_rep, bool &_result)
{
  if (!util::LogRecord::Instance())
  {
    _result = false;
    return;
  }
  _rep.set_data(util::LogRecord::Instance()->GetRunning());
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStopLoggingCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!util::LogRecord::Instance())
  {
    _result = false;
    return;
  }
  util::LogRecord::Instance()->Stop();
  _result = true;
}

bool HaptixWorldPlugin::ConvertTransform(
    const haptix::comm::msgs::hxTransform &_in, gazebo::math::Pose &_out)
{
  if (!_in.has_pos() || !_in.has_orient())
    return false;

  HaptixWorldPlugin::ConvertVector(_in.pos(), _out.pos);
  HaptixWorldPlugin::ConvertQuaternion(_in.orient(), _out.rot);
  return true;
}

bool HaptixWorldPlugin::ConvertTransform(const gazebo::math::Pose &_in,
    haptix::comm::msgs::hxTransform &_out)
{
  if (!_out.mutable_pos())
  {
    _out.set_allocated_pos(new haptix::comm::msgs::hxVector3());
  }
  if (!_out.mutable_orient())
  {
    _out.set_allocated_orient(new haptix::comm::msgs::hxQuaternion());
  }

  HaptixWorldPlugin::ConvertVector(_in.pos, *_out.mutable_pos());
  HaptixWorldPlugin::ConvertQuaternion(_in.rot, *_out.mutable_orient());
  return true;
}

bool HaptixWorldPlugin::ConvertVector(const haptix::comm::msgs::hxVector3 &_in,
    math::Vector3 &_out)
{
  if (!_in.has_x() || !_in.has_y() || !_in.has_z())
    return false;
  _out.Set(_in.x(), _in.y(), _in.z());
  return true;
}

bool HaptixWorldPlugin::ConvertVector(const math::Vector3 &_in,
    haptix::comm::msgs::hxVector3 &_out)
{
  _out.set_x(_in.x);
  _out.set_y(_in.y);
  _out.set_z(_in.z);
  return true;
}

bool HaptixWorldPlugin::ConvertQuaternion(const haptix::comm::msgs::hxQuaternion &_in,
    gazebo::math::Quaternion &_out)
{
  if (!_in.has_w() || !_in.has_x() || !_in.has_y() || !_in.has_z())
    return false;
  _out.Set(_in.w(), _in.x(), _in.y(), _in.z());
  return true;
}

bool HaptixWorldPlugin::ConvertQuaternion(const gazebo::math::Quaternion &_in,
    haptix::comm::msgs::hxQuaternion &_out)
{
  _out.set_w(_in.w);
  _out.set_x(_in.x);
  _out.set_y(_in.y);
  _out.set_z(_in.z);
  return true;
}

bool HaptixWorldPlugin::ConvertModel(const gazebo::physics::Model &_in,
  haptix::comm::msgs::hxModel &_out)
{
  _out.set_name(_in.GetName());

  math::Pose modelPose = _in.GetWorldPose();
  HaptixWorldPlugin::ConvertTransform(modelPose, *_out.mutable_transform());

  _out.set_id(_in.GetId());
  
  _out.clear_links();
  for (auto link : _in.GetLinks())
  {
    haptix::comm::msgs::hxLink *linkMsg = _out.add_links();
    ConvertLink(*link, *linkMsg);
  } 
  _out.clear_joints();
  for (auto joint : _in.GetJoints())
  {
    haptix::comm::msgs::hxJoint *jointMsg = _out.add_joints();
    ConvertJoint(*joint, *jointMsg);
  }
  return true;
}

bool HaptixWorldPlugin::ConvertLink(const gazebo::physics::Link &_in,
  haptix::comm::msgs::hxLink &_out)
{
  _out.set_name(_in.GetName());

  math::Pose linkPose = _in.GetWorldPose();
  HaptixWorldPlugin::ConvertTransform(linkPose, *(_out.mutable_transform()));

  math::Vector3 linVel = _in.GetWorldLinearVel();
  HaptixWorldPlugin::ConvertVector(linVel, *_out.mutable_linvel());

  math::Vector3 angVel = _in.GetWorldAngularVel();
  HaptixWorldPlugin::ConvertVector(angVel, *_out.mutable_angvel());

  math::Vector3 linAccel = _in.GetWorldLinearAccel();
  HaptixWorldPlugin::ConvertVector(linAccel, *_out.mutable_linacc());

  math::Vector3 angAccel = _in.GetWorldAngularAccel();
  HaptixWorldPlugin::ConvertVector(angAccel, *_out.mutable_angacc());
  return true;
}

bool HaptixWorldPlugin::ConvertJoint(gazebo::physics::Joint &_in,
  haptix::comm::msgs::hxJoint &_out)
{
  _out.set_name(_in.GetName());

  // TODO: Index???
  _out.set_pos(_in.GetAngle(0).Radian());
  _out.set_vel(_in.GetVelocity(0));

  _out.set_torque_passive(_in.GetForceTorque(0).body1Torque.GetLength());

  _out.set_torque_motor(_in.GetLinkTorque(0).GetLength());
  return true;
}
