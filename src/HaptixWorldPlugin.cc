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
  this->timerPublisher->Fini();
  this->worldControlPub->Fini();

  event::Events::DisconnectWorldUpdateBegin(this->worldUpdateConnection);

  this->ignNode.Unadvertise("/haptix/gazebo/hxs_siminfo");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_camera_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_camera_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_contacts");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_state");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_add_model");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_remove_model");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_model_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_linear_velocity");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_angular_velocity");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_force");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_torque");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_reset");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_reset_timer");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_start_timer");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_stop_timer");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_timer");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_is_logging");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_start_logging");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_stop_logging");
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::Load(physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->sdf = _sdf;

  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init(this->world->GetName());
  // TODO: different timer topics?
  this->timerPublisher =
      this->gzNode->Advertise<msgs::GzString>("~/timer_control");
  this->worldControlPub = this->gzNode->Advertise<gazebo::msgs::WorldControl>
                              ("~/world_control");

  this->pausePub = this->gzNode->Advertise<gazebo::msgs::Int>
                      ("~/motion_tracking/pause_request");

  this->worldUpdateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HaptixWorldPlugin::OnWorldUpdate, this));

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

  this->ignNode.Advertise("/haptix/gazebo/hxs_remove_model",
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

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_gravity",
    &HaptixWorldPlugin::HaptixModelGravityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_gravity",
    &HaptixWorldPlugin::HaptixSetModelGravityCallback, this);

  this->lastSimUpdateTime = this->world->GetSimTime();
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
void HaptixWorldPlugin::OnWorldUpdate()
{
  common::Time elapsed = this->world->GetSimTime() - this->lastSimUpdateTime;
  for (auto iter = this->effortDurations.begin();
       iter != this->effortDurations.end();)
  {
    // Time elapsed since last update
    if (iter->timeRemaining < elapsed && !iter->persistent)
    {
      this->effortDurations.erase(iter);
    }
    else
    {
      iter->timeRemaining = iter->timeRemaining - elapsed;

      GZ_ASSERT(iter->link, "Link of forceDuration object was NULL!");

      if (iter->isForce)
      {
        iter->link->AddLinkForce(iter->effort);
      }
      else
      {
        iter->link->AddTorque(iter->effort);
      }

      ++iter;
    }
  }

  this->lastSimUpdateTime = this->world->GetSimTime();
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSimInfoCallback(const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxSimInfo &_rep, bool &_result)
{
  _rep.clear_models();
  // Get models
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was false in SimInfoCallback" << std::endl;
    _result = false;
    return;
  }

  this->modelVector = this->world->GetModels();

  for (auto &model : this->modelVector)
  {
    haptix::comm::msgs::hxModel* modelMsg = _rep.add_models();
    if (!ConvertModel(*model, *modelMsg))
    {
      _result = false;
      return;
    }
  }

  rendering::UserCameraPtr camera = gui::get_active_camera();
  if (!camera || !camera->GetScene())
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
  if (!camera || !camera->GetScene())
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
  if (!camera || !camera->GetScene())
  {
    _result = false;
    return;
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
  std::lock_guard<std::mutex> lock(this->worldMutex);

  if (!this->world)
  {
    _result = false;
    return;
  }

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
  // TODO: more feedback from sim api review?

  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }

  physics::ModelPtr model = this->world->GetModel(_req.name());

  if (!model)
  {
    gzerr << "Model was NULL: " << _req.name() << std::endl;
    _result = false;
    return;
  }

  for (int i = 0; i < _req.joints_size(); i++)
  {
    physics::JointPtr joint = model->GetJoint(_req.joints(i).name());
    if (!joint)
    {
      gzerr << "Joint was NULL: " << _req.joints(i).name() << std::endl;
      _result = false;
      return;
    }

    joint->SetPosition(0, _req.joints(i).pos());
    joint->SetVelocity(0, _req.joints(i).vel());
    gzdbg << "Gazebo got velocity of " <<_req.joints(i).name() << " as: " << joint->GetVelocity(0) << std::endl;
  }

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
  if (!modelSDF.Root() || !modelSDF.Root()->HasElement("model"))
  {
    gzdbg << "Model SDF was invalid" << std::endl;
    _result = false;
    return;
  }
  sdf::ElementPtr modelElement = modelSDF.Root()->GetElement("model");

  if (!modelElement || !modelElement->HasAttribute("name"))
  {
    gzdbg << "Model element invalid" << std::endl;
    _result = false;
    return;
  }

  // Set name
  modelElement->GetAttribute("name")->Set<std::string>(_req.name());

  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzdbg << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  xml = modelSDF.ToString();

  // load an SDF element from XML
  this->world->InsertModelString(xml);

  physics::ModelPtr model = this->world->GetModel(_req.name());

  int tries = 0;
  while (!model)
  {
    if (tries > 10)
    {
      gzdbg << "Model not found: " << _req.name() << std::endl;
      _result = false;
      return;
    }
    tries++;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    model = this->world->GetModel(_req.name());
  }

  math::Pose pose(_req.vector3().x(), _req.vector3().y(), _req.vector3().z(),
      _req.orientation().roll(), _req.orientation().pitch(),
      _req.orientation().yaw());

  // Set pose
  model->SetWorldPose(pose);

  bool gravity = _req.gravity();
  model->SetGravityMode(gravity);

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
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    _result = false;
    return;
  }

  physics::ModelPtr model = this->world->GetModel(_req.data());
  if (!model)
  {
    _result = true;
    return;
  }
  this->world->RemoveModel(model);

  // Wait while model still exists
  int tries = 0;
  while (this->world->GetModel(_req.data()))
  {
    _result = false;
    tries++;
    if (tries > 100)
      return;
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelTransformCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    _result = false;
    return;
  }

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
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    _result = false;
    return;
  }

  physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    _result = false;
    return;
  }
  math::Vector3 linvel;
  if (!ConvertVector(_req.vector3(), linvel))
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
  std::lock_guard<std::mutex> lock(this->worldMutex);
  physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!this->world)
  {
    _result = false;
    return;
  }

  if (!model)
  {
    _result = false;
    return;
  }
  math::Vector3 angvel;
  if (!ConvertVector(_req.vector3(), angvel))
  {
    _result = false;
    return;
  }
  model->SetAngularVel(angvel);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixForceCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    _result = false;
    return;
  }

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
  if (!ConvertVector(_req.vector3(), force))
  {
    _result = false;
    return;
  }

  if (fabs(duration) < 1e-6)
  {
    // 0 is impulse 
    link->AddLinkForce(force);
    _result = true;
    return;
  }

  this->effortDurations.push_back(EffortDuration(link, force, common::Time(duration),
      true, duration < 0));

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixTorqueCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    _result = false;
    return;
  }

  physics::LinkPtr link =
      this->world->GetModel(_req.name())->GetLink(_req.string_value());
  if (!link)
  {
    _result = false;
    return;
  }
  float duration = _req.float_value();
  math::Vector3 torque;
  if (!ConvertVector(_req.vector3(), torque))
  {
    _result = false;
    return;
  }

  if (fabs(duration) < 1e-6)
  {
    // 0 is impulse 
    link->AddTorque(torque);
    _result = true;
    return;
  }

  effortDurations.push_back(EffortDuration(link, torque,
      common::Time(duration), false, duration < 0));
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
    grasp.add_grasps();
    grasp.mutable_grasps(0)->set_grasp_value(0.0);

    bool result;
    // And zero the grasp, if any.
    // TODO: check if grasp service exists?
    if(!this->ignNode.Request("haptix/gazebo/Grasp",
                              grasp,
                              1000,
                              resp,
                              result) || !result)
    {
      gzwarn << "Failed to call gazebo/Grasp service" << std::endl;
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
  // TODO mutex for timerPublisher
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

  // TODO: link against plugin
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

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelGravityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxInt &_rep, bool &_result)
{
  if (!this->world)
  {
    _result = false;
    return;
  }

  physics::ModelPtr model = this->world->GetModel(_req.data());
  bool gravity = false;
  for (auto links : model->GetLinks())
  {
    gravity |= links->GetGravityMode();
  }

  _rep.set_data(gravity);

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetModelGravityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!this->world)
  {
    _result = false;
    return;
  }

  physics::ModelPtr model = this->world->GetModel(_req.name());

  model->SetGravityMode(_req.int_value());

  _result = true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertTransform(
    const haptix::comm::msgs::hxTransform &_in, gazebo::math::Pose &_out)
{
  if (!_in.has_pos() || !_in.has_orient())
    return false;

  HaptixWorldPlugin::ConvertVector(_in.pos(), _out.pos);
  HaptixWorldPlugin::ConvertQuaternion(_in.orient(), _out.rot);
  return true;
}

bool HaptixWorldPlugin::ConvertTransform(
    const hxTransform &_in, gazebo::math::Pose &_out)
{
  _out.pos.x = _in.pos.x;
  _out.pos.y = _in.pos.y;
  _out.pos.z = _in.pos.z;

  _out.rot.w = _in.orient.w;
  _out.rot.x = _in.orient.x;
  _out.rot.y = _in.orient.y;
  _out.rot.z = _in.orient.z;

  return true;
}

bool HaptixWorldPlugin::ConvertTransform(
    const gazebo::math::Pose &_in, hxTransform &_out)
{
  ConvertVector(_in.pos, _out.pos);

  _out.orient.w = _in.rot.w;
  _out.orient.x = _in.rot.x;
  _out.orient.y = _in.rot.y;
  _out.orient.z = _in.rot.z;

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
  if ((!_in.has_x()) || (!_in.has_y()) || (!_in.has_z()))
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

bool HaptixWorldPlugin::ConvertVector(const math::Vector3 &_in,
    hxVector3 &_out)
{
  _out.x = _in.x;
  _out.y = _in.y;
  _out.z = _in.z;

  return true;
}

bool HaptixWorldPlugin::ConvertVector(const hxVector3 &_in,
    math::Vector3 &_out)
{
  _out.x = _in.x;
  _out.y = _in.y;
  _out.z = _in.z;

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

  // Gravity mode is only false if all links have gravity set to false
  bool gravity = false;
  for (auto link : _in.GetLinks())
  {
    haptix::comm::msgs::hxLink *linkMsg = _out.add_links();
    ConvertLink(*link, *linkMsg);

    // Check gravity mode
    gravity |= link->GetGravityMode();
  }

  _out.set_gravity(gravity);

  _out.clear_joints();
  for (auto joint : _in.GetJoints())
  {
    haptix::comm::msgs::hxJoint *jointMsg = _out.add_joints();
    ConvertJoint(*joint, *jointMsg);
  }

  return true;
}

bool HaptixWorldPlugin::ConvertModel(const gazebo::physics::Model &_in,
    hxModel &_out)
{
  gzdbg << "Convert model to hxModel: " << _in.GetName() << std::endl;
  delete _out.name;
  _out.name = (char*) malloc(_in.GetName().length()+1);
  memset(_out.name, 0, _in.GetName().length()+1);
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  HaptixWorldPlugin::ConvertTransform(_in.GetWorldPose(), _out.transform);

  _out.id = _in.GetId();
  
  // Gravity mode is only false if all links have gravity set to false
  bool gravity = false;
  // TODO memsets
  int i = 0;
  for (auto link : _in.GetLinks())
  {
    ConvertLink(*link, _out.links[i]);
    i++;

    // Check gravity mode
    gravity |= link->GetGravityMode();
  }
  _out.link_count = i;

  _out.gravity = gravity;

  i = 0;
  for (auto joint : _in.GetJoints())
  {
    ConvertJoint(*joint, _out.joints[i]);
    i++;
  }
  _out.joint_count = i;

  return true;
}

bool HaptixWorldPlugin::ConvertLink(const gazebo::physics::Link &_in,
    haptix::comm::msgs::hxLink &_out)
{
  _out.set_name(_in.GetName());

  math::Pose linkPose = _in.GetWorldPose();
  ConvertTransform(linkPose, *(_out.mutable_transform()));

  math::Vector3 linVel = _in.GetWorldLinearVel();
  ConvertVector(linVel, *_out.mutable_linvel());

  math::Vector3 angVel = _in.GetWorldAngularVel();
  ConvertVector(angVel, *_out.mutable_angvel());

  math::Vector3 linAccel = _in.GetWorldLinearAccel();
  ConvertVector(linAccel, *_out.mutable_linacc());

  math::Vector3 angAccel = _in.GetWorldAngularAccel();
  ConvertVector(angAccel, *_out.mutable_angacc());
  return true;
}

bool HaptixWorldPlugin::ConvertLink(const gazebo::physics::Link &_in,
    hxLink &_out)
{
  delete _out.name;
  _out.name = (char*) malloc(_in.GetName().length()+1);
  memset(_out.name, 0, _in.GetName().length()+1);
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  ConvertTransform(_in.GetWorldPose(), _out.transform);

  ConvertVector(_in.GetWorldLinearVel(), _out.linvel);

  ConvertVector(_in.GetWorldAngularVel(), _out.angvel);

  ConvertVector(_in.GetWorldLinearAccel(), _out.linacc);

  ConvertVector(_in.GetWorldAngularAccel(), _out.angacc);

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

bool HaptixWorldPlugin::ConvertJoint(gazebo::physics::Joint &_in, hxJoint &_out)
{
  delete _out.name;
  _out.name = (char*) malloc(_in.GetName().length()+1);
  memset(_out.name, 0, _in.GetName().length()+1);
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  _out.pos = _in.GetAngle(0).Radian();
  _out.vel = _in.GetVelocity(0);
  _out.torque_passive = _in.GetForceTorque(0).body1Torque.GetLength();
  _out.torque_motor = _in.GetForceTorque(0).body1Torque.GetLength();

  return true;
}
