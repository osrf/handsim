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
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/util/LogRecord.hh>
#include <gazebo/plugins/TimerGUIPlugin.hh>

#include "haptix/comm/msg/hxCommand.pb.h"
#include "haptix/comm/msg/hxGrasp.pb.h"

#include "handsim/HaptixWorldPlugin.hh"

GZ_REGISTER_WORLD_PLUGIN(HaptixWorldPlugin)

/////////////////////////////////////////////////
HaptixWorldPlugin::HaptixWorldPlugin()
{
  this->userCameraPoseValid = false;
}

/////////////////////////////////////////////////
HaptixWorldPlugin::~HaptixWorldPlugin()
{
  this->worldControlPub->Fini();

  gazebo::event::Events::DisconnectWorldUpdateBegin(
      this->worldUpdateConnection);

  this->ignNode.Unadvertise("/haptix/gazebo/hxs_sim_info");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_camera_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_camera_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_contacts");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_model_joint_state");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_model_link_state");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_add_model");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_remove_model");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_model_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_model_transform");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_linear_velocity");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_linear_velocity");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_angular_velocity");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_angular_velocity");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_apply_force");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_apply_torque");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_apply_wrench");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_reset");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_is_logging");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_start_logging");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_stop_logging");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_model_gravity_mode");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_model_gravity_mode");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_model_color");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_model_color");
  this->ignNode.Unadvertise("/haptix/gazebo/hxs_set_model_collide_mode");
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::InitializeColorMap()
{
  // Initialize color map.
  for (auto model : this->world->GetModels())
  {
    // A model might have multiple links, a link might have multiple visuals
    // with different colors
    // whatever man, this is probably not a typical case for teams

    // numVecs: the number of vectors we average over.
    // For a model with m visuals, numVecs will be 2*m
    // because each visual has two color vectors, ambient and diffuse
    int numVecs = 0;
    double r, g, b, a;
    r = g = b = a = 0;

    for (auto link : model->GetLinks())
    {
      // Get all the visuals
      sdf::ElementPtr linkSDF = link->GetSDF();
      GZ_ASSERT(linkSDF != NULL, "Got link with NULL SDF pointer in init");
      if (linkSDF->HasElement("visual"))
      {
        for (sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
             visualSDF; visualSDF = linkSDF->GetNextElement("visual"))
        {
          GZ_ASSERT(visualSDF->HasAttribute("name"),
              "Malformed visual element!");
          if (!visualSDF->HasElement("material"))
            continue;
          sdf::ElementPtr materialSDF = visualSDF->GetElement("material");
          if (!materialSDF->HasElement("ambient") &&
              !materialSDF->HasElement("diffuse"))
          {
            continue;
          }
          gazebo::common::Color ambient =
              materialSDF->GetElement("ambient")->Get<gazebo::common::Color>();
          gazebo::common::Color diffuse =
              materialSDF->GetElement("diffuse")->Get<gazebo::common::Color>();

          r += ambient.r;
          g += ambient.g;
          b += ambient.b;
          a += ambient.a;

          r += diffuse.r;
          g += diffuse.g;
          b += diffuse.b;
          a += diffuse.a;
          numVecs+=2;
        }
      }
    }
    if (numVecs == 0)
      numVecs = 1;
    gazebo::common::Color color(r / numVecs, g / numVecs, b / numVecs, a / numVecs);
    this->lastKnownColors[model->GetId()] = color;
  }
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  this->world = _world;
  GZ_ASSERT(this->world != NULL, "Got NULL world pointer!");
  this->sdf = _sdf;
  GZ_ASSERT(this->sdf != NULL, "Got NULL SDF element pointer!");

  this->InitializeColorMap();

  this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gzNode->Init(this->world->GetName());

  this->worldControlPub = this->gzNode->Advertise<gazebo::msgs::WorldControl>
                              ("~/world_control");

  this->pausePub = this->gzNode->Advertise<gazebo::msgs::Int>
                      ("~/motion_tracking/pause_request");

  this->visPub = this->gzNode->Advertise<gazebo::msgs::Visual>("~/visual");

  this->userCameraPoseValid = false;

  this->userCameraPub =
      this->gzNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

  this->userCameraSub = this->gzNode->Subscribe("~/user_camera/pose",
      &HaptixWorldPlugin::OnUserCameraPose, this);

  this->lastSimUpdateTime = this->world->GetSimTime();

  this->worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&HaptixWorldPlugin::OnWorldUpdate, this));

  // Advertise haptix sim services.
  this->ignNode.Advertise("/haptix/gazebo/hxs_sim_info",
    &HaptixWorldPlugin::HaptixSimInfoCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_camera_transform",
    &HaptixWorldPlugin::HaptixCameraTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_camera_transform",
    &HaptixWorldPlugin::HaptixSetCameraTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_contacts",
    &HaptixWorldPlugin::HaptixContactPointsCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_joint_state",
    &HaptixWorldPlugin::HaptixModelJointStateCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_link_state",
    &HaptixWorldPlugin::HaptixModelLinkStateCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_add_model",
    &HaptixWorldPlugin::HaptixAddModelCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_remove_model",
    &HaptixWorldPlugin::HaptixRemoveModelCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_transform",
    &HaptixWorldPlugin::HaptixSetModelTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_transform",
    &HaptixWorldPlugin::HaptixModelTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_linear_velocity",
    &HaptixWorldPlugin::HaptixSetLinearVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_linear_velocity",
    &HaptixWorldPlugin::HaptixLinearVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_angular_velocity",
    &HaptixWorldPlugin::HaptixSetAngularVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_angular_velocity",
    &HaptixWorldPlugin::HaptixAngularVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_apply_force",
    &HaptixWorldPlugin::HaptixApplyForceCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_apply_torque",
    &HaptixWorldPlugin::HaptixApplyTorqueCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_apply_wrench",
    &HaptixWorldPlugin::HaptixApplyWrenchCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_reset",
    &HaptixWorldPlugin::HaptixResetCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_is_logging",
    &HaptixWorldPlugin::HaptixIsLoggingCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_start_logging",
    &HaptixWorldPlugin::HaptixStartLoggingCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_stop_logging",
    &HaptixWorldPlugin::HaptixStopLoggingCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_gravity_mode",
    &HaptixWorldPlugin::HaptixModelGravityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_gravity_mode",
    &HaptixWorldPlugin::HaptixSetModelGravityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_color",
    &HaptixWorldPlugin::HaptixModelColorCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_color",
    &HaptixWorldPlugin::HaptixSetModelColorCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_collide_mode",
    &HaptixWorldPlugin::HaptixModelCollideModeCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_set_model_collide_mode",
    &HaptixWorldPlugin::HaptixSetModelCollideModeCallback, this);
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::Reset()
{
  this->wrenchDurations.clear();
  this->InitializeColorMap();
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::OnWorldUpdate()
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  gazebo::common::Time elapsed =
      this->world->GetSimTime() - this->lastSimUpdateTime;
  for (auto iter = this->wrenchDurations.begin();
       iter != this->wrenchDurations.end();)
  {
    // Time elapsed since last update
    if (iter->timeRemaining < elapsed && !iter->persistent)
    {
      this->wrenchDurations.erase(iter);
    }
    else
    {
      iter->timeRemaining = iter->timeRemaining - elapsed;

      GZ_ASSERT(iter->link, "Link of WrenchDuration object was NULL!");

      iter->link->AddLinkForce(iter->force);
      iter->link->AddTorque(iter->torque);

      ++iter;
    }
  }

  for (auto function : this->updateFunctions)
  {
    function();
  }
  this->updateFunctions.clear();

  this->lastSimUpdateTime = this->world->GetSimTime();
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::OnUserCameraPose(ConstPosePtr &_msg)
{
  this->userCameraPose = gazebo::msgs::Convert(*_msg);
  if (!this->userCameraPoseValid)
  {
    this->initialCameraPose = this->userCameraPose;
  }
  this->userCameraPoseValid = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSimInfoCallback(const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxSimInfo &_rep, bool &_result)
{
  _rep.Clear();

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
      gzerr << "Couldn't convert Gazebo model to model message" << std::endl;
      _result = false;
      return;
    }
  }

  gazebo::math::Pose pose = this->userCameraPose;
  if (!this->userCameraPoseValid)
  {
    gzwarn << "User camera pose has not yet been published. Returning default"
           << " camera pose specified in SDF." << std::endl;
  }

  haptix::comm::msgs::hxTransform *cameraTransform =
      new haptix::comm::msgs::hxTransform;
  _rep.set_allocated_camera_transform(cameraTransform);
  if (!HaptixWorldPlugin::ConvertTransform(pose,
          *_rep.mutable_camera_transform()))
  {
    gzerr << "Couldn't convert Gazebo pose to transform message" << std::endl;
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
  gazebo::math::Pose pose = this->userCameraPose;
  if (!this->userCameraPoseValid)
  {
    gzwarn << "User camera pose has not yet been published. Returning default"
           << " camera pose specified in SDF." << std::endl;
  }

  if (!HaptixWorldPlugin::ConvertTransform(pose, _rep))
  {
    gzerr << "Couldn't convert Gazebo pose to transform message" << std::endl;
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
  gazebo::math::Pose pose;

  if (!HaptixWorldPlugin::ConvertTransform(_req, pose))
  {
    gzerr << "Couldn't convert transform message to Gazebo pose" << std::endl;
    _result = false;
    return;
  }
  gazebo::msgs::Pose poseMsg;
  gazebo::msgs::Set(&poseMsg, pose);
  this->userCameraPub->Publish(poseMsg);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixContactPointsCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxContactPoint_V &_rep, bool &_result)
{
  if (!_req.has_data())
  {
    gzerr << "String request did not have data field!" << std::endl;
    _result = false;
    return;
  }
  std::string modelName = _req.data();
  std::lock_guard<std::mutex> lock(this->worldMutex);

  if (!this->world)
  {
    gzerr << "NULL world in ContactPoints callback" << std::endl;
    _result = false;
    return;
  }

  std::vector<gazebo::physics::Contact*> contacts =
      this->world->GetPhysicsEngine()->GetContactManager()->GetContacts();
  for (auto contact : contacts)
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

      gazebo::math::Vector3 linkPosition =
          contact->collision1->GetLink()->GetWorldPose().pos;

      // All vectors are relative to the link frame.
      ConvertVector(contact->positions[i] - linkPosition,
          *contactMsg->mutable_point());

      ConvertVector(contact->normals[i] - linkPosition,
          *contactMsg->mutable_normal());

      // force is always body1 onto body2
      // Don't need to subtract link pos, force and torque are in link frame
      /*ConvertVector(contact->wrench[i].body2Force,
          *contactMsg->mutable_force());
      ConvertVector(contact->wrench[i].body2Torque,
          *contactMsg->mutable_torque());*/
      ConvertWrench(contact->wrench[i], *contactMsg->mutable_wrench());
      contactMsg->set_distance(contact->depths[i]);
    }
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelJointStateCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxModel &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());

  if (!model)
  {
    gzerr << "Model was NULL: " << _req.name() << std::endl;
    _result = false;
    return;
  }

  if (_req.joints_size() < 1)
  {
    gzerr << "Not enough joints in callback" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::JointPtr joint = model->GetJoint(_req.joints(0).name());
  if (!joint)
  {
    gzerr << "Joint was NULL: " << _req.joints(0).name() << std::endl;
    _result = false;
    return;
  }
  float pos = _req.joints(0).pos();
  float vel = _req.joints(0).vel();
  auto setJointStateLambda = [joint, pos, vel]()
      {
        joint->SetPosition(0, pos);
        joint->SetVelocity(0, vel);
      };
  this->updateFunctions.push_back(setJointStateLambda);

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelLinkStateCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxModel &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());

  if (!model)
  {
    gzerr << "Model was NULL: " << _req.name() << std::endl;
    _result = false;
    return;
  }

  if (_req.links_size() < 1)
  {
    gzerr << "Not enough links in callback" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::LinkPtr link = model->GetLink(_req.links(0).name());
  if (!link)
  {
    gzerr << "Link was NULL: " << _req.links(0).name() << std::endl;
    _result = false;
    return;
  }

  gazebo::math::Pose pose;
  gazebo::math::Vector3 lin_vel;
  gazebo::math::Vector3 ang_vel;

  if (!ConvertTransform(_req.links(0).transform(), pose))
  {
    gzerr << "Couldn't convert link transform" << std::endl;
    _result = false;
    return;
  }
  if (!ConvertVector(_req.links(0).lin_vel(), lin_vel))
  {
    gzerr << "Couldn't convert linear vel vector" << std::endl;
    _result = false;
    return;
  }
  if (!ConvertVector(_req.links(0).ang_vel(), ang_vel))
  {
    gzerr << "Couldn't convert linear vel vector" << std::endl;
    _result = false;
    return;
  }

  auto setLinkStateLambda = [link, &pose, &lin_vel, &ang_vel]()
    {
      link->SetWorldPose(pose);
      /*link->SetLinearVel(lin_vel);
      link->SetAngularVel(ang_vel);*/
    };
  this->updateFunctions.push_back(setLinkStateLambda);

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAddModelCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxModel &_rep, bool &_result)
{
  // TODO Due to reply, can't call this in a thread-safe way, discuss.
  if (!_req.has_string_value())
  {
    gzerr << "Missing string field in hxParam" << std::endl;
    _result = false;
    return;
  }

  if (!_req.has_name())
  {
    gzerr << "Missing name field in hxParam" << std::endl;
    _result = false;
    return;
  }

  if (!_req.has_vector3())
  {
    gzerr << "Missing vector3 field in hxParam" << std::endl;
    _result = false;
    return;
  }

  if (!_req.has_orientation())
  {
    gzerr << "Missing orientation field in hxParam" << std::endl;
    _result = false;
    return;
  }

  if (!_req.has_gravity_mode())
  {
    gzerr << "Missing gravity_mode field in hxParam" << std::endl;
    _result = false;
    return;
  }

  std::string xml = _req.string_value();
  sdf::SDF modelSDF;
  modelSDF.SetFromString(xml);
  if (!modelSDF.Root() || !modelSDF.Root()->HasElement("model"))
  {
    gzerr << "Model SDF was invalid" << std::endl;
    _result = false;
    return;
  }
  sdf::ElementPtr modelElement = modelSDF.Root()->GetElement("model");

  if (!modelElement || !modelElement->HasAttribute("name"))
  {
    gzerr << "Model element invalid" << std::endl;
    _result = false;
    return;
  }

  // Set name
  modelElement->GetAttribute("name")->Set<std::string>(_req.name());

  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  xml = modelSDF.ToString();

  // load an SDF element from XML
  this->world->InsertModelString(xml);

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());

  int tries = 0;
  while (!model)
  {
    if (tries > 200)
    {
      gzerr << "Model not found: " << _req.name() << std::endl;
      _result = false;
      return;
    }
    tries++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    model = this->world->GetModel(_req.name());
  }

  gazebo::math::Pose pose(_req.vector3().x(), _req.vector3().y(),
      _req.vector3().z(), _req.orientation().roll(), _req.orientation().pitch(),
      _req.orientation().yaw());

  bool gravity_mode = _req.gravity_mode();

  model->SetWorldPose(pose);
  model->SetGravityMode(gravity_mode);

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
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = true;
    return;
  }
  // TODO Due to reply, can't call this in a thread-safe way. Discuss.
  this->world->RemoveModel(model);

  // Could get rid wait period and not notify user if failed to remove
  // Wait while model still exists
  int tries = 0;
  while (this->world->GetModel(_req.data()))
  {
    _result = false;
    tries++;
    if (tries > 200)
    {
      gzerr << "hxs_remove_model timed out, model still exists" << std::endl;
      _result = false;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelTransformCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxTransform &_rep, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::math::Pose pose = model->GetWorldPose();
  if (!ConvertTransform(pose, _rep))
  {
    gzerr << "Couldn't convert Gazebo pose to transform message" << std::endl;
    _result = false;
    return;
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetModelTransformCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }

  if (!_req.has_transform())
  {
    gzerr << "Missing transform field in hxParam" << std::endl;
    _result = false;
    return;
  }

  gazebo::math::Pose pose;
  if (!ConvertTransform(_req.transform(), pose))
  {
    gzerr << "Couldn't convert transform message to Gazebo pose" << std::endl;
    _result = false;
    return;
  }
  auto setModelTransformLambda = [model, &pose]()
      {
        model->SetWorldPose(pose);
      };
  this->updateFunctions.push_back(setModelTransformLambda);
  _result = true;
}


/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixLinearVelocityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxVector3 &_rep, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::math::Vector3 lin_vel = model->GetWorldLinearVel();
  if (!ConvertVector(lin_vel, _rep))
  {
    gzerr << "Couldn't convert Gazebo Vector3 to message" << std::endl;
    _result = false;
    return;
  }
  _result = true;
}
/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetLinearVelocityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_vector3())
  {
    gzerr << "Missing vector3 field in hxParam" << std::endl;
  }
  if (!_req.has_name())
  {
    gzerr << "Missing name field in hxParam" << std::endl;
  }

  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::math::Vector3 lin_vel;
  if (!ConvertVector(_req.vector3(), lin_vel))
  {
    gzerr << "Couldn't convert message to Gazebo Vector3" << std::endl;
    _result = false;
    return;
  }
  auto setLinearVelocityLambda = [model, &lin_vel] ()
    {
      model->SetLinearVel(lin_vel);
    };
  this->updateFunctions.push_back(setLinearVelocityLambda);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAngularVelocityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxVector3 &_rep, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());
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
  gazebo::math::Vector3 ang_vel = model->GetWorldAngularVel();
  if (!ConvertVector(ang_vel, _rep))
  {
    _result = false;
    return;
  }
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetAngularVelocityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_vector3())
  {
    gzerr << "Missing vector3 field in hxParam" << std::endl;
  }
  if (!_req.has_name())
  {
    gzerr << "Missing name field in hxParam" << std::endl;
  }
  std::lock_guard<std::mutex> lock(this->worldMutex);
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
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
  gazebo::math::Vector3 ang_vel;
  if (!ConvertVector(_req.vector3(), ang_vel))
  {
    _result = false;
    return;
  }
  auto setAngularVelocityLambda = [model, &ang_vel] ()
    {
      model->SetAngularVel(ang_vel);
    };
  this->updateFunctions.push_back(setAngularVelocityLambda);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixApplyForceCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_vector3())
  {
    gzerr << "Missing vector3 field in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_name())
  {
    gzerr << "Missing name field in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_float_value())
  {
    gzerr << "Missing float field in hxParam" << std::endl;
    _result = false;
    return;
  }

  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::LinkPtr link = model->GetLink(_req.string_value());
  if (!link)
  {
    gzerr << "Link pointer NULL" << std::endl;
    _result = false;
    return;
  }
  float duration = _req.float_value();
  gazebo::math::Vector3 force;
  if (!ConvertVector(_req.vector3(), force))
  {
    gzerr << "Couldn't convert vector message to Gazebo Vector3" << std::endl;
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

  this->wrenchDurations.push_back(WrenchDuration(link, force,
      gazebo::math::Vector3::Zero, gazebo::common::Time(duration),
      duration < 0));

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixApplyTorqueCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_vector3())
  {
    gzerr << "Missing vector3 field in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_name())
  {
    gzerr << "Missing name field in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_float_value())
  {
    gzerr << "Missing float field in hxParam" << std::endl;
    _result = false;
    return;
  }
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::LinkPtr link = model->GetLink(_req.string_value());
  if (!link)
  {
    gzerr << "Link pointer NULL" << std::endl;
    _result = false;
    return;
  }
  float duration = _req.float_value();
  gazebo::math::Vector3 torque;
  if (!ConvertVector(_req.vector3(), torque))
  {
    gzerr << "Couldn't convert vector message to Gazebo Vector3" << std::endl;
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

  this->wrenchDurations.push_back(WrenchDuration(link,
      gazebo::math::Vector3::Zero, torque, gazebo::common::Time(duration),
      duration < 0));
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixApplyWrenchCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_wrench())
  {
    gzerr << "Missing wrench field in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_name())
  {
    gzerr << "Missing name field in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_float_value())
  {
    gzerr << "Missing float field in hxParam" << std::endl;
    _result = false;
    return;
  }
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    gzerr << "Model pointer NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::LinkPtr link = model->GetLink(_req.string_value());
  if (!link)
  {
    gzerr << "Link pointer NULL" << std::endl;
    _result = false;
    return;
  }

  float duration = _req.float_value();
  gazebo::math::Vector3 force;
  gazebo::math::Vector3 torque;

  if (!ConvertVector(_req.wrench().force(), force))
  {
    gzerr << "Couldn't convert vector message to Gazebo Vector3" << std::endl;
    _result = false;
    return;
  }

  if (!ConvertVector(_req.wrench().torque(), torque))
  {
    gzerr << "Couldn't convert vector message to Gazebo Vector3" << std::endl;
    _result = false;
    return;
  }

  if (fabs(duration) < 1e-6)
  {
    // 0 is impulse
    link->AddForce(force);
    link->AddTorque(torque);
    _result = true;
    return;
  }

  this->wrenchDurations.push_back(WrenchDuration(link, force,
      torque, gazebo::common::Time(duration), duration < 0));
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixResetCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxInt &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  GZ_ASSERT(this->userCameraPub != NULL, "Camera publisher was NULL!");
  gazebo::msgs::Pose poseMsg;
  gazebo::msgs::Set(&poseMsg, this->initialCameraPose);
  this->userCameraPub->Publish(poseMsg);

  if (_req.data() != 0)
  {
    // Signal to WorldControl to reset the world
    gazebo::msgs::WorldControl msg;
    msg.mutable_reset()->set_model_only(true);
    this->worldControlPub->Publish(msg);

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
    if (!this->ignNode.Request("haptix/gazebo/Grasp", grasp, 1000, resp,
            result) || !result)
    {
      gzwarn << "Failed to call gazebo/Grasp service" << std::endl;
    }
  }
  else
  {
    for (auto model : world->GetModels())
    {
      if (model->GetName() != "mpl_haptix_right_forearm")
      {
        model->Reset();
      }
    }
  }

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStartLoggingCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!gazebo::util::LogRecord::Instance())
  {
    gzerr << "Log recorder was NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::util::LogRecord::Instance()->Init(_req.data());

  // TODO: Encoding type?
  if (!gazebo::util::LogRecord::Instance()->Start("zlib", ""))
  {
    gzerr << "Failed to start recording" << std::endl;
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
  if (!gazebo::util::LogRecord::Instance())
  {
    gzerr << "Log recorder was NULL" << std::endl;
    _result = false;
    return;
  }
  _rep.set_data(gazebo::util::LogRecord::Instance()->GetRunning());
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStopLoggingCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxEmpty &/*_req*/,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!gazebo::util::LogRecord::Instance())
  {
    gzerr << "Log recorder was NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::util::LogRecord::Instance()->Stop();
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelGravityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxInt &_rep, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());
  if (!model)
  {
    gzerr << "Model was NULL" << std::endl;
    _result = false;
    return;
  }
  bool gravity_mode = false;
  for (auto links : model->GetLinks())
  {
    gravity_mode |= links->GetGravityMode();
  }

  _rep.set_data(gravity_mode);

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetModelGravityCallback(
      const std::string &/*_service*/,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_name())
  {
    gzerr << "Missing required field name in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_gravity_mode())
  {
    gzerr << "Missing required field gravity_mode in hxParam" << std::endl;
    _result = false;
    return;
  }

  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());
  if (!model)
  {
    gzerr << "Model was NULL" << std::endl;
    _result = false;
    return;
  }

  bool gravity_mode = _req.gravity_mode();
  auto setModelGravityLambda = [model, gravity_mode] ()
      {
        model->SetGravityMode(gravity_mode);
      };
  this->updateFunctions.push_back(setModelGravityLambda);

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetModelColorCallback(
    const std::string &/*_service*/,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_name())
  {
    gzerr << "Missing required field name in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_color())
  {
    gzerr << "Missing required field color in hxParam" << std::endl;
    _result = false;
    return;
  }
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());

  if (!model)
  {
    gzerr << "Model was NULL" << std::endl;
    _result = false;
    return;
  }

  gazebo::common::Color newColor(_req.color().r(), _req.color().g(),
      _req.color().b(), _req.color().alpha());
  gazebo::msgs::Color *colorMsg =
      new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
  gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);
  for (auto link : model->GetLinks())
  {
    // Get all the visuals
    sdf::ElementPtr linkSDF = link->GetSDF();

    if (!linkSDF)
    {
      gzerr << "Link had NULL SDF" << std::endl;
      _result = false;
      return;
    }
    if (linkSDF->HasElement("visual"))
    {
      for (sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
           visualSDF; visualSDF = linkSDF->GetNextElement("visual"))
      {
        GZ_ASSERT(visualSDF->HasAttribute("name"), "Malformed visual element!");
        std::string visualName = visualSDF->Get<std::string>("name");
        gazebo::msgs::Visual visMsg;
        visMsg = link->GetVisualMessage(visualName);
        if ((!visMsg.has_material()) || visMsg.mutable_material() == NULL)
        {
          gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
          visMsg.set_allocated_material(materialMsg);
        }
        gazebo::msgs::Material *materialMsg = visMsg.mutable_material();
        if (materialMsg->has_ambient())
        {
          materialMsg->clear_ambient();
        }
        materialMsg->set_allocated_ambient(colorMsg);
        if (materialMsg->has_diffuse())
        {
          materialMsg->clear_diffuse();
        }
        visMsg.set_name(link->GetScopedName());
        visMsg.set_parent_name(model->GetScopedName());
        materialMsg->set_allocated_diffuse(diffuseMsg);
        visPub->Publish(visMsg);
      }
    }
  }

  this->lastKnownColors[model->GetId()] = newColor;

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelColorCallback(
    const std::string &/*_service*/,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxColor &_rep, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());

  if (!model)
  {
    gzerr << "Model was NULL" << std::endl;
    _result = false;
    return;
  }

  auto links = model->GetLinks();
  if (links.size() == 0)
  {
    gzerr << "Model has no links, can't set color!" << std::endl;
    _result = false;
    return;
  }

  gazebo::common::Color modelColor = this->lastKnownColors[model->GetId()];
  _rep.set_r(modelColor.r);
  _rep.set_b(modelColor.b);
  _rep.set_g(modelColor.g);
  _rep.set_alpha(modelColor.a);
  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixSetModelCollideModeCallback(
    const std::string &/*_service*/,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &/*_rep*/, bool &_result)
{
  if (!_req.has_name())
  {
    gzerr << "Missing required field name in hxParam" << std::endl;
    _result = false;
    return;
  }
  if (!_req.has_collide_mode())
  {
    gzerr << "Missing required field collide_mode in hxParam" << std::endl;
    _result = false;
    return;
  }
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.name());

  if (!model)
  {
    gzerr << "Model was NULL" << std::endl;
    _result = false;
    return;
  }

  auto mode = _req.collide_mode().mode();
  if (mode != haptix::comm::msgs::hxCollideMode::hxsNOCOLLIDE &&
      mode != haptix::comm::msgs::hxCollideMode::hxsCOLLIDE &&
      mode != haptix::comm::msgs::hxCollideMode::hxsDETECTIONONLY)
  {
    gzerr << "Unknown hxsCollideMode, cannot set" << std::endl;
    _result = false;
    return;
  }

  // Bit ridiculous, could put this in its own function
  auto setCollideModeLambda = [model, mode] ()
      {
        for (auto link : model->GetLinks())
        {
          for (auto collision : link->GetCollisions())
          {
            gazebo::physics::SurfaceParamsPtr surface = collision->GetSurface();

            if (mode == haptix::comm::msgs::hxCollideMode::hxsNOCOLLIDE)
            {
              surface->collideWithoutContact = false;
              // Set collideBitmask in case it was unset
              surface->collideBitmask = 0x0;
            }
            else if (mode ==
                haptix::comm::msgs::hxCollideMode::hxsDETECTIONONLY)
            {
              surface->collideWithoutContact = true;
              // Set collideBitmask in case it was unset
              if (surface->collideBitmask == 0x0)
              {
                surface->collideBitmask = 0x01;
              }
            }
            else if (mode == haptix::comm::msgs::hxCollideMode::hxsCOLLIDE)
            {
              surface->collideWithoutContact = false;
              surface->collideBitmask = 0x01;
            }
            else
            {
              gzerr << "Unknown collision mode inside lambda. This should "
                    << "never happen" << std::endl;
            }
          }
        }
      };
  this->updateFunctions.push_back(setCollideModeLambda);

  _result = true;
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelCollideModeCallback(
    const std::string &/*_service*/,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxCollideMode &_rep, bool &_result)
{
  std::lock_guard<std::mutex> lock(this->worldMutex);
  if (!this->world)
  {
    gzerr << "World was NULL" << std::endl;
    _result = false;
    return;
  }
  gazebo::physics::ModelPtr model = this->world->GetModel(_req.data());

  if (!model)
  {
    gzerr << "Model was NULL" << std::endl;
    _result = false;
    return;
  }
  bool collideWithoutContact = true;
  unsigned int totalCollideBitmask = 0x0;

  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      gazebo::physics::SurfaceParamsPtr surface = collision->GetSurface();
      if (!surface)
      {
        gzerr << "Surface was NULL" << std::endl;
        _result = false;
        return;
      }
      collideWithoutContact &= surface->collideWithoutContact;
      totalCollideBitmask |= surface->collideBitmask;
    }
  }

  if (totalCollideBitmask == 0x0)
  {
    // All of the collisions had a collide_bitmask of 0x0
    _rep.set_mode(haptix::comm::msgs::hxCollideMode::hxsNOCOLLIDE);
  }
  else if (collideWithoutContact)
  {
    // All of the collisions had collideWithoutContact = true
    _rep.set_mode(haptix::comm::msgs::hxCollideMode::hxsDETECTIONONLY);
  }
  else
  {
    _rep.set_mode(haptix::comm::msgs::hxCollideMode::hxsCOLLIDE);
  }

  _result = true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::HaptixWorldPlugin::ConvertTransform(
    const haptix::comm::msgs::hxTransform &_in, gazebo::math::Pose &_out)
{
  if (!_in.has_pos() || !_in.has_orient())
    return false;

  HaptixWorldPlugin::ConvertVector(_in.pos(), _out.pos);
  HaptixWorldPlugin::ConvertQuaternion(_in.orient(), _out.rot);
  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertTransform(
    const hxsTransform &_in, gazebo::math::Pose &_out)
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

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertTransform(
    const gazebo::math::Pose &_in, hxsTransform &_out)
{
  ConvertVector(_in.pos, _out.pos);

  _out.orient.w = _in.rot.w;
  _out.orient.x = _in.rot.x;
  _out.orient.y = _in.rot.y;
  _out.orient.z = _in.rot.z;

  return true;
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertVector(const haptix::comm::msgs::hxVector3 &_in,
    gazebo::math::Vector3 &_out)
{
  if ((!_in.has_x()) || (!_in.has_y()) || (!_in.has_z()))
    return false;
  _out.Set(_in.x(), _in.y(), _in.z());
  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertVector(const gazebo::math::Vector3 &_in,
    haptix::comm::msgs::hxVector3 &_out)
{
  _out.set_x(_in.x);
  _out.set_y(_in.y);
  _out.set_z(_in.z);
  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertVector(const gazebo::math::Vector3 &_in,
    hxsVector3 &_out)
{
  _out.x = _in.x;
  _out.y = _in.y;
  _out.z = _in.z;

  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertVector(const hxsVector3 &_in,
    gazebo::math::Vector3 &_out)
{
  _out.x = _in.x;
  _out.y = _in.y;
  _out.z = _in.z;

  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertQuaternion(
    const haptix::comm::msgs::hxQuaternion &_in, gazebo::math::Quaternion &_out)
{
  if (!_in.has_w() || !_in.has_x() || !_in.has_y() || !_in.has_z())
    return false;
  _out.Set(_in.w(), _in.x(), _in.y(), _in.z());
  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertQuaternion(const gazebo::math::Quaternion &_in,
    haptix::comm::msgs::hxQuaternion &_out)
{
  _out.set_w(_in.w);
  _out.set_x(_in.x);
  _out.set_y(_in.y);
  _out.set_z(_in.z);
  return true;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertModel(gazebo::physics::Model &_in,
    haptix::comm::msgs::hxModel &_out)
{
  _out.set_name(_in.GetName());

  gazebo::math::Pose modelPose = _in.GetWorldPose();
  HaptixWorldPlugin::ConvertTransform(modelPose, *_out.mutable_transform());

  _out.set_id(_in.GetId());

  _out.clear_links();
  bool result = true;

  // Gravity mode is only false if all links have gravity_mode set to false
  bool gravity_mode = false;
  for (auto link : _in.GetLinks())
  {
    haptix::comm::msgs::hxLink *linkMsg = _out.add_links();
    result &= ConvertLink(*link, *linkMsg);

    // Check gravity_mode mode
    gravity_mode |= link->GetGravityMode();
  }

  _out.set_gravity_mode(gravity_mode);

  _out.clear_joints();
  for (auto joint : _in.GetJoints())
  {
    haptix::comm::msgs::hxJoint *jointMsg = _out.add_joints();
    result &= ConvertJoint(*joint, *jointMsg);
  }

  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertModel(gazebo::physics::Model &_in,
    hxsModel &_out)
{
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  HaptixWorldPlugin::ConvertTransform(_in.GetWorldPose(), _out.transform);

  _out.id = _in.GetId();

  // Gravity mode is only false if all links have gravity_mode set to false
  bool gravity_mode = false;
  bool result = true;
  int i = 0;
  for (auto link : _in.GetLinks())
  {
    result &= ConvertLink(*link, _out.links[i]);
    i++;

    // Check gravity_mode mode
    gravity_mode |= link->GetGravityMode();
  }
  _out.link_count = i;

  _out.gravity_mode = gravity_mode;

  i = 0;
  for (auto joint : _in.GetJoints())
  {
    result &= ConvertJoint(*joint, _out.joints[i]);
    i++;
  }
  _out.joint_count = i;

  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertLink(const gazebo::physics::Link &_in,
    haptix::comm::msgs::hxLink &_out)
{
  _out.set_name(_in.GetName());

  bool result = true;
  gazebo::math::Pose linkPose = _in.GetWorldPose();
  result &= ConvertTransform(linkPose, *(_out.mutable_transform()));

  gazebo::math::Vector3 linVel = _in.GetWorldLinearVel();
  result &= ConvertVector(linVel, *_out.mutable_lin_vel());

  gazebo::math::Vector3 angVel = _in.GetWorldAngularVel();
  result &= ConvertVector(angVel, *_out.mutable_ang_vel());

  gazebo::math::Vector3 linAccel = _in.GetWorldLinearAccel();
  result &= ConvertVector(linAccel, *_out.mutable_lin_acc());

  gazebo::math::Vector3 angAccel = _in.GetWorldAngularAccel();
  result &= ConvertVector(angAccel, *_out.mutable_ang_acc());
  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertLink(const gazebo::physics::Link &_in,
    hxsLink &_out)
{
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());
  bool result = true;
  result &= ConvertTransform(_in.GetWorldPose(), _out.transform);

  result &= ConvertVector(_in.GetWorldLinearVel(), _out.lin_vel);

  result &= ConvertVector(_in.GetWorldAngularVel(), _out.ang_vel);

  result &= ConvertVector(_in.GetWorldLinearAccel(), _out.lin_acc);

  result &= ConvertVector(_in.GetWorldAngularAccel(), _out.ang_acc);

  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertJoint(gazebo::physics::Joint &_in,
    haptix::comm::msgs::hxJoint &_out)
{
  _out.set_name(_in.GetName());

  _out.set_pos(_in.GetAngle(0).Radian());
  _out.set_vel(_in.GetVelocity(0));

  bool result = true;
  result &= ConvertWrench(_in.GetForceTorque(0),
      *_out.mutable_wrench_reactive());

  _out.set_torque_motor(_in.GetForce(0));
  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertJoint(gazebo::physics::Joint &_in,
    hxsJoint &_out)
{
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  _out.pos = _in.GetAngle(0).Radian();
  _out.vel = _in.GetVelocity(0);
  bool result = true;
  result &= ConvertWrench(_in.GetForceTorque(0), _out.wrench_reactive);
  _out.torque_motor = _in.GetForce(0);

  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertWrench(const gazebo::physics::JointWrench &_in,
    haptix::comm::msgs::hxWrench &_out)
{
  bool result = true;
  result &= ConvertVector(_in.body2Force, *_out.mutable_force());
  result &= ConvertVector(_in.body2Torque, *_out.mutable_torque());
  return result;
}

/////////////////////////////////////////////////
bool HaptixWorldPlugin::ConvertWrench(const gazebo::physics::JointWrench &_in,
    hxsWrench &_out)
{
  bool result = true;
  result &= ConvertVector(_in.body2Force, _out.force);
  result &= ConvertVector(_in.body2Torque, _out.torque);
  return result;
}
