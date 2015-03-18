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
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
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

  // Advertise haptix sim services.
  this->ignNode.Advertise("/haptix/gazebo/hxs_siminfo",
    &HaptixWorldPlugin::HaptixSimInfoCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_camera",
    &HaptixWorldPlugin::HaptixCameraCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_camera_transform",
    &HaptixWorldPlugin::HaptixCameraTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_contacts",
    &HaptixWorldPlugin::HaptixContactsCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_jacobian",
    &HaptixWorldPlugin::HaptixJacobianCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_state",
    &HaptixWorldPlugin::HaptixStateCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_add_model",
    &HaptixWorldPlugin::HaptixAddModelCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_remove_model_id",
    &HaptixWorldPlugin::HaptixRemoveModelIDCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_model_transform",
    &HaptixWorldPlugin::HaptixModelTransformCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_linear_velocity",
    &HaptixWorldPlugin::HaptixLinearVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_angular_velocity",
    &HaptixWorldPlugin::HaptixAngularVelocityCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_linear_accel",
    &HaptixWorldPlugin::HaptixLinearAccelCallback, this);

  this->ignNode.Advertise("/haptix/gazebo/hxs_angular_accel",
    &HaptixWorldPlugin::HaptixAngularAccelCallback, this);

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
void HaptixWorldPlugin::HaptixSimInfoCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxSimInfo &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixCameraCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxCamera &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixCameraTransformCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxTransform &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixContactsCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxContact_V &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixJacobianCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxJacobian &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStateCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAddModelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxModel &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixRemoveModelIDCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxInt &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixModelTransformCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixLinearVelocityCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAngularVelocityCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixLinearAccelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixAngularAccelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixForceCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixTorqueCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixResetCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxInt &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixResetTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStartTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStopTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStartLoggingCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixIsLoggingCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxInt &_rep, bool &_result)
{
}

/////////////////////////////////////////////////
void HaptixWorldPlugin::HaptixStopLoggingCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result)
{
}
