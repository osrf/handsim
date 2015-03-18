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

#ifndef _HANDSIM_HAPTIX_WORLD_PLUGIN_HH_
#define _HANDSIM_HAPTIX_WORLD_HH_

#include <map>
#include <string>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
//#include <gazebo/transport/transport.hh>
#include <gazebo/math/Pose.hh>

#include <ignition/transport.hh>
#include "haptix/comm/haptix_sim.h"
#include "haptix/comm/haptix.h"
#include "haptix/comm/msg/hxCamera.pb.h"
#include "haptix/comm/msg/hxContact.pb.h"
#include "haptix/comm/msg/hxContact_V.pb.h"
#include "haptix/comm/msg/hxEmpty.pb.h"
#include "haptix/comm/msg/hxInt.pb.h"
#include "haptix/comm/msg/hxJacobian.pb.h"
#include "haptix/comm/msg/hxJoint.pb.h"
#include "haptix/comm/msg/hxLink.pb.h"
#include "haptix/comm/msg/hxModel.pb.h"
#include "haptix/comm/msg/hxQuaternion.pb.h"
#include "haptix/comm/msg/hxParam.pb.h"
#include "haptix/comm/msg/hxSimInfo.pb.h"
#include "haptix/comm/msg/hxString.pb.h"
#include "haptix/comm/msg/hxTransform.pb.h"
#include "haptix/comm/msg/hxVector3.pb.h"

namespace gazebo
{
  class GAZEBO_VISIBLE HaptixWorldPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: HaptixWorldPlugin();

    /// \brief Destructor.
    public: ~HaptixWorldPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    ///////////// Callback functions /////////////

    private: void HaptixSimInfoCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxSimInfo &_rep, bool &_result);

    private: void HaptixCameraCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxCamera &_rep, bool &_result);

    private: void HaptixCameraTransformCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxTransform &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixContactsCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxContact_V &_rep, bool &_result);

    private: void HaptixJacobianCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxJacobian &_rep, bool &_result);

    private: void HaptixStateCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixAddModelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxModel &_rep, bool &_result);

    private: void HaptixRemoveModelIDCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxInt &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixModelTransformCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixLinearVelocityCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixAngularVelocityCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixLinearAccelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixAngularAccelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixForceCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixTorqueCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixResetCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxInt &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixResetTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixStartTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixStopTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixStartLoggingCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixIsLoggingCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxInt &_rep, bool &_result);

    private: void HaptixStopLoggingCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    ///////////// Utility functions /////////////

    /// \brief Convert from hxTransform to Gazebo Pose
    /// \param[in] _in hxTransform to transform
    /// \param[out] _out Gazebo pose output
    /// \return True if the conversion succeeded.
    private: bool ConvertTransform(const haptix::comm::hxTransform &_in,
        gazebo::math::Pose &_out) const;

    /// \brief Convert from Gazebo Pose to hxTransform
    /// \param[in] _in hxTransform to transform
    /// \param[out] _out Gazebo pose output
    /// \return True if the conversion succeeded.
    private: bool ConvertTransform(const gazebo::math::Pose &_in,
        haptix::comm::hxTransform &_out) const;

    /// \brief Convert from hxVector3 to Gazebo Vector3
    /// \param[in] _in hxVector3 to transform
    /// \param[out] _out Gazebo Vector3 output
    /// \return True if the conversion succeeded.
    private: bool ConvertVector(const haptix::comm::hxVector3 &_in,
        gazebo::math::Vector3 &_out) const;

    /// \brief Convert from Gazebo Vector3 to hxVector3
    /// \param[in] _in Gazebo Vector3 to transform
    /// \param[out] _out hxVector3 output
    /// \return True if the conversion succeeded.
    private: bool ConvertVector(const gazebo::math::Vector3 &_in,
        haptix::comm::hxVector3 &_out) const;

    /// \brief Convert from hxQuaternion to Gazebo Quaternion

    /// \brief Convert from Gazebo Quaternion to hxQuaternion


    ///////////// Member variables /////////////

    /// \brief World pointer.
    protected: physics::WorldPtr world;

    /// \brief SDF pointer.
    protected: sdf::ElementPtr sdf;

    /// \brief: ignition transport node for talking to haptix comm
    private: ignition::transport::Node ignNode;

    /// \brief Node for Gazebo transport.
    //protected: transport::NodePtr node;

    /// \brief Subscriber for listening to changing arrangements.
    //protected: transport::SubscriberPtr sub;
  };
}
#endif
