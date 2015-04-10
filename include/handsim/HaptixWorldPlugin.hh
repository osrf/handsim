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
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

#include <ignition/transport.hh>
#include "haptix/comm/haptix_sim.h"
#include "haptix/comm/haptix.h"
#include "haptix/comm/msg/hxContactPoint.pb.h"
#include "haptix/comm/msg/hxContactPoint_V.pb.h"
#include "haptix/comm/msg/hxEmpty.pb.h"
#include "haptix/comm/msg/hxInt.pb.h"
#include "haptix/comm/msg/hxJoint.pb.h"
#include "haptix/comm/msg/hxLink.pb.h"
#include "haptix/comm/msg/hxModel.pb.h"
#include "haptix/comm/msg/hxQuaternion.pb.h"
#include "haptix/comm/msg/hxParam.pb.h"
#include "haptix/comm/msg/hxSimInfo.pb.h"
#include "haptix/comm/msg/hxString.pb.h"
#include "haptix/comm/msg/hxTransform.pb.h"
#include "haptix/comm/msg/hxTime.pb.h"
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

    private: void HaptixCameraTransformCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxTransform &_rep, bool &_result);

    private: void HaptixSetCameraTransformCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxTransform &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixContactPointsCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxString &_req,
      haptix::comm::msgs::hxContactPoint_V &_rep, bool &_result);

    private: void HaptixStateCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxModel &_req,
      haptix::comm::msgs::hxEmpty &_rep, bool &_result);

    private: void HaptixAddModelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxParam &_req,
      haptix::comm::msgs::hxModel &_rep, bool &_result);

    private: void HaptixRemoveModelCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxString &_req,
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

    private: void HaptixTimerCallback(
      const std::string &_service,
      const haptix::comm::msgs::hxEmpty &_req,
      haptix::comm::msgs::hxTime &_rep, bool &_result);

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
    /// \brief Apply a persistent force to a link over a certain time interval.
    /// \param[in] _link Link to apply the force to.
    /// \param[in] _force Force to apply.
    /// \param[in] _duration How long to apply the force. Negative to go on forever.
    private: void ForceDurationThread(const physics::LinkPtr _link,
        const math::Vector3 &_force, float _duration);

    /// \brief Apply a persistent torque to a link over a certain time interval.
    /// \param[in] _link Link to apply the torque to.
    /// \param[in] _torque Torque to apply.
    /// \param[in] _duration How long to apply the torque. Negative to go on forever.
    private: void TorqueDurationThread(const physics::LinkPtr _link,
        const math::Vector3 &_torque, float _duration);

    /// \brief Convert from hxTransform to Gazebo Pose
    /// \param[in] _in hxTransform to transform
    /// \param[out] _out Gazebo pose output
    /// \return True if the conversion succeeded.
    public: static bool ConvertTransform(const haptix::comm::msgs::hxTransform &_in,
        gazebo::math::Pose &_out);

    /// \brief Convert from Gazebo Pose to hxTransform
    /// \param[in] _in hxTransform to transform
    /// \param[out] _out Gazebo pose output
    /// \return True if the conversion succeeded.
    public: static bool ConvertTransform(const gazebo::math::Pose &_in,
        haptix::comm::msgs::hxTransform &_out);

    /// \brief Convert from hxVector3 to Gazebo Vector3
    /// \param[in] _in hxVector3 to transform
    /// \param[out] _out Gazebo Vector3 output
    /// \return True if the conversion succeeded.
    public: static bool ConvertVector(const haptix::comm::msgs::hxVector3 &_in,
        gazebo::math::Vector3 &_out);

    /// \brief Convert from Gazebo Vector3 to hxVector3
    /// \param[in] _in Gazebo Vector3 to transform
    /// \param[out] _out hxVector3 output
    /// \return True if the conversion succeeded.
    public: static bool ConvertVector(const gazebo::math::Vector3 &_in,
        haptix::comm::msgs::hxVector3 &_out);

    /// \brief Convert from hxQuaternion to Gazebo Quaternion
    public: static bool ConvertQuaternion(
        const haptix::comm::msgs::hxQuaternion &_in,
        gazebo::math::Quaternion &_out);

    /// \brief Convert from Gazebo Quaternion to hxQuaternion
    public: static bool ConvertQuaternion(const gazebo::math::Quaternion &_in,
        haptix::comm::msgs::hxQuaternion &_out);

    /// \brief Convert from Gazebo Model to hxModel
    public: static bool ConvertModel(const gazebo::physics::Model &_in,
        haptix::comm::msgs::hxModel &_out);

    /// \brief Convert from Gazebo Link to hxLink
    public: static bool ConvertLink(const gazebo::physics::Link &_in,
        haptix::comm::msgs::hxLink &_out);

    /// \brief Convert from Gazebo Joint to hxJoint
    public: static bool ConvertJoint(gazebo::physics::Joint &_in,
        haptix::comm::msgs::hxJoint &_out);

    ///////////// Member variables /////////////

    /// \brief World pointer.
    protected: physics::WorldPtr world;

    /// \brief SDF pointer.
    protected: sdf::ElementPtr sdf;

    /// \brief Node for Gazebo transport.
    protected: transport::NodePtr gzNode;

    /// \brief: ignition transport node for talking to haptix comm
    private: ignition::transport::Node ignNode;

    /// \brief For publishing commands to the GUI timer
    private: transport::PublisherPtr timerPublisher;

    /// \brief For publishing commands to the server
    private: transport::PublisherPtr worldControlPub;

    /// \brief For publishing pause commands
    private: transport::PublisherPtr pausePub;

    /// \brief Thread storage vector
    private: std::vector<std::thread> threadPool;
  };
}
#endif
