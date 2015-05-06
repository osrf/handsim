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
#include <vector>

#include <sdf/sdf.hh>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

#include <ignition/transport.hh>
#include "haptix/comm/haptix_sim.h"
#include "haptix/comm/haptix.h"
#include "haptix/comm/msg/hxCollideMode.pb.h"
#include "haptix/comm/msg/hxColor.pb.h"
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

class WrenchDuration
{
  public:
    gazebo::physics::LinkPtr link;
    gazebo::math::Vector3 force;
    gazebo::math::Vector3 torque;
    gazebo::common::Time timeRemaining;
    bool persistent;

    WrenchDuration() : link(NULL), persistent(false) {}
    WrenchDuration(gazebo::physics::LinkPtr _link,
        gazebo::math::Vector3 _force, gazebo::math::Vector3 _torque,
        gazebo::common::Time _duration, bool _persistent)
      : link(_link), force(_force), torque(_torque), timeRemaining(_duration),
        persistent(_persistent) {}
};

/// \brief Server for responding to the HAPTIX Sim API calls over Ignition
/// transport.
class HaptixWorldPlugin : public gazebo::WorldPlugin
{
  /// \brief Constructor.
  public: HaptixWorldPlugin();

  /// \brief Destructor.
  public: ~HaptixWorldPlugin();

  /// \brief Load the plugin.
  /// \param[in] _world Pointer to world
  /// \param[in] _sdf Pointer to the SDF configuration.
  public: virtual void Load(gazebo::physics::WorldPtr _world,
      sdf::ElementPtr _sdf);

  /// \brief Initialize the plugin.
  public: virtual void Init();

  /// \brief Reset the plugin.
  public: virtual void Reset();

  ///////////// Callback functions /////////////

  /// \brief hxs_sim_info callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSimInfoCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxSimInfo &_rep, bool &_result);

  /// \brief hxs_camera_transform callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixCameraTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxTransform &_rep, bool &_result);

  /// \brief hxs_set_camera_transform callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetCameraTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxTransform &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_contacts callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixContactPointsCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxContactPoint_V &_rep, bool &_result);

  /// \brief hxs_set_model_joint_state callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixModelJointStateCallback(const std::string &_service,
    const haptix::comm::msgs::hxModel &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_set_model_link_state callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixModelLinkStateCallback(const std::string &_service,
    const haptix::comm::msgs::hxModel &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_add_model callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixAddModelCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxModel &_rep, bool &_result);

  /// \brief hxs_remove_model callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixRemoveModelCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_transform callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetModelTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_transform callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixModelTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxTransform &_rep, bool &_result);

  /// \brief hxs_linear_velocity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixLinearVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxVector3 &_rep, bool &_result);

  /// \brief hxs_set_linear_velocity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetLinearVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_angular_velocity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixAngularVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxVector3 &_rep, bool &_result);

  /// \brief hxs_set_angular_velocity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetAngularVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_force callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixApplyForceCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_torque callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixApplyTorqueCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_wrench callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixApplyWrenchCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_reset callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixResetCallback(const std::string &_service,
    const haptix::comm::msgs::hxInt &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_start_logging callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixStartLoggingCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_is_logging callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixIsLoggingCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxInt &_rep, bool &_result);

  /// \brief hxs_stop_logging callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixStopLoggingCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_gravity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixModelGravityCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxInt &_rep, bool &_result);

  /// \brief hxs_set_model_gravity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetModelGravityCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_set_model_color callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetModelColorCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_color callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixModelColorCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxColor &_rep, bool &_result);

  /// \brief hxs_set_model_collide_mode callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixSetModelCollideModeCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_collide_mode callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request.
  /// \param[out] _rep The response.
  /// \param[out[ _result True if no errors were encountered.
  private: void HaptixModelCollideModeCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxCollideMode &_rep, bool &_result);

  /// \brief Callback on world update
  private: void OnWorldUpdate();

  /// \brief Callback on camera updates
  /// \param[in] _msg The viewpoint message that was received
  private: void OnUserCameraPose(ConstPosePtr &_msg);

  ///////////// Utility functions /////////////

  /// \brief Convert from hxsTransform (message type) to Gazebo Pose
  /// \param[in] _in hxsTransform to transform
  /// \param[out] _out Gazebo pose output
  /// \return True if the conversion succeeded.
  public: static bool ConvertTransform(
      const haptix::comm::msgs::hxTransform &_in, gazebo::math::Pose &_out);

  /// \brief Convert from hxsTransform to Gazebo Pose
  /// \param[in] _in hxsTransform to transform
  /// \param[out] _out Gazebo pose output
  /// \return True if the conversion succeeded.
  public: static bool ConvertTransform(
      const hxsTransform &_in, gazebo::math::Pose &_out);

  public: static bool ConvertTransform(
      const gazebo::math::Pose &_out, hxsTransform &_in);

  /// \brief Convert from Gazebo Pose to hxsTransform
  /// \param[in] _in hxsTransform to transform
  /// \param[out] _out Gazebo pose output
  /// \return True if the conversion succeeded.
  public: static bool ConvertTransform(
      const gazebo::math::Pose &_in, haptix::comm::msgs::hxTransform &_out);

  /// \brief Convert from hxsVector3 to Gazebo Vector3
  /// \param[in] _in hxsVector3 to transform
  /// \param[out] _out Gazebo Vector3 output
  /// \return True if the conversion succeeded.
  public: static bool ConvertVector(
      const haptix::comm::msgs::hxVector3 &_in, gazebo::math::Vector3 &_out);

  /// \brief Convert from Gazebo Vector3 to hxsVector3 (message type)
  /// \param[in] _in Gazebo Vector3 to transform
  /// \param[out] _out hxsVector3 output
  /// \return True if the conversion succeeded.
  public: static bool ConvertVector(
      const gazebo::math::Vector3 &_in, haptix::comm::msgs::hxVector3 &_out);

  /// \brief Convert from Gazebo Vector3 to hxsVector3
  /// \param[in] _in Gazebo Vector3 to transform
  /// \param[out] _out hxsVector3 output
  /// \return True if the conversion succeeded.
  public: static bool ConvertVector(
      const gazebo::math::Vector3 &_in, hxsVector3 &_out);

  /// \brief Convert from hxsVector3 to Gazebo Vector3
  /// \param[in] _in hxsVector3 to transform
  /// \param[out] _out Gazebo Vector3 output
  /// \return True if the conversion succeeded.
  public: static bool ConvertVector(const hxsVector3 &_in,
      gazebo::math::Vector3 &_out);

  /// \brief Convert from hxsQuaternion to Gazebo Quaternion
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertQuaternion(
      const haptix::comm::msgs::hxQuaternion &_in,
      gazebo::math::Quaternion &_out);

  /// \brief Convert from Gazebo Quaternion to hxsQuaternion
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertQuaternion(
      const gazebo::math::Quaternion &_in,
      haptix::comm::msgs::hxQuaternion &_out);

  /// \brief Convert from Gazebo Model to hxsModel (message type)
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertModel(gazebo::physics::Model &_in,
      haptix::comm::msgs::hxModel &_out);

  /// \brief Convert from Gazebo Model to hxsModel
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertModel(gazebo::physics::Model &_in,
      hxsModel &_out);

  /// \brief Convert from Gazebo Link to hxsLink (message type)
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertLink(const gazebo::physics::Link &_in,
      haptix::comm::msgs::hxLink &_out);

  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertLink(const gazebo::physics::Link &_in,
      hxsLink &_out);

  /// \brief Convert from Gazebo Joint to hxsJoint (message type)
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertJoint(gazebo::physics::Joint &_in,
      haptix::comm::msgs::hxJoint &_out);

  /// \brief Convert from Gazebo Joint to hxsJoint
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertJoint(gazebo::physics::Joint &_in,
      hxsJoint &_out);

  /// \brief Convert from Gazebo Wrench to hxsWrench
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertWrench(const gazebo::physics::JointWrench &_in,
      haptix::comm::msgs::hxWrench &_out);

  /// \brief Convert from Gazebo Wrench to hxsWrench (message type)
  /// \param[in] _in
  /// \param[out] _out
  public: static bool ConvertWrench(const gazebo::physics::JointWrench &_in,
      hxsWrench &_out);

  ///////////// Member variables /////////////

  /// \brief World pointer.
  protected: gazebo::physics::WorldPtr world;

  /// \brief SDF pointer.
  protected: sdf::ElementPtr sdf;

  /// \brief Vector of all the models in the world
  protected: gazebo::physics::Model_V modelVector;

  /// \brief World update connection for updating modelVector
  protected: gazebo::event::ConnectionPtr worldUpdateConnection;

  /// \brief Node for Gazebo transport.
  protected: gazebo::transport::NodePtr gzNode;

  /// \brief: ignition transport node for talking to haptix comm
  private: ignition::transport::Node ignNode;

  /// \brief For publishing commands to the GUI timer
  private: gazebo::transport::PublisherPtr timerPublisher;

  /// \brief For publishing commands to the server
  private: gazebo::transport::PublisherPtr worldControlPub;

  /// \brief For publishing pause commands
  private: gazebo::transport::PublisherPtr pausePub;

  /// \brief For publishing visual messages to ~/visual
  private: gazebo::transport::PublisherPtr visPub;

  /// \brief For publishing viewpoint messages to ~/user_camera/pose
  private: gazebo::transport::PublisherPtr userCameraPub;

  /// \brief For subscribing to viewpoint messages on ~/user_camera/pose
  private: gazebo::transport::SubscriberPtr userCameraSub;

  /// \brief For storing forces and torques applied over time.
  private: std::vector<WrenchDuration> wrenchDurations;

  /// \brief Maps model IDs to colors
  private: std::map<int, gazebo::common::Color> lastKnownColors;

  /// \brief Last time the effort vectors were updates
  private: gazebo::common::Time lastSimUpdateTime;

  /// \brief Mutex to protect the World pointer
  private: std::mutex worldMutex;

  /// \brief The initial camera pose, from SDF.
  private: gazebo::math::Pose initialCameraPose;

  /// \brief Last known viewpoint pose
  private: gazebo::math::Pose userCameraPose;

  /// \brief True if a user camera pose message has been published.
  private: bool userCameraPoseValid;
};

#endif
