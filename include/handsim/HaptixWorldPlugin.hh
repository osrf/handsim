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

#include <haptix/comm/haptix_sim.h>
#include <haptix/comm/haptix.h>
#include <haptix/comm/msg/hxCollideMode.pb.h>
#include <haptix/comm/msg/hxColor.pb.h>
#include <haptix/comm/msg/hxContactPoint.pb.h>
#include <haptix/comm/msg/hxContactPoint_V.pb.h>
#include <haptix/comm/msg/hxEmpty.pb.h>
#include <haptix/comm/msg/hxInt.pb.h>
#include <haptix/comm/msg/hxJoint.pb.h>
#include <haptix/comm/msg/hxLink.pb.h>
#include <haptix/comm/msg/hxModel.pb.h>
#include <haptix/comm/msg/hxQuaternion.pb.h>
#include <haptix/comm/msg/hxParam.pb.h>
#include <haptix/comm/msg/hxSimInfo.pb.h>
#include <haptix/comm/msg/hxString.pb.h>
#include <haptix/comm/msg/hxTransform.pb.h>
#include <haptix/comm/msg/hxTime.pb.h>
#include <haptix/comm/msg/hxVector3.pb.h>

#include <functional>
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

#include "handsim/WrenchHelper.hh"

/// \brief Class for representing a wrench applied to a link over a duration.
class WrenchDuration
{
  public:
    gazebo::physics::LinkPtr link;
    gazebo::WrenchHelper wrench;
    gazebo::common::Time timeRemaining;
    bool persistent;

    WrenchDuration() : persistent(false) {}
    WrenchDuration(gazebo::physics::LinkPtr _link, gazebo::WrenchHelper _wrench,
        gazebo::common::Time _duration, bool _persistent)
      : link(_link), wrench(_wrench), timeRemaining(_duration),
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

  /// \brief Reset the plugin.
  public: virtual void Reset();

  ///////////// Utility functions /////////////

  /// \brief Convert from hxsTransform (message type) to Gazebo Pose
  /// \param[in] _in hxsTransform to convert
  /// \param[out] _out Gazebo pose output
  public: static void ConvertTransform(
      const haptix::comm::msgs::hxTransform &_in, gazebo::math::Pose &_out);

  /// \brief Convert from Gazebo Pose to hxsTransform
  /// \param[in] _in Gazebo pose to convert
  /// \param[out] _out hxsTransform output
  public: static void ConvertTransform(
      const gazebo::math::Pose &_in, haptix::comm::msgs::hxTransform &_out);

  /// \brief Convert from hxsVector3 to Gazebo Vector3
  /// \param[in] _in hxsVector3 to convert
  /// \param[out] _out Gazebo Vector3 output
  public: static void ConvertVector(
      const haptix::comm::msgs::hxVector3 &_in, gazebo::math::Vector3 &_out);

  /// \brief Convert from Gazebo Vector3 to hxsVector3 (message type)
  /// \param[in] _in Gazebo Vector3 to convert
  /// \param[out] _out hxsVector3 message output
  public: static void ConvertVector(
      const gazebo::math::Vector3 &_in, haptix::comm::msgs::hxVector3 &_out);

  /// \brief Convert from hxsQuaternion (message) to Gazebo Quaternion
  /// \param[in] _in hxsQuaternion message to convert
  /// \param[out] _out Gazebo quaternion output
  public: static void ConvertQuaternion(
      const haptix::comm::msgs::hxQuaternion &_in,
      gazebo::math::Quaternion &_out);

  /// \brief Convert from Gazebo Quaternion to hxsQuaternion (message)
  /// \param[in] _in Gazebo quaternion to convert
  /// \param[out] _out hxsQuaternion output
  public: static void ConvertQuaternion(
      const gazebo::math::Quaternion &_in,
      haptix::comm::msgs::hxQuaternion &_out);

  /// \brief Convert from Gazebo Model to hxsModel (message type)
  /// \param[in] _in Gazebo model to convert
  /// \param[out] _out hxsModel message output
  public: static void ConvertModel(const gazebo::physics::Model &_in,
      haptix::comm::msgs::hxModel &_out);

  /// \brief Convert from Gazebo Link to hxsLink (message type)
  /// \param[in] _in Gazebo link to convert
  /// \param[out] _out hxLink message output
  public: static void ConvertLink(const gazebo::physics::Link &_in,
      haptix::comm::msgs::hxLink &_out);

  /// \brief Convert from Gazebo Joint to hxsJoint (message type)
  /// \param[in] _in Gazebo joint to convert
  /// \param[out] _out hxJoint message output
  public: static void ConvertJoint(gazebo::physics::Joint &_in,
      haptix::comm::msgs::hxJoint &_out);

  /// \brief Convert from Gazebo Wrench to hxsWrench
  /// \param[in] _in Gazebo wrench to convert
  /// \param[out] _out hxWrench message output
  public: static void ConvertWrench(const gazebo::physics::JointWrench &_in,
      haptix::comm::msgs::hxWrench &_out);

  ///////////// Protected helpers  //////////////
  /// \brief Read the initial colors of each model from SDF.
  protected: void InitializeColorMap();

  ///////////// Callback functions /////////////

  /// \brief hxs_sim_info callback. Iterate through all Gazebo models in the
  /// world and pack the relevant information in the reply.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request (empty).
  /// \param[out] _rep The reply: an hxSimInfo message containing model and
  /// camera information.
  /// \return _result True if no errors were encountered.
  private: void HaptixSimInfoCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxSimInfo &_rep, bool &_result);

  /// \brief hxs_camera_transform callback. Get the UserCameraPose cached from
  /// the OnUserCameraPose callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request (empty).
  /// \param[out] _rep The reply: a transform representing the current
  /// camera pose.
  /// \return _result True if no errors were encountered.
  private: void HaptixCameraTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxTransform &_rep, bool &_result);

  /// \brief hxs_set_camera_transform callback. Publishes to the camera pose
  /// topic.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: desired camera transform.
  /// \param[out] _rep The reply (empty).
  /// \return _result True if no errors were encountered.
  private: void HaptixSetCameraTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxTransform &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_contacts callback. Get contact information from the Gazebo
  /// CollisionManager.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: name of the model to check contact points.
  /// \param[out] _rep The reply: a vector of contact points representing where
  /// the model is contacting other objects.
  /// \return _result True if no errors were encountered.
  private: void HaptixContactsCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxContactPoint_V &_rep, bool &_result);

  /// \brief hxs_model_joint_state callback. Set the state of a joint.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: the name of the model to query.
  /// \param[out] _rep The reply: joint states
  /// \return _result True if no errors were encountered.
  private: void HaptixModelJointStateCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxModel &_rep, bool &_result);

  /// \brief hxs_set_model_joint_state callback. Set the state of a joint.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: hxModel message containing the name of the
  /// model to set and the desired joint state.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetModelJointStateCallback(const std::string &_service,
    const haptix::comm::msgs::hxModel &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_set_model_link_state callback. Set the state of a joint.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: hxModel message containing the name of the
  /// model to set and the desired link state.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixModelLinkStateCallback(const std::string &_service,
    const haptix::comm::msgs::hxModel &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_add_model callback. Add a model during runtime given its SDF
  /// and its initial position.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an hxParam message containing the name of
  /// the model to add, its SDF representation, its initial transform, and its
  /// gravity mode.
  /// \param[out] _rep The reply: the sim API representation of the model as
  /// an hxModel message.
  /// \return _result True if no errors were encountered.
  private: void HaptixAddModelCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxModel &_rep, bool &_result);

  /// \brief hxs_remove_model callback. Remove a model from the world.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: the name of the model to remove.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixRemoveModelCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_set_model_transform callback. Set the pose of a model.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an hxParam message containing the name of
  /// the model to set and the desired transform.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetModelTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_transform callback. Get the pose of a model.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: the name of the model to query.
  /// \param[out] _rep The reply: the model's current pose.
  /// \return _result True if no errors were encountered.
  private: void HaptixModelTransformCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxTransform &_rep, bool &_result);

  /// \brief hxs_linear_velocity callback. Get the linear velocity of a model.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: the name of the model to query.
  /// \param[out] _rep The reply: The current linear velocity of the model.
  /// \return _result True if no errors were encountered.
  private: void HaptixLinearVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxVector3 &_rep, bool &_result);

  /// \brief hxs_set_linear_velocity callback. Instantaneously set the linear
  /// velocity of a model and all its links.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: An hxParam message containing the name of
  /// the model and a Vector3 representing the desired linear velocity.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetLinearVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_angular_velocity callback. Get the angular velocity of the
  /// canonical link of a model.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: name of the model to query.
  /// \param[out] _rep The reply: Vector3 representing the angular velocity of
  /// the model.
  /// \return _result True if no errors were encountered.
  private: void HaptixAngularVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxVector3 &_rep, bool &_result);

  /// \brief hxs_set_angular_velocity callback. Instantaneously set the angular
  /// velocity of a model and all its links.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: An hxParam message containing the name of
  /// the model and a Vector3 representing the desired angular velocity in the
  /// world frame.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetAngularVelocityCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_apply_force callback. Apply a force to a model over a duration.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an hxParam message containing the name of
  /// the model which will receive the force, a Vector3 representing the force
  /// to apply, and a duration in seconds.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixApplyForceCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_apply_torque callback. Apply a torque to a model over a
  /// duration.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an hxParam message containing the name of
  /// the model which will receive the torque, a Vector3 representing the torque
  /// to apply, and a duration in seconds.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixApplyTorqueCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_apply_wrench callback. Apply a wrench to a model over a
  /// duration.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an hxParam message containing the name of
  /// the model which will receive the torque, the wrench to apply, and a
  /// duration in seconds.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixApplyWrenchCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_reset callback. Reset the scene. Simulates the "Reset" button
  /// in the HAPTIX GUI.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an integer. If 0, do not reset the pose
  /// of the limb. Else, reset the pose of the limb.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixResetCallback(const std::string &_service,
    const haptix::comm::msgs::hxInt &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_start_logging callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: filename to save the log to.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixStartLoggingCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_is_logging callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: empty.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixIsLoggingCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxInt &_rep, bool &_result);

  /// \brief hxs_stop_logging callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: empty.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixStopLoggingCallback(const std::string &_service,
    const haptix::comm::msgs::hxEmpty &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_gravity callback. Get the gravity mode of the model.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: name of the model to query.
  /// \param[out] _rep The reply: 1 if gravity on, 0 if gravity off.
  /// \return _result True if no errors were encountered.
  private: void HaptixModelGravityCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxInt &_rep, bool &_result);

  /// \brief hxs_set_model_gravity callback.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: an hxParam message containing the name of
  /// the model and the desired gravity mode (1 if gravity on, 0 if off).
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetModelGravityCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_set_model_color callback. Set the color of the model. The
  /// diffuse and ambient components of each link material will be set.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: hxParam message containing the name of the
  /// model and the desired color.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetModelColorCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_color callback. Get the color of the model averaged over
  /// the diffuse and ambient components of each link material.
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: name of the model to query.
  /// \param[out] _rep The reply: average color of the model.
  /// \return _result True if no errors were encountered.
  private: void HaptixModelColorCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxColor &_rep, bool &_result);

  /// \brief hxs_set_model_collide_mode callback. Set the collide mode of the
  /// model: COLLIDE, NO_COLLIDE, or DETECTION_ONLY (collide without contact).
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: hxParam message containing the name of the
  /// model and the desired collide mode.
  /// \param[out] _rep The reply: empty.
  /// \return _result True if no errors were encountered.
  private: void HaptixSetModelCollideModeCallback(const std::string &_service,
    const haptix::comm::msgs::hxParam &_req,
    haptix::comm::msgs::hxEmpty &_rep, bool &_result);

  /// \brief hxs_model_collide_mode callback. Get the collide mode of the
  /// model: COLLIDE, NO_COLLIDE, or DETECTION_ONLY (collide without contact).
  /// \param[in] _service The service this callback is advertised on.
  /// \param[in] _req The request: name of the model.
  /// \param[out] _rep The reply: collide mode of the model.
  /// \return _result True if no errors were encountered.
  private: void HaptixModelCollideModeCallback(const std::string &_service,
    const haptix::comm::msgs::hxString &_req,
    haptix::comm::msgs::hxCollideMode &_rep, bool &_result);

  /// \brief Callback on world update
  private: void OnWorldUpdate();

  /// \brief Callback on camera updates
  /// \param[in] _msg The viewpoint message that was received
  private: void OnUserCameraPose(ConstPosePtr &_msg);

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

  /// \brief ignition transport node for talking to haptix comm
  private: ignition::transport::Node ignNode;

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

  /// \brief The functions to execute in OnWorldUpdate
  private: std::vector<std::function<void()>> updateFunctions;

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
