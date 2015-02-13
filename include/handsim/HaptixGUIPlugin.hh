/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _HANDSIM_GUI_ARAT_PLUGIN_HH_
#define _HANDSIM_GUI_ARAT_PLUGIN_HH_

#include <boost/thread/mutex.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/transport.hh>
#include <haptix/comm/haptix.h>
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxGrasp.pb.h>

namespace haptix_gazebo_plugins
{
  // Forward declare task button
  class TaskButton;

  /// \brief A graphical interface for the HAPTIX project
  class HaptixGUIPlugin : public gazebo::GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: HaptixGUIPlugin();

    /// \brief Destructor
    public: virtual ~HaptixGUIPlugin();

    // Documentation inherited
    public: void Load(sdf::ElementPtr _elem);

    /// \brief Signal to set a contact visualization value.
    /// \param[in] _contactName Name of the contact sensor.
    /// \param[in] _value Force value.
    signals: void SetContactForce(QString _contactName, double _value);

    /// \brief Handles setting a contact visualization value.
    /// \param[in] _contactName Name of the contact sensor.
    /// \param[in] _value Force value.
    private slots: void OnSetContactForce(QString _contactName, double _value);

    /// \brief Handles the PreRender Gazebo signal
    private: void PreRender();

    /// \brief Callback triggered when a task button is pressed.
    /// \param[in] _id ID of the task.
    private slots: void OnTaskSent(const int _id);

    /// \brief Helper function to publish a task message
    /// \param[in] _taskName Name of the task to publish
    private: void PublishTaskMessage(const std::string &_taskName) const;

    /// \brief Helper function to publish a timer message
    /// \param[in] _msg Message to publish
    private: void PublishTimerMessage(const std::string &_msg) const;

    /// \brief Callback when the start/stop button is pressed.
    /// \param[in] _checked True if the button was checked.
    private slots: void OnStartStop(bool _checked);

    /// \brief Callback triggered when the next button is clicked
    private slots: void OnNextClicked();

    /// \brief Callback triggered when the reset all button is clicked
    private slots: void OnResetClicked();

    /// \brief Callback triggered when the reset scene button is clicked
    private slots: void OnResetSceneClicked();

    /// \brief Callback triggered when local frame checked is clicked.
    /// \param[in] _state State of the check box.
    private slots: void OnLocalCoordMove(int _state);

    /// \brief Helper function to initialize the task view
    /// \param[in] _elem SDF element pointer that contains HAPTIX task
    /// parameters.
    private: void InitializeTaskView(sdf::ElementPtr _elem);

    /// \brief Handle GUI keypresses
    /// \param[in] _event Key press event.
    private: bool OnKeyPress(gazebo::common::KeyEvent _event);

    /// \brief callback on Initialize hx connection
    private: void OnInitialize(ConstIntPtr &_msg);

    /// \brief Handle request responses
    /// \param[in] _msg Response message.
    private: void OnResponse(ConstResponsePtr &_msg);

    /// \brief Handle position scaling slider movement
    /// \param[in] _state State of the slider
    private slots: void OnScalingSlider(int _state);

    /// \brief Size of the contact sensor display rectangle, in pixels.
    private: gazebo::math::Vector2d defaultContactSize;

    /// \brief Minimum force value
    private: float forceMin;

    /// \brief Maximum force value
    private: float forceMax;

    /// \brief No force color value
    private: gazebo::common::Color colorNoContact;

    /// \brief Minimum force color value
    private: gazebo::common::Color colorMin;

    /// \brief Maximum force color value
    private: gazebo::common::Color colorMax;

    /// \brief Which hand is displayed (left, right)
    private: std::string handSide;

    /// \brief All the finger contact points.
    private: std::map<std::string, gazebo::math::Vector2d > contactPoints;

    /// \brief A map of contact sensor indices to human-readable names.
    private: std::map<int, std::string> contactNames;

    /// \brief The scene onto which is drawn the hand and contact
    /// force data
    private: QGraphicsScene *handScene;

    /// \brief Contact force visualization items.
    private: std::map<std::string, QGraphicsRectItem*>
             contactGraphicsItems;

    /// \brief initial camera pose
    private: gazebo::math::Pose initialCameraPose;

    /// \brief Node used to establish communication with gzserver.
    private: gazebo::transport::NodePtr node;

    /// \brief Publisher of requests.
    private: gazebo::transport::PublisherPtr requestPub;

    /// \brief Subscriber of respones.
    private: gazebo::transport::SubscriberPtr responseSub;

    /// \brief Publisher for signaling WorldControl node.
    private: gazebo::transport::PublisherPtr worldControlPub;

    // \brief Set of Gazebo signal connections.
    private: std::vector<gazebo::event::ConnectionPtr> connections;

    private: QTabWidget *taskTab;

    /// \brief Text box that hold instructions to the user.
    private: QTextEdit *instructionsView;

    /// \brief All the tasks in all the groups
    private: std::map<int, TaskButton*> taskList;

    /// \brief Number of the current task.
    private: int currentTaskId;

    /// \brief Publisher that talks with the arrange plugin to setup the
    /// scene.
    private: gazebo::transport::PublisherPtr taskPub;

    /// \brief Publisher that controls the clock
    private: gazebo::transport::PublisherPtr timerPub;

    /// \brief Task start/stop button
    private: QPushButton *startStopButton;

    /// \brief QT style for the start setting of the start/stop button
    private: std::string startStyle;

    /// \brief QT style for the start setting of the start/stop button
    private: std::string stopStyle;

    /// \brief A place to store key-to-motor mappings
    private: std::map<char, std::pair<unsigned int, float> > motorKeys;

    /// \brief A place to store key-to-arm mappings
    private: std::map<char, std::pair<unsigned int, float> > armKeys;

    /// \brief A place to store key-to-grasp mappings
    private: std::map<char, std::pair<std::string, float> > graspKeys;

    /// \brief An ignition node that we'll use for sending messages
    private: ignition::transport::Node ignNode;

    /// \brief Are we in grasp mode, or direct motor control mode?
    private: bool graspMode;

    /// \brief The last grasp request that we sent
    private: haptix::comm::msgs::hxGrasp lastGraspRequest;

    /// \brief The last motor command that we sent
    private: ::hxCommand lastMotorCommand;

    /// \brief The last sensor update that we received
    private: ::hxSensor lastSensor;

    /// \brief The device info returned by the other side
    private: ::hxRobotInfo robotInfo;

    /// \brief Have we initialized our information about the device?
    private: bool hxInitialized;

    /// \brief subscribe to initialize topic
    private: gazebo::transport::SubscriberPtr initializeSub;

    /// \brief The number of initial degrees of freedom that are in the wrist
    private: unsigned int numWristMotors;

    /// \brief Starting pose of the arm.
    private: gazebo::math::Pose armStartPose;

    /// \brief Request message used to get the initial arm pose.
    private: gazebo::msgs::Request *requestMsg;

    /// \brief When true, move in the arm's local coordinate frame.
    private: bool localCoordMove;

    /// \brief Position movement scaling factor.
    private: double posScalingFactor;

    /// \brief Publish command to pause motion tracking
    private: gazebo::transport::PublisherPtr pausePub;

    /// \brief Subscriber to pause status
    private: gazebo::transport::SubscriberPtr pauseSub;

    /// \brief Reset models, signal motion tracking to pause, reset hand state
    private: void ResetModels();

    /// \brief Callback for subscriber to pause response
    /// \param[in] _msg pause state
    private: void OnPauseRequest(ConstIntPtr &_msg);

    /// \brief was pausing motion tracking successful?
    private: bool trackingPaused;

    /// \brief a lock to hold when commanding wrist/finger positions
    private: boost::mutex motorCommandMutex;

    /// \brief start a thread to poll contact sensor data
    private: boost::thread pollSensorsThread;

    /// \brief start a thread to poll contact sensor data
    private: void PollSensors();

    /// \brief Get contact sensor information
    private: void UpdateSensorContact();

    /// \brief True if received a signal to quit
    private: bool quit;

    /// \brief subscribe to hydra
    private: gazebo::transport::SubscriberPtr hydraSub;

    /// \brief callback for subscriber to the hydra publisher
    private: void OnHydra(ConstHydraPtr &_msg);
  };
}
#endif
