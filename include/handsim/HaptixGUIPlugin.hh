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
#include <haptix/comm/haptix_sim.h>
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

    /// \brief Qt callback on mouse enter event.
    /// \param[in] _event Mouse enter event.
    protected: virtual void enterEvent(QEvent *_event);

    /// \brief Qt callback on mouse move event.
    /// \param[in] _event Mouse move event.
    protected: virtual void mouseMoveEvent(QMouseEvent *_event);

    /// \brief Qt callback on mouse press event.
    /// \param[in] _event Mouse press event.
    protected: virtual void mousePressEvent(QMouseEvent *_event);

    /// \brief Qt callback on mouse release event.
    /// \param[in] _event Mouse release event.
    protected: virtual void mouseReleaseEvent(QMouseEvent *_event);

    /// \brief Qt callback on mouse double click event.
    /// \param[in] _event Mouse double click event.
    protected: virtual void mouseDoubleClickEvent(QMouseEvent *_event);

    /// \brief Qt callback on mouse wheel event.
    /// \param[in] _event Mouse wheel event.
    protected: virtual void wheelEvent(QWheelEvent *_event);

    /// \brief Signal to set a contact visualization value.
    /// \param[in] _contactName Name of the contact sensor.
    /// \param[in] _value Force value.
    signals: void SetContactForce(QString _contactName, double _value);

    /// \brief Signal that motion capture status has changed.
    /// \param[in] _status 0: No data; 1: On; 2: Paused.
    signals: void MocapStatusChanged(int _status);

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

    /// \brief Callback triggered when the next button is clicked
    private slots: void OnResetArmClicked();

    /// \brief Callback triggered when the reset all button is clicked
    private slots: void OnResetClicked();

    /// \brief Callback triggered when the reset scene button is clicked
    private slots: void OnResetSceneClicked();

    /// \brief Callback triggered when local frame checked is clicked.
    /// \param[in] _state State of the check box.
    private slots: void OnLocalCoordMove(int _state);

    /// \brief Callback triggered when viewpoint rotations check box is clicked.
    /// \param[in] _state State of the check box.
    private slots: void OnViewpointRotationsCheck(int _state);

    /// \brief Callback triggered when stereo check box is clicked.
    /// \param[in] _state State of the check box.
    private slots: void OnStereoCheck(int _state);

    /// \brief Callback motion capture status has changed.
    /// \param[in] _status 0: No data; 1: On; 2: Paused.
    private slots: void OnMocapStatusChanged(int _status);

    /// \brief Callback when reset models has been triggered by the user.
    private slots: void OnResetModels();

    /// \brief Callback when restart timer has been triggered by the user.
    private slots: void OnRestartTimer();

    /// \brief Helper function to initialize the task view
    /// \param[in] _elem SDF element pointer that contains HAPTIX task
    /// parameters.
    private: void InitializeTaskView(sdf::ElementPtr _elem);

    /// \brief Handle GUI keypresses
    /// \param[in] _event Key press event.
    private: bool OnKeyPress(gazebo::common::KeyEvent _event);

    /// \brief callback on Initialize hx connection
    private: void OnInitialize(ConstIntPtr &_msg);

    /// \brief Reset models, signal motion tracking to pause, reset hand state
    private: void ResetModels();

    /// \brief Callback for subscriber to pause response
    /// \param[in] _msg pause state
    private: void OnPauseRequest(ConstIntPtr &_msg);

    /// \brief start a thread to poll contact sensor data
    private: void PollSensors();

    /// \brief Poll network for OptitrackBridge status
    private: void PollTracking();

    /// \brief callback for subscriber to the hydra publisher
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief Qt event filter used to filter child widget events.
    /// \param[in] _obj Object that is watched by the event filter.
    /// \param[in] _event Qt event.
    /// \return True if the event is handled.
    private: bool eventFilter(QObject *_obj, QEvent *_event);

    /// \brief Handle position scaling slider movement
    /// \param[in] _state State of the slider
    private slots: void OnScalingSlider(int _state);

    /// \brief Start a thread to stop and start the remote Optitrack service
    private slots: void OnStartStopMocap(bool _checked);

    /// \brief Optitrack alive callback
    private: void OnOptitrackAlive(ConstTimePtr &_time);

    /// \brief Get contact sensor information
    private: void UpdateSensorContact();

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

    /// \brief Publisher for signaling WorldControl node.
    private: gazebo::transport::PublisherPtr worldControlPub;

    // \brief Set of Gazebo signal connections.
    private: std::vector<gazebo::event::ConnectionPtr> connections;

    // \brief Tab for tasks.
    private: QTabWidget *taskTab;

    // \brief Frame for taskTab.
    private: QFrame *tabFrame;

    /// \brief Text box that hold instructions to the user.
    private: QTextEdit *instructionsView;

    /// \brief All the tasks in all the groups
    private: std::map<int, TaskButton*> taskList;

    /// \brief Number of the current task.
    private: int currentTaskId;

    /// \brief Publisher that talks with the arrange plugin to setup the
    /// scene.
    private: gazebo::transport::PublisherPtr taskPub;

    /// \brief Publisher that talks with the control plugin to enable viewpoint
    /// rotations.
    private: gazebo::transport::PublisherPtr viewpointRotationsPub;

    /// \brief Publisher that controls the clock
    private: gazebo::transport::PublisherPtr timerPub;

    /// \brief Reset arm button
    private: QPushButton *resetArmButton;

    /// \brief Reset scene button
    private: QPushButton *resetSceneButton;

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

    /// \brief actual starting pose of the arm.
    private: gazebo::math::Pose initialArmPose;

    /// \brief fake starting pose of the arm.
    private: gazebo::math::Pose armStartPose;

    /// \brief When true, move in the arm's local coordinate frame.
    private: bool localCoordMove;

    /// \brief Position movement scaling factor.
    private: double posScalingFactor;

    /// \brief Publish command to pause motion tracking
    private: gazebo::transport::PublisherPtr pausePub;

    /// \brief Subscriber to pause status
    private: gazebo::transport::SubscriberPtr pauseSub;

    /// \brief was pausing motion tracking successful?
    private: bool trackingPaused;

    /// \brief a lock to hold when commanding wrist/finger positions
    private: boost::mutex motorCommandMutex;

    /// \brief start a thread to poll contact sensor data
    private: boost::thread pollSensorsThread;

    /// \brief Motion capture status indicator.
    private: QLabel *mocapStatusIndicator;

    /// \brief Top bar widget.
    private: QFrame *topBarFrame;

    /// \brief Settings button.
    private: QToolButton *settingsButton;

    /// \brief start a thread to poll optitrackbridge
    private: boost::thread pollTrackingThread;

    /// \brief Subscribe to Optitrack liveliness
    private: gazebo::transport::SubscriberPtr optitrackAliveSub;

    /// \brief Last optitrack update time
    private: gazebo::common::Time optitrackUpdateTime;

    /// \brief True if received a signal to quit
    private: bool quit;

    /// \brief subscribe to hydra
    private: gazebo::transport::SubscriberPtr hydraSub;

    /// \brief Pixmap for the SVG hand
    private: QGraphicsPixmapItem *handItem;

    /// \brief Pointer to the render widget.
    private: QWidget *renderWidget;

    /// \brief GUI maximum width.
    private: int maxWidth;

    /// \brief GUI maximum height.
    private: int maxHeight;
  };
}
#endif
