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

#include <sstream>
#include <limits>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/GuiEvents.hh>

#include "handsim/config.hh"
#include "handsim/TaskButton.hh"
#include "handsim/HaptixGUIPlugin.hh"

using namespace haptix_gazebo_plugins;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(HaptixGUIPlugin)

/////////////////////////////////////////////////
HaptixGUIPlugin::HaptixGUIPlugin()
  : GUIPlugin()
{
  this->trackingPaused = true;
  this->quit = false;
  this->localCoordMove = true;
  this->posScalingFactor = 0.25;

  // Read parameters
  std::string handImgFilename =
    gazebo::common::SystemPaths::Instance()->FindFileURI(
      "file://media/gui/arat/arat_icons/hand.svg");

  // Parameters for sensor contact visualization
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame {background-color: rgba(255, 255, 255, 255);"
      "color: rgba(100, 100, 100, 255);"
      "}"
      );

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();
  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->setContentsMargins(4, 4, 0, 0);
  mainFrame->setLayout(frameLayout);

  this->setPalette(QPalette(QColor(255, 255, 255, 0)));

  // Create the hand
  // Create a QGraphicsView to draw the finger force contacts
  this->handScene = new QGraphicsScene(QRectF(0, 0, 400, 220));
  QGraphicsView *handView = new QGraphicsView(this->handScene);
  handView->setStyleSheet("border: 0px");
  handView->setSizePolicy(QSizePolicy::Expanding,
                          QSizePolicy::MinimumExpanding);

  // Load the hand image
  QPixmap handImg = QPixmap(QString(handImgFilename.c_str()));
  QGraphicsPixmapItem *handItem = new QGraphicsPixmapItem(handImg);
  handItem->setPos(-20, -73);

  // Draw the hand on the canvas
  this->handScene->addItem(handItem);

  // Create the task layout
  this->taskTab = new QTabWidget();
  this->taskTab->setStyleSheet(
      "QTabWidget {"
        "border: 1px solid rgba(128, 128, 128, 255)"
      "}"

      "QTabWidget::pane {"
        "top: -1px;"
        "background-color: #ff00ff;"
        "border: 1px solid rgba(128, 128, 128, 255);"
      "}"

      "QTabBar::tab-bar {"
        "left: 5px"
      "}"

      "QTabBar::tab {"
        "color: rgba(100, 100, 100, 255);"
        "border: 1px solid rgba(128, 128, 128, 255);"
        "padding: 0px;"
        "border-top-left-radius: 4px;"
        "border-top-right-radius: 4px;"
        "background-color: rgba(200, 200, 200, 255);"
      "}"

      "QTabBar::tab:selected {"
        "color: rgba(100, 100, 100, 255);"
        "background-color: rgba(255, 255, 255, 255);"
        "border: 1px solid rgba(128, 128, 128, 255);"
        "border-bottom: 1px solid rgba(255, 255, 255, 255);"
      "}"
      );

  QFrame *tabFrame = new QFrame();
  tabFrame->setContentsMargins(4, 0, 4, 0);
  QVBoxLayout *tabFrameLayout = new QVBoxLayout();
  tabFrame->setLayout(tabFrameLayout);

  this->instructionsView = new QTextEdit("Instructions:");
  this->instructionsView->setReadOnly(true);
  this->instructionsView->setMaximumHeight(60);
  this->instructionsView->setMinimumHeight(60);
  this->instructionsView->setStyleSheet(
      "margin-top: 0px;"
      "margin-bottom: 0px;"
      "margin-left: 20px;"
      "margin-right: 20px;"
      "background-color: #ffffff"
      );

  tabFrameLayout->addWidget(taskTab);
  tabFrameLayout->addWidget(this->instructionsView);

  QHBoxLayout *cycleButtonLayout = new QHBoxLayout();

  // reset all button
  QPushButton *resetButton = new QPushButton();
  resetButton->setFocusPolicy(Qt::NoFocus);
  resetButton->setText(QString("Reset All"));
  resetButton->setStyleSheet(
      "background-color: rgba(120, 120, 120, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnResetClicked()));
  resetButton->setMaximumWidth(120);

  // next scene button
  QPushButton *nextButton = new QPushButton();
  nextButton->setFocusPolicy(Qt::NoFocus);
  nextButton->setText(QString("Next Test"));
  nextButton->setStyleSheet(
      "background-color: rgba(120, 120, 120, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(nextButton, SIGNAL(clicked()), this, SLOT(OnNextClicked()));
  nextButton->setMaximumWidth(120);

  // add reset scene only
  QPushButton *resetSceneButton = new QPushButton();
  resetSceneButton->setFocusPolicy(Qt::NoFocus);
  resetSceneButton->setText(QString("Reset Scene"));
  resetSceneButton->setStyleSheet(
      "background-color: rgba(120, 120, 120, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(resetSceneButton, SIGNAL(clicked()), this,
    SLOT(OnResetSceneClicked()));
  resetSceneButton->setMaximumWidth(120);

  cycleButtonLayout->addWidget(resetButton);
  cycleButtonLayout->addWidget(resetSceneButton);
  cycleButtonLayout->addWidget(nextButton);

  QFrame *cycleButtonFrame = new QFrame;
  cycleButtonFrame->setLayout(cycleButtonLayout);

  // Start/Stop button
  this->startStopButton = new QPushButton();
  this->startStopButton->setFocusPolicy(Qt::NoFocus);
  this->startStopButton->setCheckable(true);
  this->startStopButton->setText(QString("Start"));
  this->startStopButton->setDisabled(true);

  this->startStyle =
      "QPushButton {"
        "margin: 10px;"
        "margin-top: 0px;"
        "margin-bottom: 0px;"
        "padding: 2px;"
        "background-color: #7A95D6;"
        "font: bold 30px;"
        "border: 0px;"
        "border-radius: 4px;"
        "color: #FFFFFF;"
      "}"

      "QPushButton:hover {"
        "background-color: rgba(83, 101, 146, 255);"
      "}"

      "QPushButton::disabled {"
        "background-color: rgba(180, 180, 180, 255);"
        "color: rgba(200, 200, 200, 255);"
      "}";

  this->stopStyle =
      "QPushButton {"
        "margin: 10px;"
        "margin-top: 0px;"
        "margin-bottom: 0px;"
        "padding: 2px;"
        "background-color: #D85C48;"
        "font: bold 30px;"
        "border: 0px;"
        "border-radius: 4px;"
        "color: #FFFFFF;"
      "}"

      "QPushButton:hover {"
        "background-color: rgba(191, 81, 64, 255);"
      "}"

      "QPushButton::disabled {"
        "background-color: rgba(180, 180, 180, 255);"
        "color: rgba(200, 200, 200, 255);"
      "}";

  this->startStopButton->setStyleSheet(this->startStyle.c_str());

  connect(this->startStopButton, SIGNAL(toggled(bool)), this,
      SLOT(OnStartStop(bool)));

  QHBoxLayout *movementLayout = new QHBoxLayout();

  QSlider *posScalingSlider = new QSlider(Qt::Horizontal);
  posScalingSlider->setRange(1, 100);
  posScalingSlider->setValue(this->posScalingFactor*100);
  posScalingSlider->setToolTip(tr("Adjust keyboard arm movement speed"));
  connect(posScalingSlider, SIGNAL(sliderMoved(int)),
          this, SLOT(OnScalingSlider(int)));

  QCheckBox *localCoordMoveCheck = new QCheckBox("Local frame");
  localCoordMoveCheck->setToolTip(tr("Enable movement in arm's local frame"));
  localCoordMoveCheck->setFocusPolicy(Qt::NoFocus);
  localCoordMoveCheck->setChecked(true);
  connect(localCoordMoveCheck, SIGNAL(stateChanged(int)),
          this, SLOT(OnLocalCoordMove(int)));

  QCheckBox *stereoCheck = new QCheckBox("Stereo");
  stereoCheck->setToolTip(tr("Enable stereo rendering"));
  stereoCheck->setFocusPolicy(Qt::NoFocus);
  stereoCheck->setChecked(true);
  connect(stereoCheck, SIGNAL(stateChanged(int)),
          this, SLOT(OnStereoCheck(int)));

  QHBoxLayout *stereoCheckLayout = new QHBoxLayout();
  stereoCheckLayout->addWidget(stereoCheck);
  stereoCheckLayout->addStretch(1);


  movementLayout->addWidget(localCoordMoveCheck);
  movementLayout->addWidget(new QLabel(tr("Arm move speed:")));
  movementLayout->addWidget(posScalingSlider);

  frameLayout->addLayout(movementLayout);
  frameLayout->addLayout(stereoCheckLayout);

  QHBoxLayout *bottomLayout = new QHBoxLayout();
  std::string versionStr = std::string("  v ") + HANDSIM_VERSION_FULL;
  QLabel *versionLabel = new QLabel(tr(versionStr.c_str()));
  versionLabel->setStyleSheet("QLabel {font: 10px}");
  bottomLayout->addWidget(versionLabel);

  // Add all widgets to the main frame layout
  frameLayout->addWidget(handView, 1.0);
  frameLayout->addWidget(tabFrame);
  frameLayout->addWidget(instructionsView);
  frameLayout->addWidget(cycleButtonFrame);
  frameLayout->addWidget(startStopButton);
  frameLayout->addLayout(bottomLayout);

  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->move(10, 10);
  this->resize(480, 840);

  // Create a QueuedConnection to set contact visualization value.
  connect(this, SIGNAL(SetContactForce(QString, double)),
          this, SLOT(OnSetContactForce(QString, double)), Qt::QueuedConnection);

  // Create a node for transportation
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();

  // Create the publisher that communicates with the arrange plugin
  this->taskPub = this->node->Advertise<gazebo::msgs::GzString>("~/arrange");

  // Connect to the PreRender Gazebo signal
  this->connections.push_back(gazebo::event::Events::ConnectPreRender(
                              boost::bind(&HaptixGUIPlugin::PreRender, this)));

  // currentTaskId has default value of 0, gets set after reading SDF
  this->currentTaskId = 0;

  // Advertise the Ignition topic on which we'll publish arm pose changes
  this->ignNode.Advertise("haptix/arm_pose_inc");
}

/////////////////////////////////////////////////
HaptixGUIPlugin::~HaptixGUIPlugin()
{
  this->quit = true;
  this->pollSensorsThread.join();
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::Load(sdf::ElementPtr _elem)
{
  gazebo::rendering::UserCameraPtr userCamera =
                                            gazebo::gui::get_active_camera();
  if (userCamera)
    userCamera->SetHFOV(GZ_DTOR(120));
  this->initialCameraPose = userCamera->GetWorldPose();

  // Hide the scene tree.
  gazebo::gui::Events::leftPaneVisibility(false);

  // Create the publisher that controls the timer
  if (_elem->HasElement("timer_topic"))
  {
    this->timerPub = this->node->Advertise<gazebo::msgs::GzString>(
        _elem->Get<std::string>("timer_topic"));
  }
  else
  {
    this->timerPub =
      this->node->Advertise<gazebo::msgs::GzString>("~/timer_control");
  }

  this->pausePub =
    this->node->Advertise<gazebo::msgs::Int>("~/motion_tracking/pause_request");

  this->pauseSub =
    this->node->Subscribe("~/motion_tracking/pause_response",
      &HaptixGUIPlugin::OnPauseRequest, this);

  this->defaultContactSize = _elem->Get<gazebo::math::Vector2d>("default_size");

  this->forceMin = _elem->Get<double>("force_min");
  this->forceMax = _elem->Get<double>("force_max");

  this->colorNoContact = _elem->Get<gazebo::common::Color>("color_no_contact");
  this->colorMin = _elem->Get<gazebo::common::Color>("color_min");
  this->colorMax = _elem->Get<gazebo::common::Color>("color_max");

  this->handSide = _elem->Get<std::string>("hand_side");

  // Get contact names
  if (_elem->HasElement("contacts"))
  {
    sdf::ElementPtr contact = _elem->GetElement("contacts");
    contact = contact->GetElement("contact");

    while (contact)
    {
      if (contact->HasAttribute("name"))
      {
        // Read the contact data from SDF
        std::string contactName = contact->Get<std::string>("name");
        gazebo::math::Vector2d contactPos =
          contact->Get<gazebo::math::Vector2d>("pos");

        gazebo::math::Vector2d contactSize = this->defaultContactSize;
        if (contact->HasElement("size"))
        {
          contactSize = contact->Get<gazebo::math::Vector2d>("size");
        }

        this->contactGraphicsItems[contactName] =
          new QGraphicsRectItem(contactPos.x,
              contactPos.y, contactSize.x, contactSize.y);
        this->handScene->addItem(this->contactGraphicsItems[contactName]);

        this->contactGraphicsItems[contactName]->setBrush(
            QBrush(QColor(255, 255, 255, 0)));
        this->contactGraphicsItems[contactName]->setPen(
            QPen(QColor(153, 153, 153, 255)));

        // Get the position of the contact
        this->contactPoints[contactName] = contactPos;

        // Get the contact index
        int index = contact->Get<int>("index");
        this->contactNames[index] = contactName;

        contact = contact->GetNextElement();
      }
    }
  }

  // Draw contact pads and force gauge
  {
    float scaleXPos = 365;
    float scaleWidth = 20;

    QGraphicsRectItem *forceScaleItem =
      new QGraphicsRectItem(scaleXPos, -40, scaleWidth, 400);
    forceScaleItem->setPen(QPen(QColor(255, 255, 255, 0)));
    QLinearGradient grad(0, 0, 0, 400);
    grad.setColorAt(0, QColor(this->colorMax[0],
                              this->colorMax[1],
                              this->colorMax[2],
                              this->colorMax[3]));
    grad.setColorAt(1, QColor(this->colorMin[0],
                              this->colorMin[1],
                              this->colorMin[2],
                              this->colorMin[3]));
    forceScaleItem->setBrush(grad);
    this->handScene->addItem(forceScaleItem);

    // Draw the lines and force values.
    int lineStep = 40;
    double force = this->forceMax;
    int steps = (440/40) - 1;
    double forceStep = (this->forceMax - this->forceMin)/steps;

    for (int i = -40; i < 400-lineStep; i += lineStep)
    {
      QGraphicsLineItem *lineItem =
        new QGraphicsLineItem(scaleXPos, i, scaleXPos + scaleWidth, i);
      lineItem->setPen(QPen(
            QBrush(QColor(255, 255, 255, 255)), 2.0));
      this->handScene->addItem(lineItem);

      std::stringstream forceStream;
      forceStream << std::fixed << std::setprecision(1) << force;
      force -= forceStep;

      QGraphicsTextItem *text = new QGraphicsTextItem(
          forceStream.str().c_str());
      text->setPos(scaleXPos + scaleWidth + 4, i-11.5);
      this->handScene->addItem(text);
    }

    // Draw the PSI label
    QGraphicsTextItem *newtonText = new QGraphicsTextItem(tr("N"));
    newtonText->setPos(scaleXPos + scaleWidth - 20, -62);
    this->handScene->addItem(newtonText);
  }

  // Get motor key commands
  if (_elem->HasElement("motor_keys"))
  {
    sdf::ElementPtr motor = _elem->GetElement("motor_keys");
    motor = motor->GetElement("motor");

    while (motor)
    {
      if (motor->HasAttribute("inc_key") &&
          motor->HasAttribute("dec_key") &&
          motor->HasAttribute("index") &&
          motor->HasAttribute("increment"))
      {
        std::pair<unsigned int, float> mapping;
        mapping.first = motor->Get<int>("index");
        mapping.second = motor->Get<float>("increment");
        this->motorKeys[motor->Get<std::string>("inc_key")[0]] = mapping;
        mapping.second = -mapping.second;
        std::string decKey = motor->Get<std::string>("dec_key");

        // Special case to work around trouble with parsing "&" ("&amp;" doesn't
        // work either).
        if (decKey == "amp")
          decKey = "&";

        this->motorKeys[decKey[0]] = mapping;
      }
      else
      {
        gzwarn << "Skipping malformed motor_key/motor element" << std::endl;
      }
      motor = motor->GetNextElement();
    }
  }

  // Get arm key commands
  if (_elem->HasElement("arm_keys"))
  {
    sdf::ElementPtr arm = _elem->GetElement("arm_keys");
    arm = arm->GetElement("arm");

    while (arm)
    {
      if (arm->HasAttribute("inc_key") &&
          arm->HasAttribute("dec_key") &&
          arm->HasAttribute("index") &&
          arm->HasAttribute("increment"))
      {
        std::pair<unsigned int, float> mapping;
        mapping.first = arm->Get<int>("index");
        mapping.second = arm->Get<float>("increment");
        this->armKeys[arm->Get<std::string>("inc_key")[0]] = mapping;
        mapping.second = -mapping.second;
        std::string decKey = arm->Get<std::string>("dec_key");

        // Special case to work around trouble with parsing "&" ("&amp;" doesn't
        // work either).
        if (decKey == "amp")
          decKey = "&";
        this->armKeys[decKey[0]] = mapping;
      }
      else
      {
        gzwarn << "Skipping malformed arm_key/arm element" << std::endl;
      }
      arm = arm->GetNextElement();
    }
  }

  // Get grasp key commands
  if (_elem->HasElement("grasp_keys"))
  {
    sdf::ElementPtr grasp = _elem->GetElement("grasp_keys");
    grasp = grasp->GetElement("grasp");

    while (grasp)
    {
      if (grasp->HasAttribute("inc_key") &&
          grasp->HasAttribute("dec_key") &&
          grasp->HasAttribute("name") &&
          grasp->HasAttribute("increment"))
      {
        std::pair<std::string, float> mapping;
        mapping.first = grasp->Get<std::string>("name");
        mapping.second = grasp->Get<float>("increment");
        this->graspKeys[grasp->Get<std::string>("inc_key")[0]] = mapping;
        mapping.second = -mapping.second;
        std::string decKey = grasp->Get<std::string>("dec_key");
        // Special case to work around trouble with parsing "&" ("&amp;" doesn't
        // work either).
        if (decKey == "amp")
          decKey = "&";
        this->graspKeys[decKey[0]] = mapping;
      }
      else
      {
        gzwarn << "Skipping malformed grasp_key/grasp element" << std::endl;
      }
      grasp = grasp->GetNextElement();
    }
  }

  this->graspMode = true;
  this->hxInitialized = false;
  this->numWristMotors = 3;

  this->InitializeTaskView(_elem);

  gazebo::gui::KeyEventHandler::Instance()->SetAutoRepeat(true);
  gazebo::gui::KeyEventHandler::Instance()->AddPressFilter("arat_gui",
                          boost::bind(&HaptixGUIPlugin::OnKeyPress, this, _1));

  // hydra trigger maps to grasp
  this->hydraSub = this->node->Subscribe("~/hydra",
      &HaptixGUIPlugin::OnHydra, this);

  // Setup default arm starting pose
  this->armStartPose.rot = gazebo::math::Quaternion(0, 0, -1.5707);

  this->requestPub = this->node->Advertise<gazebo::msgs::Request>(
      "~/request");

  this->worldControlPub = this->node->Advertise<gazebo::msgs::WorldControl>
                                        ("~/world_control");

  this->responseSub = this->node->Subscribe("~/response",
      &HaptixGUIPlugin::OnResponse, this, true);

  // Request info about the mpl arm
  this->requestMsg = gazebo::msgs::CreateRequest("entity_info", "mpl");
  this->requestPub->Publish(*this->requestMsg);

  this->pollSensorsThread = boost::thread(
    boost::bind(&HaptixGUIPlugin::PollSensors, this));

  // latched subscription, HaptixControlPlugin only publishes this once.
  this->initializeSub = this->node->Subscribe("~/haptix_load",
      &HaptixGUIPlugin::OnInitialize, this, true);
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnInitialize(ConstIntPtr &/*_msg*/)
{
  // The first time, we need to talk to the hand.  Can't do this at startup
  // because the hand might not have been spawned yet.
  if (this->hxInitialized)
  {
    // already initialized
    // gzerr << "someone else initialized hxInitialized?\n";
    return;
  }
  else
  {
    if (::hx_robot_info(&this->robotInfo) != ::hxOK)
    {
      gzerr << "hx_robot_info(): Request error. Cannot control hand."
        << std::endl;
      return;
    }
    memset(&this->lastMotorCommand, 0, sizeof(this->lastMotorCommand));
    //::hxSensor sensor;
    if(::hx_update(&this->lastMotorCommand, &this->lastSensor) != ::hxOK)
    {
      gzerr << "hx_update(): Request error.\n" << std::endl;
      return;
    }

    this->hxInitialized = true;
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  gazebo::msgs::Model modelMsg;
  modelMsg.ParseFromString(_msg->serialized_data());
  //this->armStartPose = gazebo::msgs::Convert(modelMsg.pose());
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnSetContactForce(QString _contactName, double _value)
{
  // gzerr << _contactName.toStdString() << " : " << _value << "\n";

  // constant for decay
  // const float epsilon = 0.01;

  float colorArray[4] = {this->colorNoContact[0],
                         this->colorNoContact[1],
                         this->colorNoContact[2],
                         this->colorNoContact[3]};

  float forceRange = this->forceMax - this->forceMin;

  // stay white if below forceMin
  if (std::abs(_value) >= forceMin)
  {
    for (int i = 0; i < 3; ++i)
    {
      float colorRange = this->colorMax[i] - this->colorMin[i];

      colorArray[i] = this->colorMin[i] +
        colorRange * (std::abs(_value) - forceMin)/forceRange;

      if (colorMax[i] > this->colorMin[i])
      {
        colorArray[i] = gazebo::math::clamp(colorArray[i],
          this->colorMin[i], this->colorMax[i]);
      }
      else
      {
        colorArray[i] = gazebo::math::clamp(colorArray[i],
          this->colorMax[i], this->colorMin[i]);
      }
    }
    colorArray[3] = 255;
  }

  // debug
  // if (std::abs(_value) > this->forceMin)
  //   gzerr << _value << " :(" << colorArray[0] << ", " << colorArray[1]
  //         << ", " << colorArray[2] << ")\n";

  QBrush color(QColor(colorArray[0], colorArray[1], colorArray[2], colorArray[3]));

  this->contactGraphicsItems[_contactName.toStdString()]->setBrush(color);
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::PreRender()
{
  // Fade out old force values
  for (std::map<std::string, QGraphicsRectItem*>::iterator iter =
      this->contactGraphicsItems.begin();
      iter != this->contactGraphicsItems.end(); ++iter)
  {
    QBrush brush = iter->second->brush();
    QColor color = brush.color();
    color.setAlpha(std::max(0, color.alpha()-5));
    brush.setColor(color);

    iter->second->setBrush(brush);
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::InitializeTaskView(sdf::ElementPtr _elem)
{
  // Populate the taskTab by parsing out SDF
  if (!_elem->HasElement("task_group"))
    return;

  int taskIndex = 0;
  int groupIndex = 0;

  // Create a button group. This group will hold all the task buttons.
  QButtonGroup *buttonGroup = new QButtonGroup();

  sdf::ElementPtr taskGroup = _elem->GetElement("task_group");

  // Process each task group, as specified in SDF
  while (taskGroup)
  {
    std::string taskGroupName = taskGroup->Get<std::string>("name");
    sdf::ElementPtr task = taskGroup->GetElement("task");

    // Create the button layout and frame
    QFrame *groupFrame = new QFrame();
    QGridLayout *groupLayout = new QGridLayout();
    groupFrame->setLayout(groupLayout);

    int count = 0;
    bool initialTab = false;

    // Process each task in the group
    while (task)
    {
      // Read task information
      std::string id = task->Get<std::string>("id");
      std::string name = task->Get<std::string>("name");
      std::string instructions = task->Get<std::string>("instructions");
      std::string iconPath =
        gazebo::common::SystemPaths::Instance()->FindFileURI(
          task->Get<std::string>("icon"));
      bool enabled = task->Get<bool>("enabled");

      // Create a new button for the task
      TaskButton *taskButton = new TaskButton(name, id, taskIndex, groupIndex);
      taskButton->setFocusPolicy(Qt::NoFocus);
      taskButton->SetInstructions(instructions);
      taskButton->setEnabled(enabled);

      // Listen to the task button press signal
      connect(taskButton, SIGNAL(SendTask(const int)),
         this, SLOT(OnTaskSent(const int)));

      int col = count % 4;
      int row = count / 4;

      // Add the button to the visual layout
      groupLayout->addWidget(taskButton, row, col);

      // Add the button to the button group (ensurce exclusive buttons)
      buttonGroup->addButton(taskButton);

      // Add an icon, if specified
      if (!iconPath.empty())
      {
        QPixmap iconPixmap(QString::fromStdString(iconPath));

        taskButton->setIcon(QIcon(iconPixmap));
        taskButton->setIconSize(QSize(60, 54));
        taskButton->setMinimumSize(80, 80);
        taskButton->setMaximumSize(100, 80);
      }

      this->taskList[taskIndex] = taskButton;
      if (enabled &&
          (task->HasElement("initial")) && (task->Get<int>("initial") == 1))
      {
        this->currentTaskId = taskIndex;
        taskButton->setChecked(true);
        this->startStopButton->setDisabled(false);
        initialTab = true;
      }

      task = task->GetNextElement();

      count++;
      taskIndex++;
    }

    this->taskTab->addTab(groupFrame, QString::fromStdString(taskGroupName));
    if (initialTab)
      this->taskTab->setCurrentIndex(groupIndex);
    taskGroup = taskGroup->GetNextElement("task_group");
    groupIndex++;
  }

  this->instructionsView->setDocument(
      this->taskList[this->currentTaskId] ->Instructions());
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnTaskSent(const int _id)
{
  this->startStopButton->setDisabled(false);
  this->startStopButton->setChecked(false);

  // reset the clock when a new task is selected
  this->PublishTimerMessage("reset");

  // Show the instructions to the user
  this->instructionsView->setDocument(this->taskList[_id]->Instructions());
  this->currentTaskId = _id;
  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());

  // Reset models
  this->ResetModels();

  // Reset the camera
  gazebo::gui::get_active_camera()->SetWorldPose(this->initialCameraPose);
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnNextClicked()
{
  this->startStopButton->setDisabled(false);
  this->startStopButton->setChecked(false);

  // reset the clock when a new task is selected
  this->PublishTimerMessage("reset");

  do
  {
    this->currentTaskId = (this->currentTaskId+1) % this->taskList.size();
  } while (!this->taskList[this->currentTaskId]->isEnabled());

  this->instructionsView->setDocument(
      this->taskList[this->currentTaskId]->Instructions());
  this->taskList[this->currentTaskId]->setChecked(true);
  this->taskTab->setCurrentIndex(this->taskList[this->currentTaskId]->Group());

  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());

  // Reset models
  this->ResetModels();

  // Reset the camera
  gazebo::gui::get_active_camera()->SetWorldPose(this->initialCameraPose);
}

////////////////////////////////////////////////
void HaptixGUIPlugin::PublishTimerMessage(const std::string &_msg) const
{
  gazebo::msgs::GzString msg;
  msg.set_data(_msg);
  this->timerPub->Publish(msg);
}

////////////////////////////////////////////////
void HaptixGUIPlugin::PublishTaskMessage(const std::string &_taskId) const
{
  gazebo::msgs::GzString msg;
  msg.set_data(_taskId);
  this->taskPub->Publish(msg);
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnStartStop(bool _checked)
{
  if (_checked)
  {
    this->startStopButton->setText(tr("Stop"));
    this->startStopButton->setStyleSheet(this->stopStyle.c_str());
    this->PublishTimerMessage("start");
  }
  else
  {
    this->startStopButton->setText(tr("Start"));
    this->startStopButton->setStyleSheet(this->startStyle.c_str());
    this->PublishTimerMessage("stop");
  }
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnResetClicked()
{
  this->startStopButton->setChecked(false);

  // Signal to the TimerPlugin to reset the clock
  this->PublishTimerMessage("reset");

  // Reset models
  this->ResetModels();

  // Reset the camera
  gazebo::gui::get_active_camera()->SetWorldPose(this->initialCameraPose);
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnResetSceneClicked()
{
  this->startStopButton->setChecked(false);

  // Signal to the TimerPlugin to reset the clock
  this->PublishTimerMessage("reset");

  // place scene objects back
  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::PollSensors()
{
  while(!quit)
  {
    if (this->hxInitialized)
    {
      // gzdbg << "contact sensor polling thread running\n";
      if (::hx_read_sensors(&this->lastSensor) != ::hxOK)
      {
        gzerr << "hx_read_sensors(): Request error." << std::endl;
      }
      this->UpdateSensorContact();
    }
    usleep(1000);  // 1kHz max
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::UpdateSensorContact()
{
  /* debug print forces
  gzerr << "ncontact sensor [" << this->robotInfo.ncontactsensor << "]\n";
  for (int i = 0; i < this->robotInfo.ncontactsensor; ++i)
  {
    gzerr << "sensor [" << i
          << "]: contacts [" << this->lastSensor.contact[i] << "]\n";
  }
  */

  for (std::map<int, std::string>::iterator c = this->contactNames.begin();
       c != this->contactNames.end(); ++c)
  {
    int i = c->first;
    // gzdbg << "name: " << c->second << "\n"
    //       << "index: " << i << "\n"
    //       << "force: " << this->lastSensor.contact[i] << "\n";
    this->SetContactForce(QString::fromStdString(this->contactNames[i]),
      this->lastSensor.contact[i]);
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::ResetModels()
{
  boost::mutex::scoped_lock lock(this->motorCommandMutex);

  // Signal to HaptixControlPlugin to pause motion tracking
  this->trackingPaused = false;
  gazebo::msgs::Int pause;
  pause.set_data(1);
  this->pausePub->Publish(pause);
  int maxTries = 30;
  gzdbg << "waiting for response from motion tracker (max wait 3 sec).\n";
  while (maxTries > 0 && !this->trackingPaused)
  {
    --maxTries;
    usleep(100000);
  }

  // Signal to WorldControl to reset the world
  gazebo::msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  this->worldControlPub->Publish(msg);

  // Also reset wrist and finger posture
  memset(&this->lastMotorCommand, 0, sizeof(this->lastMotorCommand));
  //::hxSensor sensor;
  if (::hx_update(&this->lastMotorCommand, &this->lastSensor) != ::hxOK)
    gzerr << "hx_update(): Request error.\n" << std::endl;

  // And zero the grasp, if any.
  if (this->lastGraspRequest.grasps_size() > 0)
  {
    this->lastGraspRequest.mutable_grasps(0)->set_grasp_value(0.0);
    haptix::comm::msgs::hxCommand resp;
    bool result;
    if(!this->ignNode.Request("haptix/gazebo/Grasp",
                              this->lastGraspRequest,
                              1000,
                              resp,
                              result) || !result)
    {
      gzerr << "Failed to call gazebo/Grasp service" << std::endl;
    }
  }
}

/////////////////////////////////////////////////
bool HaptixGUIPlugin::OnKeyPress(gazebo::common::KeyEvent _event)
{
  boost::mutex::scoped_lock lock(this->motorCommandMutex);

  if (!this->hxInitialized)
  {
    gzwarn << "hxInitialized is false, waiting for arm to spawn?\n";
    return false;
  }

  char key = _event.text[0];

  if (key == 'p' || key == ' ')
  {
    gazebo::msgs::Int pauseState;
    bool oldPauseState = this->trackingPaused;
    pauseState.set_data(!oldPauseState);
    this->pausePub->Publish(pauseState);

    int maxTries = 30;
    while (maxTries > 0 && this->trackingPaused == oldPauseState)
    {
      // Wait for ControlPlugin to pause
      --maxTries;
      usleep(100000);
    }
    return true;
  }

  // '~' toggles between grasp mode and motor mode
  if (key == '~')
  {
    // Send a motor command to hold current pose
    //::hxSensor sensor;
    if (::hx_update(&this->lastMotorCommand, &this->lastSensor) != ::hxOK)
    {
      gzerr << "hx_update(): Request error." << std::endl;
    }

    if (this->graspMode)
    {
      // Send an empty grasp request, to switch modes in the control plugin
      haptix::comm::msgs::hxGrasp req;
      haptix::comm::msgs::hxCommand rep;
      bool result;
      if(!this->ignNode.Request("haptix/gazebo/Grasp",
                                req, 1000, rep, result) || !result)
      {
        gzerr << "Failed to call gazebo/Grasp service" << std::endl;
      }
    }
    this->graspMode = !this->graspMode;
    gzdbg << "Changed graspMode to: " << this->graspMode << std::endl;
    return false;
  }

  // Is this an arm motion command?  These keys don't overlap with any others.
  std::map<char, std::pair<unsigned int, float> >::const_iterator arm;
  arm = this->armKeys.find(key);
  if (arm != this->armKeys.end())
  {
    unsigned int index = arm->second.first;
    if (index > 5)
    {
      gzerr << "Index out of bounds for arm control." << std::endl;
      return false;
    }
    float inc = arm->second.second;

    float poseIncArgs[6] = {0, 0, 0, 0, 0, 0};
    poseIncArgs[index] = inc;

    gazebo::math::Vector3 position, rot;

    // Move in the local coordinate frame if true.
    if (this->localCoordMove)
    {
      rot = gazebo::math::Vector3(poseIncArgs[4],
          -poseIncArgs[3], poseIncArgs[5]);
      position = gazebo::math::Vector3(-poseIncArgs[0],
          -poseIncArgs[1], poseIncArgs[2]);

      position = this->armStartPose.rot.RotateVector(position);
      rot = this->armStartPose.rot.RotateVector(rot);
    }
    else
    {
      position = gazebo::math::Vector3(poseIncArgs[0], poseIncArgs[1],
          poseIncArgs[2]);
      rot = gazebo::math::Vector3(-poseIncArgs[3], -poseIncArgs[4],
          poseIncArgs[5]);
    }

    gazebo::math::Pose increment(position * this->posScalingFactor, rot);
    this->armStartPose.rot = gazebo::math::Quaternion(rot) *
      this->armStartPose.rot;

    gazebo::msgs::Pose msg = gazebo::msgs::Convert(increment);

    // std::cout << "haptix/arm_pose_inc: " << msg.DebugString() << std::endl;
    this->ignNode.Publish("haptix/arm_pose_inc", msg);
    return true;
  }

  // If we're in grasp mode, is this a grasp command?
  std::map<char, std::pair<std::string, float> >::const_iterator grasp;
  grasp = this->graspKeys.find(key);
  if (this->graspMode && (grasp != this->graspKeys.end()))
  {
    std::string name = grasp->second.first;
    float inc = grasp->second.second;
    // If we're commanding the same grasp as last time, increment it; otherwise,
    // start over.
    haptix::comm::msgs::hxGrasp graspTmp;
    haptix::comm::msgs::hxGrasp::hxGraspValue* gv = graspTmp.add_grasps();
    gv->set_grasp_name(name);
    if ((this->lastGraspRequest.grasps_size() > 0) &&
        (this->lastGraspRequest.grasps(0).grasp_name() == name))
    {
      float curr = this->lastGraspRequest.grasps(0).grasp_value();
      float newValue = curr + inc;
      if (newValue > 1.0)
        newValue = 1.0;
      if (newValue < 0.0)
        newValue = 0.0;
      gv->set_grasp_value(newValue);
    }
    else
      gv->set_grasp_value(inc);

    bool result;
    // std::cout << "haptix/gazebo/Grasp: " << grasp.DebugString() << std::endl;
    haptix::comm::msgs::hxCommand resp;
    if(!this->ignNode.Request("haptix/gazebo/Grasp",
                              graspTmp,
                              1000,
                              resp,
                              result) || !result)
    {
      gzerr << "Failed to call gazebo/Grasp service" << std::endl;
      return false;
    }

    //gzdbg << "Received grasp response: " << resp.DebugString() << std::endl;

    this->lastGraspRequest = graspTmp;
    // Assign to lastMotorCommand, because now we're tracking the target based
    // purely on grasp poses.
    for (int i = this->numWristMotors; i < this->robotInfo.motor_count; ++i)
    {
      this->lastMotorCommand.ref_pos[i] = resp.ref_pos(i);
    }
    return true;
  }

  // If it's a motor command and either: we're not in grasp mode, or it's
  // for the wrist motors (which are the first 3 indices).
  std::map<char, std::pair<unsigned int, float> >::const_iterator motor;
  motor = this->motorKeys.find(key);
  if (motor != this->motorKeys.end())
  {
    int index = motor->second.first;
    if (index >= this->robotInfo.motor_count)
    {
      gzerr << "Index out of bounds for motor control." << std::endl;
      return false;
    }
    float inc = motor->second.second;
    if (!this->graspMode || (motor->second.first < this->numWristMotors))
    {
      // Start with the last direct motor command.
      ::hxCommand cmd;
      for (int i=0; i < this->robotInfo.motor_count; ++i)
        cmd.ref_pos[i] = this->lastMotorCommand.ref_pos[i];

      // Now add in the new diff
      cmd.ref_pos[index] += inc;

      // Now command it.
      // std::cout << "Sending: " << std::endl;
      //  for(int i = 0; i < this->robotInfo.motor_count; ++i)
      //    std::cout << cmd.ref_pos[i] << " ";
      // std::cout << std::endl;
      //::hxSensor sensor;
      if (::hx_update(&cmd, &this->lastSensor) != ::hxOK)
      {
        gzerr << "hx_update(): Request error." << std::endl;
      }

      // print current command for capturing into grasp
      /*
      gzdbg << "Current grasp:\n";
      for (unsigned int i=0; i<this->robotInfo.motor_count; ++i)
      {
        // cannot use gzdbg because extra code line info.
        std::cout << cmd.ref_pos[i] << " ";
      }
      gzdbg << "\n";
      */

      // And record it for next time
      for (int i=0; i<this->robotInfo.motor_count; ++i)
      {
        if (cmd.ref_pos[i] < this->robotInfo.motor_limit[i][0])
        {
          this->lastMotorCommand.ref_pos[i] =
              this->robotInfo.motor_limit[i][0];
        }
        else if (cmd.ref_pos[i] > this->robotInfo.motor_limit[i][1])
        {
          this->lastMotorCommand.ref_pos[i] =
              this->robotInfo.motor_limit[i][1];
        }
        else
        {
          this->lastMotorCommand.ref_pos[i] = cmd.ref_pos[i];
        }
      }
      return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnLocalCoordMove(int _state)
{
  this->localCoordMove = _state == Qt::Unchecked ? false : true;
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnStereoCheck(int _state)
{
  gazebo::gui::get_active_camera()->EnableStereo(_state);
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnScalingSlider(int _state)
{
  this->posScalingFactor = _state * 0.01;
}

//////////////////////////////////////////////////
void HaptixGUIPlugin::OnPauseRequest(ConstIntPtr &_msg)
{
  if (_msg->data() == 0)
  {
    this->trackingPaused = false;
  }
  else if (_msg->data() == 1)
  {
    this->trackingPaused = true;
  }
  else
  {
    gzwarn << "Got unexpected message data in";
  }
}

//////////////////////////////////////////////////
void HaptixGUIPlugin::OnHydra(ConstHydraPtr &_msg)
{
  if (!hxInitialized)
    return;

  bool engage = _msg->right().button_1();

  if (!engage)
    return;

  double command = _msg->right().trigger();

  std::string name = "Spherical";
  haptix::comm::msgs::hxGrasp grasp;
  haptix::comm::msgs::hxGrasp::hxGraspValue* gv = grasp.add_grasps();
  gv->set_grasp_name(name);
  gv->set_grasp_value(command);

  bool result;
  // std::cout << "haptix/gazebo/Grasp: " << grasp.DebugString() << std::endl;
  haptix::comm::msgs::hxCommand resp;
  if(!this->ignNode.Request("haptix/gazebo/Grasp",
                            grasp,
                            1000,
                            resp,
                            result) || !result)
  {
    gzerr << "Failed to call gazebo/Grasp service" << std::endl;
  }

  // gzdbg << "Received grasp response: " << resp.DebugString() << std::endl;

  this->lastGraspRequest = grasp;
  // Assign to lastMotorCommand, because now we're tracking the target based
  // purely on grasp poses.
  for (int i = this->numWristMotors; i < this->robotInfo.motor_count; ++i)
  {
    this->lastMotorCommand.ref_pos[i] = resp.ref_pos(i);
  }
}
