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
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/GuiEvents.hh>

#include "haptix_gazebo_plugins/TaskButton.hh"
#include "haptix_gazebo_plugins/HaptixGUIPlugin.hh"

using namespace haptix_gazebo_plugins;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(HaptixGUIPlugin)

/////////////////////////////////////////////////
HaptixGUIPlugin::HaptixGUIPlugin()
  : GUIPlugin()
{
  // Read parameters
  std::string handImgFilename = common::SystemPaths::Instance()->FindFileURI
                                  ("file://config/arat_icons/hand.svg");

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
  QPushButton *resetButton = new QPushButton();
  resetButton->setText(QString("Reset Test"));
  resetButton->setStyleSheet(
      "background-color: rgba(120, 120, 120, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnResetClicked()));
  resetButton->setMaximumWidth(120);

  QPushButton *nextButton = new QPushButton();
  nextButton->setText(QString("Next Test"));
  nextButton->setStyleSheet(
      "background-color: rgba(120, 120, 120, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(nextButton, SIGNAL(clicked()), this, SLOT(OnNextClicked()));
  nextButton->setMaximumWidth(120);

  cycleButtonLayout->addWidget(resetButton);
  cycleButtonLayout->addWidget(nextButton);

  QFrame *cycleButtonFrame = new QFrame;
  cycleButtonFrame->setLayout(cycleButtonLayout);

  // Start/Stop button
  this->startStopButton = new QPushButton();
  this->startStopButton->setCheckable(true);
  this->startStopButton->setText(QString("Start"));
  this->startStopButton->setDisabled(true);
  this->startStyle =
      "QPushButton {"
        "margin: 10px;"
        "padding: 10px;"
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
        "padding: 10px;"
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

  // Add all widgets to the main frame layout
  frameLayout->addWidget(handView, 1.0);
  frameLayout->addWidget(tabFrame);
  frameLayout->addWidget(instructionsView);
  frameLayout->addWidget(cycleButtonFrame);
  frameLayout->addWidget(startStopButton);

  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->move(10, 10);
  this->resize(450, 800);

  // Create a QueuedConnection to set contact visualization value.
  connect(this, SIGNAL(SetContactForce(QString, double)),
          this, SLOT(OnSetContactForce(QString, double)), Qt::QueuedConnection);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Create the publisher that communicates with the arrange plugin
  this->taskPub = this->node->Advertise<msgs::GzString>("~/arrange");

  // Connect to the PreRender Gazebo signal
  this->connections.push_back(event::Events::ConnectPreRender(
                              boost::bind(&HaptixGUIPlugin::PreRender, this)));

  this->currentTaskId = 0;
}

/////////////////////////////////////////////////
HaptixGUIPlugin::~HaptixGUIPlugin()
{
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::Load(sdf::ElementPtr _elem)
{
  if (gui::get_active_camera())
    gui::get_active_camera()->SetHFOV(GZ_DTOR(120));

  // Hide the scene tree.
  gui::Events::leftPaneVisibility(false);

  // Create the publisher that controls the timer
  if (_elem->HasElement("timer_topic"))
  {
    this->timerPub = this->node->Advertise<msgs::GzString>(
        _elem->Get<std::string>("timer_topic"));
  }
  else
  {
    this->timerPub = this->node->Advertise<msgs::GzString>("~/timer_control");
  }

  this->circleSize = _elem->Get<int>("circle_size");

  this->forceMin = _elem->Get<double>("force_min");
  this->forceMax = _elem->Get<double>("force_max");

  this->colorMin = _elem->Get<common::Color>("color_min");
  this->colorMax = _elem->Get<common::Color>("color_max");

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
        math::Vector2d contactPos = contact->Get<math::Vector2d>("pos");
        std::string topic = contact->Get<std::string>("topic");

        // Create a subscriber that receive contact data
        transport::SubscriberPtr sub = this->node->Subscribe(topic,
            &HaptixGUIPlugin::OnFingerContact, this);
        this->contactSubscribers.push_back(sub);

        this->contactGraphicsItems[contactName] =
          new QGraphicsEllipseItem(contactPos.x,
              contactPos.y, this->circleSize, this->circleSize);
        this->handScene->addItem(this->contactGraphicsItems[contactName]);

        this->contactGraphicsItems[contactName]->setBrush(
            QBrush(QColor(255, 255, 255, 0)));
        this->contactGraphicsItems[contactName]->setPen(
            QPen(QColor(153, 153, 153, 255)));

        // Get the position of the contact
        this->contactPoints[contactName] = contactPos;

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
    grad.setColorAt(0, QColor(255, 227, 32, 255));
    grad.setColorAt(1, QColor(255, 102, 102, 255));
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

  this->InitializeTaskView(_elem);

  gui::KeyEventHandler::Instance()->SetAutoRepeat(true);
  gui::KeyEventHandler::Instance()->AddPressFilter("arat_gui",
                          boost::bind(&HaptixGUIPlugin::OnKeyPress, this, _1));
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnFingerContact(ConstContactsPtr &_msg)
{
  // Parse out the finger name
  if (_msg->contact_size() > 0)
  {
    std::string collisionName1 = _msg->contact(0).collision1();
    std::string collisionName2 = _msg->contact(0).collision2();

    // Calculate the force
    msgs::Vector3d forceVector = _msg->contact(0).wrench(0).
      body_1_wrench().force();
    double force = math::Vector3(forceVector.x(), forceVector.y(),
        forceVector.z()).GetLength();

    if (this->contactPoints.find(collisionName1) != this->contactPoints.end())
    {
      // Draw the new force value
      this->SetContactForce(QString::fromStdString(collisionName1), force);
    }
    else if (this->contactPoints.find(collisionName2) !=
             this->contactPoints.end())
    {
      // Draw the new force value
      this->SetContactForce(QString::fromStdString(collisionName2), force);
    }
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnSetContactForce(QString _contactName, double _value)
{
  double colorArray[3];
  float forceRange = this->forceMax - this->forceMin;

  for (int i = 0; i < 3; ++i)
  {
    float colorRange = this->colorMax[i] - this->colorMin[i];
    colorArray[i] = colorRange/forceRange * _value + this->colorMin[i];

    if (colorArray[i] > this->colorMin[i])
      colorArray[i] = this->colorMin[i];
    else if (colorArray[i] < this->colorMax[i])
      colorArray[i] = this->colorMax[i];
  }

  QBrush color(QColor(colorArray[0], colorArray[1], colorArray[2]));

  this->contactGraphicsItems[_contactName.toStdString()]->setBrush(color);
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::PreRender()
{
  // Fade out old force values
  for (std::map<std::string, QGraphicsEllipseItem*>::iterator iter =
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

    // Process each task in the group
    while (task)
    {
      // Read task information
      std::string id = task->Get<std::string>("id");
      std::string name = task->Get<std::string>("name");
      std::string instructions = task->Get<std::string>("instructions");
      std::string iconPath = common::SystemPaths::Instance()->FindFileURI(
          task->Get<std::string>("icon"));
      bool enabled = task->Get<bool>("enabled");

      // Create a new button for the task
      TaskButton *taskButton = new TaskButton(name, id, taskIndex, groupIndex);
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
        taskButton->setIconSize(QSize(60, 60));
        taskButton->setMinimumSize(80, 80);
        taskButton->setMaximumSize(100, 80);
      }

      this->taskList[taskIndex] = taskButton;

      task = task->GetNextElement();

      count++;
      taskIndex++;
    }

    this->taskTab->addTab(groupFrame, QString::fromStdString(taskGroupName));
    taskGroup = taskGroup->GetNextElement("task_group");
    groupIndex++;
  }
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
}

////////////////////////////////////////////////
void HaptixGUIPlugin::PublishTimerMessage(const std::string &_msg) const
{
  msgs::GzString msg;
  msg.set_data(_msg);
  this->timerPub->Publish(msg);
}

////////////////////////////////////////////////
void HaptixGUIPlugin::PublishTaskMessage(const std::string &_taskId) const
{
  msgs::GzString msg;
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

  // Signal to the ArrangePlugin to set up the current task
  this->PublishTimerMessage("reset");
  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());
}

/////////////////////////////////////////////////
bool HaptixGUIPlugin::OnKeyPress(common::KeyEvent /*_event*/)
{
  return true;
}
