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
#ifndef _GUI_ARAT_PLUGIN_HH_
#define _GUI_ARAT_PLUGIN_HH_

#include <queue>

#include <haptix/comm/haptix.h>
#include <ignition/transport.hh>

#include "gazebo/common/Events.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include "gazebo/math/gzmath.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{

    class ContactsWrapper
    {
      public: ConstContactsPtr msg;
      public: std::string name;
      public: ContactsWrapper(ConstContactsPtr m, const std::string &n) : msg(m), name(n){}
    };

    class KeyCommand
    {
      public:
        char button;
        std::string name;
        int index;
        float increment;
        KeyCommand(const char b, const std::string& n, const float i) :
                        button(b), name(n), increment(i){}
        KeyCommand(){index = 0; }
    };

    class Grasp
    {
      public:
        float sliderValue;
        char incKey;
        char decKey;
        void SliderChanged(char key, float inc);
        Grasp(char i, char d) : incKey(i), decKey(d) { sliderValue = 0; }
        Grasp() {}
    };

    class QTaskButton : public QToolButton
    {
      Q_OBJECT
      public: QTaskButton();

      public: void SetTaskId(const std::string &task_id);
      public: void SetTaskInstructionsDocument(QTextDocument* instr);
      public: void SetIndex(const int i);

      public slots: void OnButton();

      private: std::string id;
      private: QTextDocument* instructions;

      // The canonical index at which this task is stored
      private: int index;

      signals: void SendTask(const std::string &id,
                             QTextDocument* instructions, const int index);
    };

    // Digital clock adapted from
    // http://qt-project.org/doc/qt-4.8/widgets-digitalclock.html
    class DigitalClock : public QLCDNumber
    {
       Q_OBJECT
       
       public: DigitalClock(QWidget *parent = 0);
       public: void StopClock();
       private: QTime lastStartTime;
       private: bool running;

       protected slots: void ShowTime();

      private slots: void OnStartStop();
    };

    class HaptixGUIPlugin : public gazebo::GUIPlugin 
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: HaptixGUIPlugin();

      /// \brief Destructor
      public: virtual ~HaptixGUIPlugin();

      /// \brief Callback trigged when the button is pressed.
      //protected slots: void OnButton();
      protected slots: void OnTaskSent(const std::string &id,
                                       QTextDocument* instructions,
                                       const int index);
      protected slots: void OnResetClicked(); 
      protected slots: void OnNextClicked(); 
      protected slots: void OnStartStopClicked();

      private: void PublishTaskMessage(const std::string &task_name); 

      /// \brief Node used to establish communication with gzserver.
      private: gazebo::transport::NodePtr node;

      private: ignition::transport::Node* ignNode;
      private: hxSensor handSensor;
      private: hxDeviceInfo handDeviceInfo;
      hxCommand handCommand;

      private: std::map<char, KeyCommand> armCommands;
      private: std::map<char, KeyCommand> handCommands;
      private: std::map<std::string, std::vector<char> > buttonNames;
      // The mapping between a button and the grasp it is commanding
      private: std::map<char, std::string> graspCommands;
      private: std::map<std::string, Grasp> grasps;

      /// \brief Publisher of factory messages.
      private: gazebo::transport::PublisherPtr taskPub;

      /// \brief Subscriber to finger contact sensors.
      private: std::vector<transport::SubscriberPtr> contactSubscribers;

      /// \brief Maximum number tasks.
      /// \sa taskNum

      private: int handImgX;
      private: int handImgY;

      private: std::string handImgFilename;
      private: std::string configFilename;

      private: float GUIScaleFactor;
      private: int circleSize;
      private: math::Vector2d iconSize;
      private: math::Vector3 colorMin;
      private: math::Vector3 colorMax;
      private: float forceMin;
      private: float forceMax;
      private: float graspIncrement;

      private: bool isTestRunning;
      private: QString startButtonStyle;
      private: QString stopButtonStyle;

      private: std::string handSide;
      
      private: std::map<std::string, math::Vector2d > contactPoints;

      private: std::map<std::string, QGraphicsEllipseItem*>
                                                contactGraphicsItems;

      private: std::vector<QTextDocument*> instructionsList;

      private: QVBoxLayout* mainLayout;

      private: QTextEdit* instructionsView;

      private: QGraphicsScene *handScene;

      private: QPushButton *startStopButton;
  
      private: DigitalClock *digitalClock;

      private: std::vector<event::ConnectionPtr> connections;

      private: std::queue<ContactsWrapper> msgQueue;
  
      private: std::vector<std::string> taskList;

      /// \brief Number of the current task.
      private: int currentTaskIndex;

      private: void InitializeHandView();

      private: void InitializeTaskView(sdf::ElementPtr elem,
                                       common::SystemPaths* paths);

      private: void OnFingerContact(ConstContactsPtr &msg);
      
      private: bool OnKeyPress(common::KeyEvent _event);

      private: std::string getTopicName(const std::string& fingerName);

      private: void PreRender();

    };

}
#endif
