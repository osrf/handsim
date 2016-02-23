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
#include <fcntl.h>
#include <map>
#include <time.h>
#include <unistd.h>

#include <gazebo/common/common.hh>
#include <handsim/TactorsPlugin.hh>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(TactorsPlugin)

TactorsPlugin::TactorsPlugin()
{
  this->robotInitialized = false;
  this->running = false;
}

void TactorsPlugin::Load(physics::WorldPtr _parent,
    sdf::ElementPtr /*_sdf*/)
{
  // might have to be on Init instead?
  // Open the USB device for communication with the motors.
  for (int i = 0; i < 4; i++)
  {
    std::string deviceName = "/dev/ttyACM" + std::to_string(i);
    this->fd = open(deviceName.c_str(), O_WRONLY);
    if (fd >= 0)
    {
      std::cout << "Writing to device: " << deviceName << std::endl;
      break;
    }
    else if (i == 3)
    {
      perror("Failed to open USB port");
      return;
    }
  }

  this->minContactForce = 0;
  this->maxContactForce = 7;

  // Initialize sensor index to Lilypad motor index mapping
  // These sensor indices correspond to the JHU APL MPL arm

  for (int i = 0; i < robotInfo.contact_sensor_count; i++)
  {
    // Uninitialized
    this->sensorMotorIndexMapping[i] = 5;
  }
  for (unsigned int i = 4; i <= 6; i++)
  {
    // thumb
    this->sensorMotorIndexMapping[i] = 0;
  }
  for (unsigned int i = 10; i <= 12; i++)
  {
    // Middle
    this->sensorMotorIndexMapping[i] = 1;
  }
  for (unsigned int i = 16; i <= 18; i++)
  {
    // pinky
    this->sensorMotorIndexMapping[i] = 2;
  }
  for (unsigned int i = 7; i <= 9; i++)
  {
    // index
    this->sensorMotorIndexMapping[i] = 3;
  }
  for (unsigned int i = 13; i <= 15; i++)
  {
    // Ring
    this->sensorMotorIndexMapping[i] = 4;
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TactorsPlugin::OnUpdate, this, _1));

  this->gzNode = transport::NodePtr(new transport::Node());
  gzNode->Init(_parent->GetName());
  this->runningSub = this->gzNode->Subscribe("~/tactors_running",
      &TactorsPlugin::OnRunningCallback, this);
}

void TactorsPlugin::OnRunningCallback(ConstIntPtr &_msg)
{
  this->running = _msg->data();
}

void TactorsPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  if (!running)
  {
    return;
  }

  if (!this->robotInitialized)
  {
    // Initialize haptix-comm
    if (hx_robot_info(&robotInfo) != hxOK)
    {
      printf("hx_robot_info(): Request error.\n");
      return;
    }
    this->robotInitialized = true;
  }

  hxSensor sensor;
  if (hx_read_sensors(&sensor) != hxOK)
  {
    gzerr << "hx_read_sensors(): Request error.\n";
    return;
  }

  // hack: skip the first 4 contact sensors because they are palm sensors
  for (int i = 4; i < this->robotInfo.contact_sensor_count; i++)
  {
    char j = this->sensorMotorIndexMapping[i];
    if (sensor.contact[i] > minContactForce)
    {
      if (j <= 4 && j >= 0)
      {
        // Write to the corresponding motor to make it buzz
        float contactForce = sensor.contact[i] > this->maxContactForce ?
            this->maxContactForce : sensor.contact[i];
        contactForce = 32*contactForce/this->maxContactForce;
        unsigned char multiplier = contactForce;
        if (multiplier > 0)
        {
          unsigned char keyChar = (j << 5) + multiplier;
          unsigned char key[1] = {keyChar};
          write(this->fd, key, 1);
        }
      }
    }
  }
}
