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

void TactorsPlugin::Load(physics::WorldPtr /*_parent*/, sdf::ElementPtr /*_sdf*/)
{
  // might have to be on Init instead?
  // Open the USB device for communication with the motors.
  for (int i = 0; i < 4; i ++)
  {
    std::string deviceName = "/dev/ttyACM" + std::to_string(i);
    this->fd = open(deviceName.c_str(), O_WRONLY);
    if (fd < 0 && i == 3)
    {
      perror("Failed to open USB port");
      return;
    }
    else
    {
      break;
    }
  }

  this->minContactForce = 0;

  // Initialize sensor index to Lilypad motor index mapping
  // These sensor indices correspond to the JHU APL MPL arm
  for (unsigned int i = 0; i < robotInfo.contact_sensor_count; i++)
  {
    // Uninitialized
    this->sensorMotorIndexMapping[i] = '0';
  }
  for (unsigned int i = 3; i <= 6; i++)
  {
    // Index
    this->sensorMotorIndexMapping[i] = '1';
  }
  for (unsigned int i = 11; i <= 14; i++)
  {
    // Middle
    this->sensorMotorIndexMapping[i] = '2';
  }
  for (unsigned int i = 17; i <= 20; i++)
  {
    // Ring
    this->sensorMotorIndexMapping[i] = '3';
  }
  for (unsigned int i = 7; i <= 9; i++)
  {
    // Little
    this->sensorMotorIndexMapping[i] = '4';
  }
  for (unsigned int i = 21; i <= 23; i++)
  {
    // Thumb
    this->sensorMotorIndexMapping[i] = '5';
  }

  this->motorInterval = common::Time(0.5);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TactorsPlugin::OnUpdate, this, _1));
}

void TactorsPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
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

  for (unsigned int i = 0; i < this->robotInfo.contact_sensor_count; i++)
  {
    char j = this->sensorMotorIndexMapping[i];
    if (sensor.contact[i] > minContactForce)
    {
      if (j <= '5' && j >= '1')
      {
        if (this->motorTimes[j].GetElapsed() > this->motorInterval)
        {
          // Write to the corresponding motor to make it buzz
          char key[1] = {j};
          write(this->fd, key, 1);

          this->motorTimes[j].Reset();
          this->motorTimes[j].Start();
        }
      }
    }
    else
    {
      this->motorTimes[j].Stop();
    }
  }
}
