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
#include <haptix/comm/haptix.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class TactorsPlugin : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & _info);

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: bool robotInitialized;
    private: hxRobotInfo robotInfo;
    private: int fd;
    private: std::map<int, char> sensorMotorIndexMapping;
    private: float minContactForce;
    // For each motor, keep track of the last buzz time
    private: std::map<char, common::Timer> motorTimes;

    private: common::Time motorInterval;
  };
}
