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

#include <gtest/gtest.h>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test_config.h"

class Issue86Test : public gazebo::ServerFixture
{
  public: gazebo::physics::WorldPtr InitWorld(const std::string &_worldFile);
};

gazebo::physics::WorldPtr Issue86Test::InitWorld(const std::string &_worldFile)
{
  boost::filesystem::path path = HANDSIM_TEST_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path.string());
  Load(_worldFile, true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  return world;
}


/////////////////////////////////////////////////
TEST_F(Issue86Test, MotorLimits)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);
  hx_robot_info();
}

/////////////////////////////////////////////////
TEST_F(Issue86Test, MotorPositions)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);


}
