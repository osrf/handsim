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
#include <map>
#include <string>

#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;

const double g_big = 1e29;
const double g_physics_tol = 1e-2;

class HaptixGraspingTest : public ServerFixture,
                           public testing::WithParamInterface<const char*>
{
  public: void HoldGrasps(const std::string &_world);
};

/////////////////////////////////////////////////
void HaptixGraspingTest::HoldGrasps(const std::string &_world)
{
  Load(_world, true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Unthrottle real time

  // Find the hand model

  // Move objects into hand
  // Close fingers

  // Wait fixed amount (measured in sim time)

  // 
}

/////////////////////////////////////////////////
TEST_P(HaptixGraspingTest, HoldGrasps)
{
  this->HoldGrasps(GetParam());
}

INSTANTIATE_TEST_CASE_P(AratWorlds, HaptixGraspingTest,
    testing::Values("worlds/arat.world", "worlds/arat_left.world"));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

