/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <haptix/comm/haptix.h>
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxRobot.pb.h>
#include <haptix/comm/msg/hxSensor.pb.h>
#include <haptix/comm/msg/hxGrasp.pb.h>
#include <ignition/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_config.h>
#include <gazebo/Server.hh>

#include "ServerFixture.hh"

using namespace gazebo;

class PhysicsTest : public ServerFixture
{
  /// \breif test physics
  /// \param[in] _test
  public: void HandStiffnessTest1(double _test);
};

void PhysicsTest::HandStiffnessTest1(double /*_test*/)
{
  Load("worlds/arat.world");
  physics::WorldPtr world = physics::get_world("default");
   ASSERT_TRUE(world != NULL);
   physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
   ASSERT_TRUE(physics != NULL);
}

TEST_F(PhysicsTest, Test1)
{
  HandStiffnessTest1(1.0);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
