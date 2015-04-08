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
#include <haptix/comm/haptix.h>
#include "ServerFixture.hh"

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

  // Unthrottle real time (?)

  // Find the hand model
  /*physics::ModelPtr armModel;

  if (_world == "worlds/arat.world")
  {
    armModel = world->GetModel("mpl_haptix_right_forearm");
  }
  else if (_world == "worlds/arat_left.world")
  {
    armModel = world->GetModel("mpl_haptix_left_forearm");
  }

  EXPECT_TRUE(armModel);

  EXPECT_TRUE(armModel->GetLink("wristz"));
  math::Pose palmPose = armModel->GetLink("wristz")->GetWorldPose();

  // Iterate through choice objects
  // C++11 vector initialization
  std::vector<std::string> graspObjects =
      {"wood_cube_10cm", "wood_cube_7_5cm","wood_cube_5cm", "cricket_ball"};

  // alternative approach:
  // Iterate through trios of test parameters:
  //    std::string name, math::Pose objectPose, hxCommand grasp

  for (auto const &object : graspObjects)
  {
    // Move objects into hand
    ModelPtr objectModel = world->GetModel(object);
    EXPECT_TRUE(objectModel);

    // Align x and y with palmPose, move Z based on radius of the object
    math::Pose objectPose = palmPose;
    objectPose.pos.z -= objectModel->GetBoundingBox()->GetZLength()*0.51;
    objectModel->SetWorldPose(objectPose);

    // Temporarily disable object's gravity while closing fingers?
    objectModel->SetGravityMode(false);

    hxRobotInfo robotInfo;
    hxCommand cmd;
    hxSensor sensor;

    // Close fingers using haptix-comm (of course), test motors
    EXPECT_EQ(haptix::comm::hx_robot_info(&robotInfo), haptix::comm::hxOK)

    // TODO fill cmd
    
    for (int i = 0; i < 10 && haptix::comm::hx_update(&cmd, &sensor) !=
        haptix::comm::hxOK; i++)
    {
      gzmsg << "hx_update not OK, trying command again" << std::endl;
    }
    EXPECT_TRUE(world->GetPhysicsEngine());
    // TODO: std::ceil?
    // Wait 3 seconds sim time
    world->Step(static_cast<int>(world->GetPhysicsEngine()->GetRealTimeFactor()*3));
  
    objectModel->SetGravityMode(true);
    // Wait 10 seconds sim time
    world->Step(static_cast<int>(world->GetPhysicsEngine()->GetRealTimeFactor()*10));

    // Verify object is still in the hand: allow for ~5cm of error
    math::Pose finalPose = objectModel->GetWorldPose();
    EXPECT_NEAR(objectPose.pos.x, finalPose.pos.x, 0.05);
    EXPECT_NEAR(objectPose.pos.y, finalPose.pos.y, 0.05);
    EXPECT_NEAR(objectPose.pos.z, finalPose.pos.z, 0.05);
  }*/
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

