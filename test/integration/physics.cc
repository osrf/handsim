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

#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/test/ServerFixture.hh>
#include <ignition/transport.hh>

#include "handsim/HaptixWorldPlugin.hh"
#include "haptix/comm/haptix_sim.h"
#include "haptix/comm/haptix.h"
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxGrasp.pb.h>
#include "test_config.h"

class PhysicsTest : public gazebo::ServerFixture
{
  public: gazebo::physics::WorldPtr InitWorld(const std::string &_worldFile);
  public: ignition::transport::Node ignNode;
};

gazebo::physics::WorldPtr PhysicsTest::InitWorld(const std::string &_worldFile)
{
  boost::filesystem::path path = HANDSIM_TEST_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path.string());
  Load(_worldFile, true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  return world;
}

TEST_F(PhysicsTest, Test1)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // step to make sure the arm loaded correctly
  world->Step(1);

  // set arm position
  hxsTransform armT;
  // ASSERT_EQ(hxs_model_transform("mpl_haptix_right_forearm", &armT), hxOK);
  ignition::math::Vector3<double> p(0.03, -0.88, 1.24);
  armT.pos.x = p[0];
  armT.pos.y = p[1];
  armT.pos.z = p[2];
  ignition::math::Quaternion<double> q(0.351391, 0.000060, -3.141582);
  armT.orient.w = q.W();
  armT.orient.x = q.X();
  armT.orient.y = q.Y();
  armT.orient.z = q.Z();
  ASSERT_EQ(hxs_set_model_transform("mpl_haptix_right_forearm", &armT), hxOK);
  // send same command to arm base pose controller
  gazebo::msgs::Pose msg =
    gazebo::msgs::Convert(ignition::math::Pose3<double>(p, q));
  this->ignNode.Advertise("/haptix/arm_pose_inc");
  this->ignNode.Advertise("/haptix/arm_model_pose");
  this->ignNode.Publish("/haptix/arm_model_pose", msg);

  hxsTransform woodT;
  woodT.pos.x    = 0.0;
  woodT.pos.y    = -0.25;
  woodT.pos.z    = 1.0336;
  woodT.orient.w = 1.0;
  woodT.orient.x = 0.0;
  woodT.orient.y = 0.0;
  woodT.orient.z = 0.0;
  ASSERT_EQ(hxs_set_model_transform("wood_cube_5cm", &woodT), hxOK);

  world->Step(1);
  // gzerr << "ready to grab"; getchar();

  // call hxs_sim_info to get motor_count
  ::hxRobotInfo robotInfo;
  ASSERT_EQ(::hx_robot_info(&robotInfo), ::hxOK);
  ::hxCommand lastMotorCommand;
  ::hxSensor lastSensor;
  // setup grasp command message
  std::string name = "2Pinch";
  haptix::comm::msgs::hxGrasp grasp;
  haptix::comm::msgs::hxGrasp::hxGraspValue* gv = grasp.add_grasps();
  gv->set_grasp_name(name);
  for (int n = 0; n < 76; ++n)
  {
    // set grasp value
    gv->set_grasp_value((double)n/100);

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
    unsigned int numWristMotors = 3;
    for (int i = numWristMotors; i < robotInfo.motor_count; ++i)
    {
      lastMotorCommand.ref_pos[i] = resp.ref_pos(i);
    }
    lastMotorCommand.ref_pos_enabled = 1;
    lastMotorCommand.ref_vel_max_enabled = 0;
    lastMotorCommand.gain_pos_enabled = 0;
    lastMotorCommand.gain_vel_enabled = 0;
    if(::hx_update(&lastMotorCommand, &lastSensor) != ::hxOK)
    {
      gzerr << "hx_update(): Request error.\n" << std::endl;
      return;
    }
    world->Step(10);
  }
  // gzerr << "grasped"; getchar();

  // set new arm position
  p.Set(0, 0, 0.001);
  q.Set(1, 0, 0, 0);
  msg = gazebo::msgs::Convert(ignition::math::Pose3<double>(p, q));
  for (int n = 0; n < 200; ++n)
  {
    this->ignNode.Publish("/haptix/arm_pose_inc", msg);
    world->Step(10);
  }

  // gzerr << "moved"; getchar();
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  gazebo::math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
