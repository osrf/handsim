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

#include <limits>
#include <gtest/gtest.h>
#include <haptix/comm/haptix.h>
#include <ignition/math/Rand.hh>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test_config.h"

#define TOL 1e-3

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
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat.world");
  ASSERT_TRUE(world != NULL);

  world->Step(1);

  // Get joint limits indexed on the numerical motor index
  gazebo::physics::ModelPtr armModel = world->GetModel("mpl_haptix_right_forearm");
  ASSERT_TRUE(armModel != NULL);

  sdf::ElementPtr modelSDF = armModel->GetSDF();
  ASSERT_TRUE(modelSDF != NULL);
  // TODO: will have to change if multiple ModelPlugins
  sdf::ElementPtr pluginSDF = modelSDF->GetElement("plugin");

  ASSERT_TRUE(pluginSDF != NULL);

  std::map<int, std::pair<double, double>> indexMotorLimitMap;
  /*c omputer from joint limit? might be hard to do
  for (sdf::ElementPtr motorSDF = pluginSDF->GetElement("motor"); motorSDF;
        motorSDF = pluginSDF->GetNextElement("motor"))
  {
    ASSERT_TRUE(motorSDF->HasAttribute("id"));
    int id = motorSDF->Get<int>("id");
    ASSERT_TRUE(motorSDF->HasElement("powered_motor_joint"));
    std::string jointName =
        motorSDF->GetElement("powered_motor_joint")->Get<std::string>();
    gazebo::physics::JointPtr joint = armModel->GetJoint(jointName);
    indexMotorLimitMap[id] =
        std::pair<double, double>(joint->GetLowerLimit(0).Radian(),
                                  joint->GetUpperLimit(0).Radian());
  }
  */
  // manually set target results by hard coding for now.
  // this test will at least detect changes in code.
  indexMotorLimitMap[0] = std::pair<double, double>(-282.6, 282.6);
  indexMotorLimitMap[1] =
    std::pair<double, double>(-46.98, 141.30000305175781);
  indexMotorLimitMap[2] =
    std::pair<double, double>(-187.19999694824219, 187.19999694824219);
  indexMotorLimitMap[3] =
    std::pair<double, double>(0, 728.6400146484375);
  indexMotorLimitMap[4] =
    std::pair<double, double>(0, 362.55999755859375);
  indexMotorLimitMap[5] =
    std::pair<double, double>(0, 362.55999755859375);
  indexMotorLimitMap[6] =
    std::pair<double, double>(-288.28799438476562, 450.55999755859375);
  indexMotorLimitMap[7] =
    std::pair<double, double>(-121.44000244140625, 0);
  indexMotorLimitMap[8] =
    std::pair<double, double>(0, 378.50812);
  indexMotorLimitMap[9] =
    std::pair<double, double>(0, 378.50812);
  indexMotorLimitMap[10] =
    std::pair<double, double>(0, 378.50812);
  indexMotorLimitMap[11] =
    std::pair<double, double>(0, 121.44);
  indexMotorLimitMap[12] =
    std::pair<double, double>(0, 378.50812);

  world->Step(1);
  hxRobotInfo robot_info;
  ASSERT_EQ(hx_robot_info(&robot_info), hxOK);

  // Expect motor limits to equal limits from SDF
  for (int i = 0; i < robot_info.motor_count; i++)
  {
    gzdbg << "testing motor [" << i << "]\n";
    EXPECT_FLOAT_EQ(robot_info.motor_limit[i][0],
      indexMotorLimitMap[i].first);
    EXPECT_FLOAT_EQ(robot_info.motor_limit[i][1],
      indexMotorLimitMap[i].second);
  }
}

/////////////////////////////////////////////////
TEST_F(Issue86Test, MotorPositions)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat.world");
  ASSERT_TRUE(world != NULL);

  world->GetPhysicsEngine()->SetGravity(gazebo::math::Vector3(0, 0, 0));

  world->Step(1);

  hxRobotInfo robot_info;
  hxSensor sensor;
  ASSERT_EQ(hx_robot_info(&robot_info), hxOK);

  // manually set target results by hard coding for now.
  // this test will at least detect changes in code.
  std::map<int, std::pair<double, double>> indexMotorLimitMap;
  indexMotorLimitMap[0] = std::pair<double, double>(-282.6, 282.6);
  indexMotorLimitMap[1] =
    std::pair<double, double>(-46.98, 141.30000305175781);
  indexMotorLimitMap[2] =
    std::pair<double, double>(-187.19999694824219, 187.19999694824219);
  indexMotorLimitMap[3] =
    std::pair<double, double>(0, 728.6400146484375);
  indexMotorLimitMap[4] =
    std::pair<double, double>(0, 362.55999755859375);
  indexMotorLimitMap[5] =
    std::pair<double, double>(0, 362.55999755859375);
  indexMotorLimitMap[6] =
    std::pair<double, double>(-288.28799438476562, 450.55999755859375);
  indexMotorLimitMap[7] =
    std::pair<double, double>(-121.44000244140625, 0);
  indexMotorLimitMap[8] =
    std::pair<double, double>(0, 0.96312499046325684);
  indexMotorLimitMap[9] =
    std::pair<double, double>(0, 0.96312499046325684);
  indexMotorLimitMap[10] =
    std::pair<double, double>(0, 0.96312499046325684);
  indexMotorLimitMap[11] =
    std::pair<double, double>(0, 121.44);
  indexMotorLimitMap[12] =
    std::pair<double, double>(0, 0.96312499046325684);
  // Expect that the hand starts at 0

  hx_read_sensors(&sensor);
  for (int i = 0; i < robot_info.motor_count; i++)
  {
    EXPECT_DOUBLE_EQ(sensor.motor_pos[i], 0);
  }

  for (int j = 0; j < 3; j++)
  {
    hxCommand command;
    // Create a new command based on a sinusoidal wave.
    for (int i = 0; i < robot_info.motor_count; ++i)
    {
      // Set the desired position of this motor
      if (j == 0)
        command.ref_pos[i] = indexMotorLimitMap[i].first;
      else if (j == 1)
        command.ref_pos[i] = 0.5 * (
          indexMotorLimitMap[i].first + indexMotorLimitMap[i].second);
      else if (j == 2)
        command.ref_pos[i] = 0.8*indexMotorLimitMap[i].second;
    }
    command.ref_pos_enabled = 1;
    command.ref_vel_max_enabled = 0;
    command.gain_pos_enabled = 0;
    command.gain_vel_enabled = 0;

    // Send the new joint command and receive the state update.
    if (hx_update(&command, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }
    world->Step(6000);

    unsigned int sleeptime_us = 10000;
    usleep(sleeptime_us);
    hx_update(&command, &sensor);

    for (int i = 0; i < robot_info.motor_count; ++i)
    {
      gzdbg << "Verifying motor index: " << i << std::endl;
      gzdbg << command.ref_pos[i] << " : " << sensor.motor_pos[i] << "\n";
      EXPECT_NEAR(command.ref_pos[i], sensor.motor_pos[i], TOL);
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Get a random partition name.
  auto rnd = ignition::math::Rand::IntUniform(0,
      std::numeric_limits<int>::max());
  std::string partition = std::to_string(rnd);

  // Set the partition name for this process.
  setenv("IGN_PARTITION", partition.c_str(), 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
