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
#include <haptix/comm/haptix.h>

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
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat.world");
  ASSERT_TRUE(world != NULL);

  // Get joint limits indexed on the numerical motor index
  gazebo::physics::ModelPtr armModel = world->GetModel("mpl_haptix_right_forearm");
  ASSERT_TRUE(armModel != NULL);

  sdf::ElementPtr modelSDF = armModel->GetSDF();
  ASSERT_TRUE(modelSDF != NULL);
  // TODO: will have to change if multiple ModelPlugins
  sdf::ElementPtr pluginSDF = modelSDF->GetElement("plugin");

  ASSERT_TRUE(pluginSDF != NULL);

  std::map<int, std::pair<double, double>> indexMotorLimitMap;
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

  hxRobotInfo robot_info;
  ASSERT_EQ(hx_robot_info(&robot_info), hxOK);

  // Expect motor limits to equal limits from SDF
  for (int i = 0; i < robot_info.motor_count; i++)
  {
    EXPECT_DOUBLE_EQ(robot_info.motor_limit[i][0], indexMotorLimitMap[i].first);
    EXPECT_DOUBLE_EQ(robot_info.motor_limit[i][1], indexMotorLimitMap[i].second);
  }
}

/////////////////////////////////////////////////
TEST_F(Issue86Test, MotorPositions)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat.world");
  ASSERT_TRUE(world != NULL);

  hxRobotInfo robot_info;
  hxCommand command;
  hxSensor sensor;
  ASSERT_EQ(hx_robot_info(&robot_info), hxOK);

  // Expect that the hand starts at 0
  hx_read_sensors(&sensor);
  for (int i = 0; i < robot_info.motor_count; i++)
  {
    EXPECT_DOUBLE_EQ(sensor.motor_pos[i], 0);
  }

  // Command the hand to some position
  command.ref_pos_enabled = 1;
  for (int i = 0; i < robot_info.motor_count; i++)
  {
    command.ref_pos[i] = i*0.1;
  }
  hx_update(&command, &sensor);

  // wait a moment
  gazebo::common::Time::Sleep(1);

  hx_update(&command, &sensor);

  // Expect that update will have the correct command
  for (int i = 0; i < robot_info.motor_count; i++)
  {
    // TODO: near and give some room?
    EXPECT_DOUBLE_EQ(command.ref_pos[i], sensor.motor_pos[i]);
  }
}
