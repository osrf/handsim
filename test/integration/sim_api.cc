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

#include "ServerFixture.hh"
#include "haptix/comm/haptix_sim.h"
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/GuiIface.hh>
#include "handsim/HaptixWorldPlugin.hh"

using namespace gazebo;

class SimApiTest : public ServerFixture
                   //public testing::WithParamInterface<const char*>
{
};

TEST_F(SimApiTest, HxsSimInfo)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  if (!scene)
  {
    FAIL();
  }

  rendering::UserCameraPtr camera(new rendering::UserCamera("default", scene));
  gui::set_active_camera(camera);

  if (!camera)
  {
    FAIL();
  }
  camera->Load();
  camera->Init();
  math::Pose cameraPose(1, 2, 3, 4, 5, 6);
  camera->SetWorldPose(cameraPose);
  
  // Wait a little while for the world to initialize
  world->Step(20);

  hxSimInfo simInfo;
  EXPECT_EQ(hxs_siminfo(&simInfo), hxOK);

  math::Pose cameraOut;
  HaptixWorldPlugin::ConvertTransform(simInfo.camera_transform, cameraOut);

  // Verify object locations, camera pose, etc.
  EXPECT_EQ(cameraPose.pos, cameraOut.pos);
  EXPECT_EQ(cameraPose.rot, -cameraOut.rot);

  /*EXPECT_EQ();

  for (auto model : simInfo.models)
  {
  }*/
}

TEST_F(SimApiTest, HxsCameraTransform)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  if (!scene)
  {
    FAIL();
  }

  rendering::UserCameraPtr camera(new rendering::UserCamera("default", scene));
  gui::set_active_camera(camera);

  if (!camera)
  {
    FAIL();
  }
  camera->Load();
  camera->Init();
  math::Pose cameraPose(1, 2, 3, 4, 5, 6);
  camera->SetWorldPose(cameraPose);

  hxTransform transform;
  EXPECT_EQ(hxs_camera_transform(&transform), hxOK);

  // TODO verify pose
}

TEST_F(SimApiTest, HxsSetCameraTransform)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  if (!scene)
  {
    FAIL();
  }

  rendering::UserCameraPtr camera(new rendering::UserCamera("default", scene));
  gui::set_active_camera(camera);

  if (!camera)
  {
    FAIL();
  }
  camera->Load();
  camera->Init();
  math::Pose cameraPose(1, 2, 3, 4, 5, 6);
  camera->SetWorldPose(cameraPose);

  hxTransform transform;
  transform.pos.x = 3;
  transform.pos.y = 2;
  transform.pos.z = 1;

  transform.orient.w = 4;
  transform.orient.x = 5;
  transform.orient.y = 6;
  transform.orient.z = 7;

  EXPECT_EQ(hxs_set_camera_transform(&transform), hxOK);

  // TODO verify pose
}

TEST_F(SimApiTest, HxsContacts)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }

  // Wait a little while for the world to initialize
  world->Step(20);

  hxContactPoints contactPoints;
  ASSERT_EQ(hxs_contacts("table", &contactPoints), hxOK);

}

TEST_F(SimApiTest, HxsSetState)
{
  // TODO SetState

  // hxModel desiredCube;
  // TODO Set a desired state for the cube

  //ASSERT_EQ(hxs_set_state(desiredCube));

  // TODO Expect that the state of the cube from Gazebo matches what we commanded
}

TEST_F(SimApiTest, HxsAddModel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  // TODO SDF
  std::string modelSDF = "";
  std::string name = "new_model";
  hxModel model;

  float x = 1.0;
  float y = 2.0;
  float z = 3.0;
  float roll = 4.0;
  float pitch = 5.0;
  float yaw = 6.0;

  // Create a new model.
  EXPECT_EQ(hxs_add_model(modelSDF.c_str(), name.c_str(), x, y, z, roll, pitch, yaw,
    &model), hxOK);

  // Verify that the new model matches the pose we specified.
  EXPECT_FLOAT_EQ(model.transform.pos.x, 0);
  EXPECT_FLOAT_EQ(model.transform.pos.y, 0.1);
  EXPECT_FLOAT_EQ(model.transform.pos.z, 0.2);
  EXPECT_FLOAT_EQ(model.transform.orient.w, 0.3);
  EXPECT_FLOAT_EQ(model.transform.orient.x, 0.4);
  EXPECT_FLOAT_EQ(model.transform.orient.y, 0.5);

  EXPECT_FLOAT_EQ(model.transform.orient.z, 0.6);

  // TODO Expect that its links and joints match what was specified in SDF
}

TEST_F(SimApiTest, HxsLinearVel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  hxVector3 linvel;
  linvel.x = -0.01;
  linvel.y = -0.02;
  linvel.z = -0.03;

  EXPECT_EQ(hxs_linear_velocity("wood_cube_5cm", &linvel), hxOK);
  
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  if (!model)
  {
    FAIL();
  }
  EXPECT_EQ(model->GetWorldLinearVel(), math::Vector3(-0.01, -0.02, -0.03));
}

TEST_F(SimApiTest, HxsAngularVel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  hxVector3 angvel;
  angvel.x = -0.01;
  angvel.y = -0.02;
  angvel.z = -0.03;

  EXPECT_EQ(hxs_angular_velocity("wood_cube_5cm", &angvel), hxOK);
  
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  if (!model)
  {
    FAIL();
  }
  EXPECT_EQ(model->GetWorldAngularVel(), math::Vector3(-0.01, -0.02, -0.03));
}

TEST_F(SimApiTest, HxsForce)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  hxVector3 force;
  force.x = -0.01;
  force.y = -0.02;
  force.z = 0.03;

  float duration = 1.0;

  EXPECT_EQ(hxs_force("wood_cube_5cm", "link", &force, duration), hxOK);

  // TODO Check force for a duration
  
}

TEST_F(SimApiTest, HxsTorque)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  hxVector3 torque;
  torque.x = -0.01;
  torque.y = -0.02;
  torque.z = -0.03;

  float duration = 1.0;

  EXPECT_EQ(hxs_torque("wood_cube_5cm", "link", &torque, duration), hxOK);

  // TODO Check torque for a duration
}

TEST_F(SimApiTest, HxsRemoveModel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  EXPECT_EQ(hxs_remove_model("wood_cube_5cm"), hxOK);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  EXPECT_FALSE(model);
}

TEST_F(SimApiTest, HxsReset)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    FAIL();
  }
  // Wait a little while for the world to initialize
  world->Step(20);

  EXPECT_EQ(hxs_reset(0), hxOK);

  // TODO Check that everything is in its initial state
  // TODO move stuff

  EXPECT_EQ(hxs_reset(1), hxOK);

  // TODO Check that the limb pose was also reset
}

TEST_F(SimApiTest, HxsResetTimer)
{
  // TODO
}


TEST_F(SimApiTest, HxsStartTimer)
{
  // TODO
}


TEST_F(SimApiTest, HxsTimer)
{
  // TODO
}


TEST_F(SimApiTest, HxsStartLogging)
{
  // TODO
}


TEST_F(SimApiTest, HxsStopLogging)
{
  // TODO
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
