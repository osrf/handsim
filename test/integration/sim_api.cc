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
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  rendering::UserCameraPtr camera(new rendering::UserCamera("default", scene));
  ASSERT_TRUE(camera != NULL);

  gui::set_active_camera(camera);

  camera->Load();
  camera->Init();
  math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  camera->SetWorldPose(cameraPose);
  
  // Wait a little while for the world to initialize
  world->Step(20);

  hxSimInfo simInfo;
  ASSERT_EQ(hxs_siminfo(&simInfo), hxOK);

  math::Pose cameraOut;
  HaptixWorldPlugin::ConvertTransform(simInfo.camera_transform, cameraOut);

  // Verify object locations, camera pose, etc.
  EXPECT_EQ(cameraPose.pos, cameraOut.pos);

  EXPECT_FLOAT_EQ(cameraPose.rot.w, cameraOut.rot.w);
  EXPECT_FLOAT_EQ(cameraPose.rot.x, cameraOut.rot.x);
  EXPECT_FLOAT_EQ(cameraPose.rot.y, cameraOut.rot.y);
  EXPECT_FLOAT_EQ(cameraPose.rot.z, cameraOut.rot.z);

  for (int i = 0; i < simInfo.modelCount; ++i)
  {
    physics::ModelPtr gzModel = world->GetModel(simInfo.models[i].name);
    ASSERT_TRUE(gzModel != NULL);

    math::Pose modelPose;
    HaptixWorldPlugin::ConvertTransform(simInfo.models[i].transform, modelPose);
    EXPECT_EQ(modelPose, gzModel->GetWorldPose());

    for (int j = 0; j < simInfo.models[i].link_count; ++j)
    {
      physics::LinkPtr gzLink = gzModel->GetLink(simInfo.models[i].links[j].name);
      ASSERT_TRUE(gzLink != NULL);

      math::Pose linkPose;
      HaptixWorldPlugin::ConvertTransform(simInfo.models[i].links[j].transform,
          linkPose);
      EXPECT_EQ(linkPose, gzLink->GetWorldPose());

      math::Vector3 tmp;
      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].linvel, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldLinearVel());

      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].angvel, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldAngularVel());

      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].linacc, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldLinearAccel());

      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].angacc, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldAngularAccel());
    }

    for (int j = 0; j < simInfo.models[i].joint_count; ++j)
    {
      physics::JointPtr gzJoint = gzModel->GetJoint(simInfo.models[i].joints[j].name);
      ASSERT_TRUE(gzJoint != NULL);

      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].pos,
          gzJoint->GetAngle(0).Radian());
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].vel, gzJoint->GetVelocity(0));
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].torque_passive,
          gzJoint->GetForceTorque(0).body1Torque.GetLength());
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].torque_motor,
          gzJoint->GetLinkTorque(0).GetLength());
    }
    // TODO gravity test?
  }
}

TEST_F(SimApiTest, HxsCameraTransform)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  rendering::UserCameraPtr camera(new rendering::UserCamera("default", scene));
  ASSERT_TRUE(camera != NULL);
  gui::set_active_camera(camera);

  camera->Load();
  camera->Init();
  math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  camera->SetWorldPose(cameraPose);

  hxTransform transform;
  ASSERT_EQ(hxs_camera_transform(&transform), hxOK);

  math::Pose cameraOut;
  HaptixWorldPlugin::ConvertTransform(transform, cameraOut);

  // Verify camera pose
  EXPECT_EQ(cameraPose.pos, cameraOut.pos);

  EXPECT_FLOAT_EQ(cameraPose.rot.w, cameraOut.rot.w);
  EXPECT_FLOAT_EQ(cameraPose.rot.x, cameraOut.rot.x);
  EXPECT_FLOAT_EQ(cameraPose.rot.y, cameraOut.rot.y);
  EXPECT_FLOAT_EQ(cameraPose.rot.z, cameraOut.rot.z);
}

TEST_F(SimApiTest, HxsSetCameraTransform)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  rendering::UserCameraPtr camera(new rendering::UserCamera("default", scene));
  ASSERT_TRUE(camera != NULL);
  gui::set_active_camera(camera);

  camera->Load();
  camera->Init();

  hxTransform transform;
  transform.pos.x = 1;
  transform.pos.y = 2;
  transform.pos.z = 3;

  math::Quaternion q(3.14159, 1.570796, 0);

  transform.orient.w = q.w;
  transform.orient.x = q.x;
  transform.orient.y = q.y;
  transform.orient.z = q.z;

  ASSERT_EQ(hxs_set_camera_transform(&transform), hxOK);

  math::Pose outputPose = gui::get_active_camera()->GetWorldPose();
  EXPECT_EQ(outputPose.pos, math::Vector3(1, 2, 3));

  EXPECT_FLOAT_EQ(outputPose.rot.w, q.w);
  EXPECT_FLOAT_EQ(outputPose.rot.x, q.x);
  EXPECT_FLOAT_EQ(outputPose.rot.y, q.y);
  EXPECT_FLOAT_EQ(outputPose.rot.z, q.z);
}

TEST_F(SimApiTest, HxsContacts)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  hxContactPoints contactPoints;
  ASSERT_EQ(hxs_contacts("table", &contactPoints), hxOK);

  // TODO
}

TEST_F(SimApiTest, HxsSetState)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  world->Step(5);

  physics::ModelPtr gzDoorModel = world->GetModel("door");
  ASSERT_TRUE(gzDoorModel != NULL);
  hxModel doorModel;
  HaptixWorldPlugin::ConvertModel(*gzDoorModel, doorModel);
  // Find the door's hinge joint
  // Maybe we should have a function for this?
  int i = 0;
  for (; i < doorModel.joint_count; i++)
  {
    if (strncmp(doorModel.joints[i].name, "hinge", 5) == 0)
    {
      break;
    }
  }
  ASSERT_LT(i, doorModel.joint_count);
  // Set the door's hinge joint to its lower limit
  doorModel.joints[i].pos = -1.58;
  doorModel.joints[i].vel = 0.01;

  ASSERT_EQ(hxs_set_state(&doorModel), hxOK);

  ASSERT_TRUE(gzDoorModel->GetJoint("hinge") != NULL);
  EXPECT_FLOAT_EQ(doorModel.joints[i].pos,
      gzDoorModel->GetJoint("hinge")->GetAngle(0).Radian());

  // TODO why does GetVelocity return 0?
  /*EXPECT_FLOAT_EQ(doorModel.joints[i].vel,
      gzDoorModel->GetJoint("hinge")->GetVelocity(0));*/
}

TEST_F(SimApiTest, HxsAddModel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  std::string modelSDF =
    "<sdf version=\"1.5\">\
      <model name=\"new_model\">\
        <link name=\"link1\">\
          <pose>0 0 0.0375 0 0 0</pose>\
          <collision name=\"collision\">\
            <geometry>\
              <box>\
                <size>0.075 0.075 0.075</size>\
              </box>\
            </geometry>\
          </collision>\
          <visual name=\"visual\">\
            <geometry>\
              <box>\
                <size>0.075 0.075 0.075</size>\
              </box>\
            </geometry>\
          </visual>\
        </link>\
        <link name=\"link2\">\
          <pose>0.06 0 0.0375 0 0 0</pose>\
          <collision name=\"collision\">\
            <geometry>\
              <box>\
                <size>0.075 0.075 0.075</size>\
              </box>\
            </geometry>\
          </collision>\
          <visual name=\"visual\">\
            <geometry>\
              <box>\
                <size>0.075 0.075 0.075</size>\
              </box>\
            </geometry>\
          </visual>\
        </link>\
        <joint name=\"joint\" type=\"revolute\">\
          <parent>link1</parent>\
          <child>link2</child>\
          <axis>\
            <xyz>0 0 1</xyz>\
            <limit>\
              <lower>0</lower>\
              <upper>0</upper>\
            </limit>\
            <use_parent_model_frame>true</use_parent_model_frame>\
          </axis>\
        </joint>\
      </model>\
    </sdf>";

  std::string name = "new_model";
  hxModel model;

  float x = 0;
  float y = 0.1;
  float z = 0.2;
  float roll = 3.14159;
  float pitch = 1.570796;
  float yaw = -1.570796;

  math::Quaternion q(roll, pitch, yaw);

  // Create a new model.
  ASSERT_EQ(hxs_add_model(modelSDF.c_str(), name.c_str(), x, y, z, roll,
    pitch, yaw, false, &model), hxOK);

  // Verify that the new model matches the pose we specified.
  EXPECT_FLOAT_EQ(model.transform.pos.x, 0);
  EXPECT_FLOAT_EQ(model.transform.pos.y, 0.1);
  EXPECT_FLOAT_EQ(model.transform.pos.z, 0.2);

  EXPECT_FLOAT_EQ(model.transform.orient.w, q.w);
  EXPECT_FLOAT_EQ(model.transform.orient.x, q.x);
  EXPECT_FLOAT_EQ(model.transform.orient.y, q.y);
  EXPECT_FLOAT_EQ(model.transform.orient.z, q.z);

  EXPECT_EQ(model.joint_count, 1);
  EXPECT_EQ(model.link_count, 2);
}

TEST_F(SimApiTest, HxsLinearVel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  hxVector3 linvel;
  linvel.x = -0.01;
  linvel.y = -0.02;
  linvel.z = -0.03;

  ASSERT_EQ(hxs_linear_velocity("wood_cube_5cm", &linvel), hxOK);
  
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  EXPECT_EQ(model->GetWorldLinearVel(), math::Vector3(-0.01, -0.02, -0.03));
}

TEST_F(SimApiTest, HxsAngularVel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  hxVector3 angvel;
  angvel.x = -0.01;
  angvel.y = -0.02;
  angvel.z = -0.03;

  ASSERT_EQ(hxs_angular_velocity("wood_cube_5cm", &angvel), hxOK);
  
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);

  EXPECT_EQ(model->GetWorldAngularVel(), math::Vector3(-0.01, -0.02, -0.03));
}

TEST_F(SimApiTest, HxsForce)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity makes it easier to verify test
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxVector3 force;
  force.x = -0.01;
  force.y = -0.02;
  force.z = 0.03;

  float duration = 1.0;
  math::Vector3 gzForce(-0.01, -0.02, 0.03);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  ASSERT_EQ(hxs_force("wood_cube_5cm", "link", &force, duration), hxOK);
  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; i++)
  {
    EXPECT_EQ(link->GetWorldForce(), gzForce);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; i++)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    world->Step(100);
  }
}

TEST_F(SimApiTest, HxsTorque)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity makes it easier to verify test
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxVector3 torque;
  torque.x = -0.01;
  torque.y = -0.02;
  torque.z = -0.03;

  float duration = 1.0;
  math::Vector3 gzTorque(-0.01, -0.02, -0.03);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  ASSERT_EQ(hxs_torque("wood_cube_5cm", "link", &torque, duration), hxOK);

  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; i++)
  {
    EXPECT_EQ(link->GetWorldTorque(), gzTorque);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; i++)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    world->Step(100);
  }
}

TEST_F(SimApiTest, HxsRemoveModel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  ASSERT_EQ(hxs_remove_model("wood_cube_5cm"), hxOK);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  EXPECT_FALSE(model);
}

TEST_F(SimApiTest, HxsReset)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);
  // Get everything's initial pose
  std::map<std::string, math::Pose> initialPoses;
  for (auto model : world->GetModels())
  {
    initialPoses[model->GetName()] = model->GetWorldPose();
  }
  // Move everything
  for (auto model : world->GetModels())
  {
    math::Pose targetPose = initialPoses[model->GetName()];
    targetPose.pos += math::Vector3(1, 2, 3);
    model->SetWorldPose(targetPose);
  }
  world->Step(2);

  ASSERT_EQ(hxs_reset(0), hxOK);
  world->Step(2);
  for (auto model : world->GetModels())
  {
    EXPECT_NEAR(model->GetWorldPose().pos.x, initialPoses[model->GetName()].pos.x, 1.3e-2);
    EXPECT_NEAR(model->GetWorldPose().pos.y, initialPoses[model->GetName()].pos.y, 1.3e-2);
    EXPECT_NEAR(model->GetWorldPose().pos.z, initialPoses[model->GetName()].pos.z, 1.3e-2);

    EXPECT_NEAR(model->GetWorldPose().rot.w, initialPoses[model->GetName()].rot.w, 1.3e-2);
    EXPECT_NEAR(model->GetWorldPose().rot.x, initialPoses[model->GetName()].rot.x, 1.3e-2);
    EXPECT_NEAR(model->GetWorldPose().rot.y, initialPoses[model->GetName()].rot.y, 1.3e-2);
    EXPECT_NEAR(model->GetWorldPose().rot.z, initialPoses[model->GetName()].rot.z, 1.3e-2);
  }

  // Move everything again
  for (auto model : world->GetModels())
  {
    math::Pose targetPose = initialPoses[model->GetName()];
    targetPose.pos += math::Vector3(1, 2, 3);
    model->SetWorldPose(targetPose);
  }
  world->Step(2);

  ASSERT_EQ(hxs_reset(1), hxOK);
  world->Step(2);
  // Expect that everything is in its initial state
  // TODO move arm command!
  for (auto model : world->GetModels())
  {
    if (model->GetName() != "mpl_haptix_right_forearm")
    {
      EXPECT_NEAR(model->GetWorldPose().pos.x, initialPoses[model->GetName()].pos.x, 1.3e-2);
      EXPECT_NEAR(model->GetWorldPose().pos.y, initialPoses[model->GetName()].pos.y, 1.3e-2);
      EXPECT_NEAR(model->GetWorldPose().pos.z, initialPoses[model->GetName()].pos.z, 1.3e-2);

      EXPECT_NEAR(model->GetWorldPose().rot.w, initialPoses[model->GetName()].rot.w, 1.3e-2);
      EXPECT_NEAR(model->GetWorldPose().rot.x, initialPoses[model->GetName()].rot.x, 1.3e-2);
      EXPECT_NEAR(model->GetWorldPose().rot.y, initialPoses[model->GetName()].rot.y, 1.3e-2);
      EXPECT_NEAR(model->GetWorldPose().rot.z, initialPoses[model->GetName()].rot.z, 1.3e-2);
    }
    else
    {
      math::Pose targetPose = initialPoses[model->GetName()];
      targetPose.pos += math::Vector3(1, 2, 3);
      EXPECT_EQ(model->GetWorldPose().pos, targetPose.pos);

      EXPECT_NEAR(model->GetWorldPose().rot.w, targetPose.rot.w, 1e-1);
      EXPECT_NEAR(model->GetWorldPose().rot.x, targetPose.rot.x, 1e-1);
      EXPECT_NEAR(model->GetWorldPose().rot.y, targetPose.rot.y, 1e-1);
      EXPECT_NEAR(model->GetWorldPose().rot.z, targetPose.rot.z, 1e-1);
    }
  }
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

TEST_F(SimApiTest, HxsModelGravity)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  int gravity;
  ASSERT_EQ(hxs_model_gravity("wood_cube_5cm", &gravity), hxOK);

  EXPECT_EQ(gravity, 1);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  model->SetGravityMode(0);

  ASSERT_EQ(hxs_model_gravity("wood_cube_5cm", &gravity), hxOK);

  EXPECT_EQ(gravity, 0);
}

TEST_F(SimApiTest, HxsSetModelGravity)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  int gravity = 0;
  ASSERT_EQ(hxs_set_model_gravity("wood_cube_5cm", gravity), hxOK);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  for (auto link : model->GetLinks())
  {
    EXPECT_FALSE(link->GetGravityMode());
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
