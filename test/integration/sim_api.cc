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

#include <boost/filesystem/path.hpp>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo/rendering/UserCamera.hh>

#include "handsim/HaptixWorldPlugin.hh"

#include "haptix/comm/haptix_sim.h"

#include "test_config.h"

using namespace gazebo;

class SimApiTest : public ServerFixture
{
  public: physics::WorldPtr InitWorld(const std::string _worldFile);
};

physics::WorldPtr SimApiTest::InitWorld(const std::string _worldFile)
{
  boost::filesystem::path path = HANDSIM_TEST_PATH;
  common::SystemPaths::Instance()->AddGazeboPaths(path.string());
  Load(_worldFile, true);
  physics::WorldPtr world = physics::get_world("default");
  return world;
}

TEST_F(SimApiTest, HxsSimInfo)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  // Spawn a camera
  rendering::UserCameraPtr camera = scene->CreateUserCamera("test_camera");
  camera->SetWorldPose(cameraPose);
  camera->Update();
  gui::set_active_camera(camera);
  ASSERT_TRUE(gui::get_active_camera() != NULL);

  hxsSimInfo simInfo;
  ASSERT_EQ(hxs_sim_info(&simInfo), hxOK);
  common::Time::Sleep(1);

  math::Pose cameraOut;
  HaptixWorldPlugin::ConvertTransform(simInfo.camera_transform, cameraOut);

  // Verify object locations, camera pose, etc.
  EXPECT_EQ(cameraPose.pos, cameraOut.pos);

  EXPECT_FLOAT_EQ(cameraPose.rot.w, cameraOut.rot.w);
  EXPECT_FLOAT_EQ(cameraPose.rot.x, cameraOut.rot.x);
  EXPECT_FLOAT_EQ(cameraPose.rot.y, cameraOut.rot.y);
  EXPECT_FLOAT_EQ(cameraPose.rot.z, cameraOut.rot.z);

  for (int i = 0; i < simInfo.model_count; ++i)
  {
    physics::ModelPtr gzModel = world->GetModel(simInfo.models[i].name);
    ASSERT_TRUE(gzModel != NULL);

    math::Pose modelPose;
    HaptixWorldPlugin::ConvertTransform(simInfo.models[i].transform, modelPose);
    EXPECT_EQ(modelPose, gzModel->GetWorldPose());

    for (int j = 0; j < simInfo.models[i].link_count; ++j)
    {
      physics::LinkPtr gzLink =
          gzModel->GetLink(simInfo.models[i].links[j].name);
      ASSERT_TRUE(gzLink != NULL);

      math::Pose linkPose;
      HaptixWorldPlugin::ConvertTransform(simInfo.models[i].links[j].transform,
          linkPose);
      EXPECT_EQ(linkPose, gzLink->GetWorldPose());

      math::Vector3 tmp;
      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].lin_vel, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldLinearVel());

      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].ang_vel, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldAngularVel());

      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].lin_acc, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldLinearAccel());

      HaptixWorldPlugin::ConvertVector(simInfo.models[i].links[j].ang_acc, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldAngularAccel());
    }

    for (int j = 0; j < simInfo.models[i].joint_count; ++j)
    {
      physics::JointPtr gzJoint =
          gzModel->GetJoint(simInfo.models[i].joints[j].name);
      ASSERT_TRUE(gzJoint != NULL);

      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].pos,
          gzJoint->GetAngle(0).Radian());
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].vel, gzJoint->GetVelocity(0));
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].wrench_reactive.force.x,
          gzJoint->GetForceTorque(0).body2Force.x);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].wrench_reactive.force.y,
          gzJoint->GetForceTorque(0).body2Force.y);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].wrench_reactive.force.z,
          gzJoint->GetForceTorque(0).body2Force.z);

      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].wrench_reactive.torque.x,
          gzJoint->GetForceTorque(0).body2Torque.x);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].wrench_reactive.torque.y,
          gzJoint->GetForceTorque(0).body2Torque.y);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].wrench_reactive.torque.z,
          gzJoint->GetForceTorque(0).body2Torque.z);

      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].torque_motor,
          gzJoint->GetForce(0));
    }
    // TODO gravity_mode test?
  }
}

TEST_F(SimApiTest, HxsCameraTransform)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  // Spawn a camera
  /*SpawnCamera("test_camera_model", "test_camera", cameraPose.pos,
      cameraPose.rot.GetAsEuler());*/
  rendering::UserCameraPtr camera = scene->CreateUserCamera("test_camera");
  camera->SetWorldPose(cameraPose);
  camera->Update();
  gui::set_active_camera(camera);
  ASSERT_TRUE(gui::get_active_camera() != NULL);

  hxsTransform transform;
  ASSERT_EQ(hxs_camera_transform(&transform), hxOK);
  common::Time::Sleep(2);

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
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  // Spawn a camera
  rendering::UserCameraPtr camera = scene->CreateUserCamera("test_camera");
  gui::set_active_camera(camera);

  hxsTransform transform;
  transform.pos.x = 1;
  transform.pos.y = 2;
  transform.pos.z = 3;

  math::Quaternion q(3.14159, 1.570796, 0);

  transform.orient.w = q.w;
  transform.orient.x = q.x;
  transform.orient.y = q.y;
  transform.orient.z = q.z;

  ASSERT_EQ(hxs_set_camera_transform(&transform), hxOK);
  common::Time::Sleep(2);

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

  hxsContactPoints contactPoints;

  physics::ContactManager *contactManager =
      world->GetPhysicsEngine()->GetContactManager();
  ASSERT_TRUE(contactManager != NULL);

  physics::ModelPtr tableModel = world->GetModel("table");

  ASSERT_EQ(hxs_contacts("table", &contactPoints), hxOK);

  // Have to find contacts and sort them by distance to compare
  // since they don't have string keys

  for (auto contact : contactManager->GetContacts())
  {
    if (contact->collision1->GetModel() == tableModel)
    {
      for (int i = 0; i < contact->count; i++)
      {
        math::Vector3 linkPos =
            contact->collision1->GetLink()->GetWorldPose().pos;
        contact->positions[i] -= linkPos;
        contact->normals[i] -= linkPos;

        // Now find matching contact point as returned by hxs_contacts
        int j = 0;
        for (; j < contactPoints.contact_count; j++)
        {
          bool link1NameMatch = std::string(contactPoints.contacts[i].link1) ==
              contact->collision1->GetLink()->GetName();
          bool link2NameMatch = std::string(contactPoints.contacts[i].link2) ==
              contact->collision2->GetLink()->GetName();
          math::Vector3 contactPos, contactNormal, contactForce, contactTorque;
          HaptixWorldPlugin::ConvertVector(contactPoints.contacts[i].point,
              contactPos);
          HaptixWorldPlugin::ConvertVector(contactPoints.contacts[i].normal,
              contactNormal);
          HaptixWorldPlugin::ConvertVector(contactPoints.contacts[i].wrench.force,
              contactForce);
          HaptixWorldPlugin::ConvertVector(contactPoints.contacts[i].wrench.torque,
              contactTorque);
          if (link1NameMatch && link2NameMatch &&
              contactPos == contact->positions[i] &&
              contactNormal == contact->normals[i] &&
              contactForce == contact->wrench[i].body1Force &&
              contactTorque == contact->wrench[i].body1Torque &&
              contactPoints.contacts[i].distance == contact->depths[i])
          {
            break;
          }
        }
        EXPECT_LT(j, contactPoints.contact_count);
      }
    }
  }
}

TEST_F(SimApiTest, HxsSetModelJointState)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr gzDoorModel = world->GetModel("door");
  ASSERT_TRUE(gzDoorModel != NULL);

  world->Step(5);

  float pos = -1.58;
  float vel = 0.01;

  ASSERT_EQ(hxs_set_model_joint_state("door", "hinge", pos, vel), hxOK);

  ASSERT_TRUE(gzDoorModel->GetJoint("hinge") != NULL);
  EXPECT_FLOAT_EQ(pos, gzDoorModel->GetJoint("hinge")->GetAngle(0).Radian());

  // TODO why does GetVelocity return 0?
  EXPECT_FLOAT_EQ(vel, gzDoorModel->GetJoint("hinge")->GetVelocity(0));
}

TEST_F(SimApiTest, HxsSetModelLinkState)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr gzDoorModel = world->GetModel("door");
  ASSERT_TRUE(gzDoorModel != NULL);

  world->Step(5);

  math::Pose gzLinkPose(0, 0, 2, 3.14159, 0, 0.58);
  gzLinkPose += gzDoorModel->GetWorldPose();

  hxsTransform pose;
  HaptixWorldPlugin::ConvertTransform(gzLinkPose, pose);

  hxsVector3 lin_vel;
  lin_vel.x = -0.003;
  lin_vel.y = -0.02;
  lin_vel.z = 0;
  hxsVector3 ang_vel;
  ang_vel.x = 0;
  ang_vel.y = 0;
  ang_vel.z = 0.03;

  hxsVector3 zero;

  physics::LinkPtr doorLink = gzDoorModel->GetLink("door");
  ASSERT_TRUE(doorLink != NULL);

  ASSERT_EQ(hxs_set_model_link_state("door", "door", &pose, &zero,
      &ang_vel), hxOK);

  EXPECT_EQ(gzLinkPose, doorLink->GetWorldPose());

  EXPECT_EQ(math::Vector3(0, 0, 0.03), doorLink->GetWorldAngularVel());

  ASSERT_EQ(hxs_set_model_link_state("door", "door", &pose, &lin_vel,
      &zero), hxOK);
  EXPECT_EQ(math::Vector3(-0.003, -0.02, 0), doorLink->GetWorldLinearVel());
}

TEST_F(SimApiTest, HxsAddModel)
{
  Load("worlds/arat_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  std::string modelSDF =
    "<sdf version=\"1.5\">"
      "<model name=\"new_model\">"
        "<link name=\"link1\">"
          "<pose>0 0 0.0375 0 0 0</pose>"
          "<collision name=\"collision\">"
            "<geometry>"
              "<box>"
                "<size>0.075 0.075 0.075</size>"
              "</box>"
            "</geometry>"
          "</collision>"
          "<visual name=\"visual\">"
            "<geometry>"
              "<box>"
                "<size>0.075 0.075 0.075</size>"
              "</box>"
            "</geometry>"
          "</visual>"
        "</link>"
        "<link name=\"link2\">"
          "<pose>0.06 0 0.0375 0 0 0</pose>"
          "<collision name=\"collision\">"
            "<geometry>"
              "<box>"
                "<size>0.075 0.075 0.075</size>"
              "</box>"
            "</geometry>"
          "</collision>"
          "<visual name=\"visual\">"
            "<geometry>"
              "<box>"
                "<size>0.075 0.075 0.075</size>"
              "</box>"
            "</geometry>"
          "</visual>"
        "</link>"
        "<joint name=\"joint\" type=\"revolute\">"
          "<parent>link1</parent>"
          "<child>link2</child>"
          "<axis>"
            "<xyz>0 0 1</xyz>"
            "<limit>"
              "<lower>0</lower>"
              "<upper>0</upper>"
            "</limit>"
            "<use_parent_model_frame>true</use_parent_model_frame>"
          "</axis>"
        "</joint>"
      "</model>"
    "</sdf>";

  std::string name = "new_model";
  hxsModel model;

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
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  hxsVector3 lin_vel;
  lin_vel.x = -0.01;
  lin_vel.y = -0.02;
  lin_vel.z = -0.03;

  ASSERT_EQ(hxs_linear_velocity("wood_cube_5cm", &lin_vel), hxOK);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  EXPECT_EQ(model->GetWorldLinearVel(), math::Vector3(-0.01, -0.02, -0.03));
}

TEST_F(SimApiTest, HxsAngularVel)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  hxsVector3 ang_vel;
  ang_vel.x = -0.01;
  ang_vel.y = -0.02;
  ang_vel.z = -0.03;

  ASSERT_EQ(hxs_angular_velocity("wood_cube_5cm", &ang_vel), hxOK);

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

  // disabling gravity_mode makes it easier to verify test
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 force;
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
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity_mode makes it easier to verify test
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 torque;
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

TEST_F(SimApiTest, HxsWrench)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity_mode makes it easier to verify test
  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsWrench wrench;
  wrench.force.x = -0.01;
  wrench.force.y = -0.02;
  wrench.force.z = 0.03;
  wrench.torque.x = -0.004;
  wrench.torque.y = -0.005;
  wrench.torque.z = 0.006;

  float duration = 1.0;
  math::Vector3 gzForce(-0.01, -0.02, 0.03);
  math::Vector3 gzTorque(-0.004, -0.005, 0.006);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  ASSERT_EQ(hxs_wrench("wood_cube_5cm", "link", &wrench, duration), hxOK);
  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; i++)
  {
    EXPECT_NEAR(link->GetRelativeForce().x, gzForce.x, 5e-3);
    EXPECT_NEAR(link->GetRelativeForce().y, gzForce.y, 5e-3);
    EXPECT_NEAR(link->GetRelativeForce().z, gzForce.z, 5e-3);
    EXPECT_EQ(link->GetRelativeTorque(), gzTorque);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; i++)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    EXPECT_EQ(link->GetWorldTorque(), empty);
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
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
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

  world->Step(1);

  ASSERT_EQ(hxs_reset(0), hxOK);
  world->Step(1);
  for (auto model : world->GetModels())
  {
    math::Pose modelPose = model->GetWorldPose();
    EXPECT_NEAR(modelPose.pos.x, initialPoses[model->GetName()].pos.x, 5e-2);
    EXPECT_NEAR(modelPose.pos.y, initialPoses[model->GetName()].pos.y, 5e-2);
    EXPECT_NEAR(modelPose.pos.z, initialPoses[model->GetName()].pos.z, 5e-2);

    EXPECT_NEAR(modelPose.rot.w, initialPoses[model->GetName()].rot.w, 5e-2);
    EXPECT_NEAR(modelPose.rot.x, initialPoses[model->GetName()].rot.x, 5e-2);
    EXPECT_NEAR(modelPose.rot.y, initialPoses[model->GetName()].rot.y, 5e-2);
    EXPECT_NEAR(modelPose.rot.z, initialPoses[model->GetName()].rot.z, 5e-2);
  }

  // Move everything again
  for (auto model : world->GetModels())
  {
    math::Pose targetPose = initialPoses[model->GetName()];
    targetPose.pos += math::Vector3(1, 2, 3);
    model->SetWorldPose(targetPose);
  }
  world->Step(1);

  ASSERT_EQ(hxs_reset(1), hxOK);
  world->Step(1);
  // Expect that everything is in its initial state
  for (auto model : world->GetModels())
  {
    if (model->GetName() != "mpl_haptix_right_forearm")
    {
      math::Pose modelPose = model->GetWorldPose();
      EXPECT_NEAR(modelPose.pos.x, initialPoses[model->GetName()].pos.x, 5e-2);
      EXPECT_NEAR(modelPose.pos.y, initialPoses[model->GetName()].pos.y, 5e-2);
      EXPECT_NEAR(modelPose.pos.z, initialPoses[model->GetName()].pos.z, 5e-2);

      EXPECT_NEAR(modelPose.rot.w, initialPoses[model->GetName()].rot.w, 5e-2);
      EXPECT_NEAR(modelPose.rot.x, initialPoses[model->GetName()].rot.x, 5e-2);
      EXPECT_NEAR(modelPose.rot.y, initialPoses[model->GetName()].rot.y, 5e-2);
      EXPECT_NEAR(modelPose.rot.z, initialPoses[model->GetName()].rot.z, 5e-2);
    }
    else
    {
      math::Pose targetPose = initialPoses[model->GetName()];
      targetPose.pos += math::Vector3(1, 2, 3);

      EXPECT_NEAR(model->GetWorldPose().pos.x, targetPose.pos.x, 5e-2);
      EXPECT_NEAR(model->GetWorldPose().pos.y, targetPose.pos.y, 5e-2);
      EXPECT_NEAR(model->GetWorldPose().pos.z, targetPose.pos.z, 5e-2);

      EXPECT_NEAR(model->GetWorldPose().rot.w, targetPose.rot.w, 5e-2);
      EXPECT_NEAR(model->GetWorldPose().rot.x, targetPose.rot.x, 5e-2);
      EXPECT_NEAR(model->GetWorldPose().rot.y, targetPose.rot.y, 5e-2);
      EXPECT_NEAR(model->GetWorldPose().rot.z, targetPose.rot.z, 5e-2);
    }
  }
}

TEST_F(SimApiTest, HxsModelGravity)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);
  int gravity_mode;
  ASSERT_EQ(hxs_model_gravity_mode("wood_cube_5cm", &gravity_mode), hxOK);

  EXPECT_EQ(gravity_mode, 1);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  model->SetGravityMode(0);

  ASSERT_EQ(hxs_model_gravity_mode("wood_cube_5cm", &gravity_mode), hxOK);

  EXPECT_EQ(gravity_mode, 0);
}

TEST_F(SimApiTest, HxsSetModelGravity)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  int gravity_mode = 0;
  ASSERT_EQ(hxs_set_model_gravity_mode("wood_cube_5cm", gravity_mode), hxOK);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  for (auto link : model->GetLinks())
  {
    EXPECT_FALSE(link->GetGravityMode());
  }
}

TEST_F(SimApiTest, HxsSetModelColor)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != NULL);

  // Spawn a camera facing the box
  SpawnCamera("test_camera_model", "test_camera",
      math::Vector3(0, 0, 0), math::Vector3(0, 0, 0));

  int sleep = 0;
  int maxSleep = 5;
  rendering::VisualPtr visual;
  while (!visual && sleep < maxSleep)
  {
    gzdbg << "Visual count: " << scene->GetVisualCount() << std::endl;
    visual = scene->GetVisual("cricket_ball::link");
    common::Time::MSleep(100);
    sleep++;
  }
  ASSERT_TRUE(visual != NULL);

  hxsColor blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.alpha = 1.0;
  ASSERT_EQ(hxs_set_model_color("cricket_ball", &blue), hxOK);

  // Wait a moment for visual message to publish
  common::Time::Sleep(2);

  EXPECT_FLOAT_EQ(blue.r, visual->GetAmbient().r);
  EXPECT_FLOAT_EQ(blue.g, visual->GetAmbient().g);
  EXPECT_FLOAT_EQ(blue.b, visual->GetAmbient().b);
  EXPECT_FLOAT_EQ(blue.alpha, visual->GetAmbient().a);

  EXPECT_FLOAT_EQ(blue.r, visual->GetDiffuse().r);
  EXPECT_FLOAT_EQ(blue.g, visual->GetDiffuse().g);
  EXPECT_FLOAT_EQ(blue.b, visual->GetDiffuse().b);
  EXPECT_FLOAT_EQ(blue.alpha, visual->GetDiffuse().a);

  hxsColor rep;
  ASSERT_EQ(hxs_model_color("cricket_ball", &rep), hxOK);

  EXPECT_FLOAT_EQ(blue.r, rep.r);
  EXPECT_FLOAT_EQ(blue.g, rep.g);
  EXPECT_FLOAT_EQ(blue.b, rep.b);
  EXPECT_FLOAT_EQ(blue.alpha, rep.alpha);
}

TEST_F(SimApiTest, HxsModelCollideMode)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);

  hxsCollideMode rep;
  ASSERT_EQ(hxs_model_collide_mode("wood_cube_5cm", &rep), hxOK);
  EXPECT_EQ(rep, hxsCOLLIDE);

  // Set an object to collide_without_contact using Gazebo
  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      collision->GetSurface()->collideWithoutContact = true;
    }
  }
  ASSERT_EQ(hxs_model_collide_mode("wood_cube_5cm", &rep), hxOK);
  EXPECT_EQ(rep, hxsDETECTIONONLY);

  // Set an object to no_collide using Gazebo
  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      collision->GetSurface()->collideWithoutContact = false;
      collision->GetSurface()->collideBitmask = 0x0;
    }
  }
  ASSERT_EQ(hxs_model_collide_mode("wood_cube_5cm", &rep), hxOK);
  EXPECT_EQ(rep, hxsNOCOLLIDE);
}

TEST_F(SimApiTest, HxsSetModelCollideMode)
{
  physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);

  // Set collide_without_contact
  hxsCollideMode req = hxsDETECTIONONLY;
  ASSERT_EQ(hxs_set_model_collide_mode("wood_cube_5cm", &req), hxOK);
  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      EXPECT_TRUE(collision->GetSurface()->collideWithoutContact);
      EXPECT_EQ(collision->GetSurface()->collideBitmask, 0x01);
    }
  }

  // Set collide
  req = hxsCOLLIDE;
  ASSERT_EQ(hxs_set_model_collide_mode("wood_cube_5cm", &req), hxOK);
  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      EXPECT_FALSE(collision->GetSurface()->collideWithoutContact);
      EXPECT_EQ(collision->GetSurface()->collideBitmask, 0x01);
    }
  }

  // Set no_collide
  req = hxsNOCOLLIDE;
  ASSERT_EQ(hxs_set_model_collide_mode("wood_cube_5cm", &req), hxOK);
  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      EXPECT_FALSE(collision->GetSurface()->collideWithoutContact);
      EXPECT_EQ(collision->GetSurface()->collideBitmask, 0x0);
    }
  }

}

// TODO Implement stubbed out tests.
/*

// Logging tests will be implemented once logging is fully implemented.
TEST_F(SimApiTest, HxsStartLogging)
{
}


TEST_F(SimApiTest, HxsStopLogging)
{
}
*/

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
