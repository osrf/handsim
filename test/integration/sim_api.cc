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
#include <gazebo/math/Helpers.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <haptix/comm/haptix_sim.h>
#include "handsim/HaptixWorldPlugin.hh"
#include "test_config.h"

/// \brief This variable will have a random partition name for avoid collision
/// with other instances of the same test.
std::string partition;

///////////// Utility functions /////////////

/////////////////////////////////////////////////
void ConvertVector(const gazebo::math::Vector3 &_in, hxsVector3 &_out)
{
  _out.x = _in.x;
  _out.y = _in.y;
  _out.z = _in.z;
}

/////////////////////////////////////////////////
void ConvertVector(const hxsVector3 &_in, gazebo::math::Vector3 &_out)
{
  _out.x = _in.x;
  _out.y = _in.y;
  _out.z = _in.z;
}

/////////////////////////////////////////////////
void ConvertTransform(const hxsTransform &_in, gazebo::math::Pose &_out)
{
  ConvertVector(_in.pos, _out.pos);

  _out.rot.w = _in.orient.w;
  _out.rot.x = _in.orient.x;
  _out.rot.y = _in.orient.y;
  _out.rot.z = _in.orient.z;
}

//////////////////////////////////////////////////
void ConvertTransform(const gazebo::math::Pose &_in, hxsTransform &_out)
{
  ConvertVector(_in.pos, _out.pos);

  _out.orient.w = _in.rot.w;
  _out.orient.x = _in.rot.x;
  _out.orient.y = _in.rot.y;
  _out.orient.z = _in.rot.z;
}

////////////////////////////////////////////////
void ConvertWrench(const gazebo::physics::JointWrench &_in, hxsWrench &_out)
{
  ConvertVector(_in.body2Force, _out.force);
  ConvertVector(_in.body2Torque, _out.torque);
}

/////////////////////////////////////////////////
void ConvertJoint(gazebo::physics::Joint &_in, hxsJoint &_out)
{
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  _out.pos = _in.GetAngle(0).Radian();
  _out.vel = _in.GetVelocity(0);
  ConvertWrench(_in.GetForceTorque(0), _out.wrench_reactive);
  _out.torque_motor = _in.GetForce(0);
}

/////////////////////////////////////////////////
void ConvertLink(const gazebo::physics::Link &_in, hxsLink &_out)
{
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  ConvertTransform(_in.GetWorldPose(), _out.transform);
  ConvertVector(_in.GetWorldLinearVel(), _out.lin_vel);
  ConvertVector(_in.GetWorldAngularVel(), _out.ang_vel);
  ConvertVector(_in.GetWorldLinearAccel(), _out.lin_acc);
  ConvertVector(_in.GetWorldAngularAccel(), _out.ang_acc);
}

/////////////////////////////////////////////////
void ConvertModel(const gazebo::physics::Model &_in, hxsModel &_out)
{
  strncpy(_out.name, _in.GetName().c_str(), _in.GetName().length());

  ConvertTransform(_in.GetWorldPose(), _out.transform);

  // Gravity mode is only false if all links have gravity_mode set to false
  bool gravity_mode = false;
  int i = 0;
  for (auto link : _in.GetLinks())
  {
    ConvertLink(*link, _out.links[i]);
    i++;

    // Check gravity_mode mode
    gravity_mode |= link->GetGravityMode();
  }
  _out.link_count = i;

  _out.gravity_mode = gravity_mode;

  i = 0;
  for (auto joint : _in.GetJoints())
  {
    ConvertJoint(*joint, _out.joints[i]);
    i++;
  }
  _out.joint_count = i;
}

///////////// Server Fixture /////////////

class SimApiTest : public gazebo::ServerFixture
{
  public: gazebo::physics::WorldPtr InitWorld(const std::string &_worldFile);
};

gazebo::physics::WorldPtr SimApiTest::InitWorld(const std::string &_worldFile)
{
  boost::filesystem::path path = HANDSIM_TEST_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path.string());
  Load(_worldFile, true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  world->Step(1);
  return world;
}

TEST_F(SimApiTest, HxsSimInfo)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != NULL);

  gazebo::math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  // Spawn a camera
  gazebo::rendering::UserCameraPtr camera =
      scene->CreateUserCamera("test_camera");
  camera->SetWorldPose(cameraPose);
  camera->Update();
  gazebo::gui::set_active_camera(camera);
  ASSERT_TRUE(gazebo::gui::get_active_camera() != NULL);

  world->Step(1);
  sleep(0.5);
  hxsSimInfo simInfo;
  ASSERT_EQ(hxs_sim_info(&simInfo), hxOK);
  sleep(0.5);

  gazebo::math::Pose cameraOut;
  ConvertTransform(simInfo.camera_transform, cameraOut);

  // Verify object locations, camera pose, etc.
  EXPECT_EQ(cameraPose.pos, cameraOut.pos);

  EXPECT_FLOAT_EQ(cameraPose.rot.w, cameraOut.rot.w);
  EXPECT_FLOAT_EQ(cameraPose.rot.x, cameraOut.rot.x);
  EXPECT_FLOAT_EQ(cameraPose.rot.y, cameraOut.rot.y);
  EXPECT_FLOAT_EQ(cameraPose.rot.z, cameraOut.rot.z);

  for (int i = 0; i < simInfo.model_count; ++i)
  {
    gazebo::physics::ModelPtr gzModel = world->GetModel(simInfo.models[i].name);
    ASSERT_TRUE(gzModel != NULL);

    gazebo::math::Pose modelPose;
    ConvertTransform(simInfo.models[i].transform, modelPose);
    EXPECT_EQ(modelPose, gzModel->GetWorldPose());

    for (int j = 0; j < simInfo.models[i].link_count; ++j)
    {
      gazebo::physics::LinkPtr gzLink =
          gzModel->GetLink(simInfo.models[i].links[j].name);
      ASSERT_TRUE(gzLink != NULL);

      gazebo::math::Pose linkPose;
      ConvertTransform(simInfo.models[i].links[j].transform, linkPose);
      EXPECT_EQ(linkPose, gzLink->GetWorldPose());

      gazebo::math::Vector3 tmp;
      ConvertVector(simInfo.models[i].links[j].lin_vel, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldLinearVel());

      ConvertVector(simInfo.models[i].links[j].ang_vel, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldAngularVel());

      ConvertVector(simInfo.models[i].links[j].lin_acc, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldLinearAccel());

      ConvertVector(simInfo.models[i].links[j].ang_acc, tmp);
      EXPECT_EQ(tmp, gzLink->GetWorldAngularAccel());
    }

    for (int j = 0; j < simInfo.models[i].joint_count; ++j)
    {
      gazebo::physics::JointPtr gzJoint =
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
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  if (!scene)
  {
    scene = gazebo::rendering::create_scene("default", false);
    scene->Load();
    scene->Init();
  }

  ASSERT_TRUE(scene != NULL);

  gazebo::math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  // Spawn a camera
  gazebo::rendering::UserCameraPtr camera =
      scene->CreateUserCamera("test_camera");
  camera->SetWorldPose(cameraPose);
  camera->Update();
  gazebo::gui::set_active_camera(camera);
  ASSERT_TRUE(gazebo::gui::get_active_camera() != NULL);
  gazebo::common::Time::Sleep(1);

  hxsTransform transform;
  ASSERT_EQ(hxs_camera_transform(&transform), hxOK);
  gazebo::common::Time::Sleep(2);

  gazebo::math::Pose cameraOut;
  ConvertTransform(transform, cameraOut);

  // Verify camera pose
  EXPECT_EQ(cameraPose.pos, cameraOut.pos);

  EXPECT_FLOAT_EQ(cameraPose.rot.w, cameraOut.rot.w);
  EXPECT_FLOAT_EQ(cameraPose.rot.x, cameraOut.rot.x);
  EXPECT_FLOAT_EQ(cameraPose.rot.y, cameraOut.rot.y);
  EXPECT_FLOAT_EQ(cameraPose.rot.z, cameraOut.rot.z);
}

TEST_F(SimApiTest, HxsSetCameraTransform)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
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
  gazebo::rendering::UserCameraPtr camera =
      scene->CreateUserCamera("test_camera");
  gazebo::gui::set_active_camera(camera);

  hxsTransform transform;
  transform.pos.x = 1;
  transform.pos.y = 2;
  transform.pos.z = 3;

  gazebo::math::Quaternion q(3.14159, 1.570796, 0);

  transform.orient.w = q.w;
  transform.orient.x = q.x;
  transform.orient.y = q.y;
  transform.orient.z = q.z;

  ASSERT_EQ(hxs_set_camera_transform(&transform), hxOK);
  gazebo::common::Time::Sleep(2);

  gazebo::math::Pose outputPose =
      gazebo::gui::get_active_camera()->GetWorldPose();
  EXPECT_EQ(outputPose.pos, gazebo::math::Vector3(1, 2, 3));

  EXPECT_FLOAT_EQ(outputPose.rot.w, q.w);
  EXPECT_FLOAT_EQ(outputPose.rot.x, q.x);
  EXPECT_FLOAT_EQ(outputPose.rot.y, q.y);
  EXPECT_FLOAT_EQ(outputPose.rot.z, q.z);
}

TEST_F(SimApiTest, HxsContacts)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  hxsContactPoints contactPoints;

  gazebo::physics::ContactManager *contactManager =
      world->GetPhysicsEngine()->GetContactManager();
  ASSERT_TRUE(contactManager != NULL);

  const std::string modelName = "wood_cube_10cm";
  gazebo::physics::ModelPtr model = world->GetModel(modelName);
  model->SetWorldPose(gazebo::math::Pose(0, 0.25, -1.33, M_PI, 0, 0));
  model->SetAutoDisable(false);

  // Wait a little while for the world to initialize
  world->Step(20);
  world->SetPaused(false);

  ASSERT_EQ(hxs_contacts(modelName.c_str(), &contactPoints), hxOK);
  world->SetPaused(true);
  EXPECT_GT(contactPoints.contact_count, 0);

  // Have to find contacts and sort them by distance to compare
  // since they don't have string keys

  int matched_contacts = 0;

  // Each contact manager contact should have a unique match in contacts
  for (unsigned int k = 0; k < contactManager->GetContactCount(); ++k)
  {
    auto contact = contactManager->GetContacts()[k];
    if (contact->collision1->GetModel()->GetName() == modelName ||
        contact->collision2->GetModel()->GetName() == modelName)
    {
      for (int i = 0; i < contact->count; ++i)
      {
        gazebo::math::Pose linkPose =
            contact->collision1->GetLink()->GetWorldPose();

        // Now find matching contact point as returned by hxs_contacts
        for (int j = 0; j < contactPoints.contact_count; ++j)
        {
          bool link1NameMatch = std::string(contactPoints.contacts[i].link1) ==
              contact->collision1->GetLink()->GetName();
          bool link2NameMatch = std::string(contactPoints.contacts[i].link2) ==
              contact->collision2->GetLink()->GetName();
          gazebo::math::Vector3 contactPos, contactNormal, contactForce,
              contactTorque;
          ConvertVector(contactPoints.contacts[i].point, contactPos);
          ConvertVector(contactPoints.contacts[i].normal, contactNormal);
          ConvertVector(contactPoints.contacts[i].wrench.force, contactForce);
          ConvertVector(contactPoints.contacts[i].wrench.torque, contactTorque);

          gazebo::math::Pose contactPosPose(contact->positions[j],
              gazebo::math::Quaternion());

          // Transform the pose into the link frame
          contactPosPose = contactPosPose + linkPose.GetInverse();
          gazebo::math::Vector3 expectedContactPos = contactPosPose.pos;

          // Convert contactPos to the link frame
          if (link1NameMatch && link2NameMatch &&
              contactPos == expectedContactPos)
          {
            EXPECT_NEAR(contactPoints.contacts[j].distance,
                  contact->depths[i], 1e-6);
            EXPECT_EQ(contactForce, contact->wrench[j].body2Force);
            EXPECT_EQ(contactTorque, contact->wrench[j].body2Torque);
            // Expect the normal to face in the Gazebo Z direction, which is
            // the negative Z direction in the link frame, since we rotated
            // the cube
            EXPECT_EQ(contactNormal, gazebo::math::Vector3(0, 0, -1));
            // Expect the position to be on a corner of the box
            double radius = sqrt(3*pow(0.05, 2));
            EXPECT_NEAR(contactPos.GetLength(), radius, 1e-3);
            ++matched_contacts;
            break;
          }
        }
      }
    }
  }

  EXPECT_EQ(matched_contacts, contactPoints.contact_count);
  EXPECT_EQ(matched_contacts, 4);
}

TEST_F(SimApiTest, HxsSetModelJointState)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::ModelPtr gzDoorModel = world->GetModel("door");
  ASSERT_TRUE(gzDoorModel != NULL);
  ASSERT_TRUE(gzDoorModel->GetJoint("hinge") != NULL);
  gzDoorModel->GetJoint("hinge")->SetDamping(0, 0);

  world->Step(5);

  float pos = -1.58;
  float vel = 0.01;

  // test hxs_model_joint_state before setting joint states
  hxsModel model;
  ASSERT_EQ(hxs_model_joint_state("door", &model), hxOK);
  EXPECT_EQ(model.joint_count, 3);
  for (int i = 0; i < model.joint_count; ++i)
  {
    EXPECT_NEAR(model.joints[i].pos, 0, 1e-4);
    EXPECT_NEAR(model.joints[i].vel, 0, 1e-4);
  }

  // test hxs_set_model_joint_state
  ASSERT_EQ(hxs_set_model_joint_state("door", "hinge", pos, vel), hxOK);
  world->Step(1);

  ASSERT_TRUE(gzDoorModel->GetJoint("hinge") != NULL);
  EXPECT_NEAR(pos, gzDoorModel->GetJoint("hinge")->GetAngle(0).Radian(), 1e-2);

  EXPECT_NEAR(vel, gzDoorModel->GetJoint("hinge")->GetVelocity(0), 1e-2);

  // test hxs_model_joint_state after setting joint states
  ASSERT_EQ(hxs_model_joint_state("door", &model), hxOK);
  ASSERT_EQ(model.joint_count, 3);
  EXPECT_NEAR(model.joints[0].pos, 0, 1e-4);
  EXPECT_NEAR(model.joints[0].vel, 0, 1e-4);
  EXPECT_NEAR(model.joints[1].pos, -1.58, 1e-5);
  EXPECT_NEAR(model.joints[1].vel, 0.01, 1e-2);
  EXPECT_NEAR(model.joints[2].pos, 0, 1e-4);
  EXPECT_NEAR(model.joints[2].vel, 0, 1e-4);
}

TEST_F(SimApiTest, HxsAddModel)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
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

  gazebo::math::Quaternion q(roll, pitch, yaw);

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

  world->Step(1000);
  EXPECT_TRUE(world->GetModel("new_model") != NULL);
}

TEST_F(SimApiTest, HxsTransform)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  gazebo::math::Pose pose(-0.01, -0.02, -0.03, 3.14, 1.57, 1.57);
  model->SetWorldPose(pose);

  hxsTransform transform;

  ASSERT_EQ(hxs_model_transform("wood_cube_5cm", &transform), hxOK);

  EXPECT_EQ(model->GetWorldPose(), pose);
}

TEST_F(SimApiTest, HxsSetTransform)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  gazebo::math::Pose pose(0.01, 0.02, 3, 3.14, 1.57, 1.57);
  hxsTransform transform;
  transform.pos.x = pose.pos.x;
  transform.pos.y = pose.pos.y;
  transform.pos.z = pose.pos.z;
  transform.orient.w = pose.rot.w;
  transform.orient.x = pose.rot.x;
  transform.orient.y = pose.rot.y;
  transform.orient.z = pose.rot.z;

  ASSERT_EQ(hxs_set_model_transform("wood_cube_5cm", &transform), hxOK);
  world->Step(1);
  EXPECT_NEAR(model->GetWorldPose().pos.x, pose.pos.x, 1e-3);
  EXPECT_NEAR(model->GetWorldPose().pos.y, pose.pos.y, 1e-3);
  EXPECT_NEAR(model->GetWorldPose().pos.z, pose.pos.z, 1e-3);
  EXPECT_NEAR(model->GetWorldPose().rot.w, pose.rot.w, 1e-3);
  EXPECT_NEAR(model->GetWorldPose().rot.x, pose.rot.x, 1e-3);
  EXPECT_NEAR(model->GetWorldPose().rot.y, pose.rot.y, 1e-3);
  EXPECT_NEAR(model->GetWorldPose().rot.z, pose.rot.z, 1e-3);
}

TEST_F(SimApiTest, HxsLinearVel)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);
  gazebo::math::Vector3 gzLinVel(-0.01, -0.02, -0.03);
  model->SetLinearVel(gzLinVel);

  hxsVector3 lin_vel;

  ASSERT_EQ(hxs_linear_velocity("wood_cube_5cm", &lin_vel), hxOK);

  EXPECT_NEAR(model->GetWorldLinearVel().x, gzLinVel.x, 1e-3);
  EXPECT_NEAR(model->GetWorldLinearVel().y, gzLinVel.y, 1e-3);
  EXPECT_NEAR(model->GetWorldLinearVel().z, gzLinVel.z, 1e-3);
}

TEST_F(SimApiTest, HxsSetLinearVel)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 lin_vel;
  lin_vel.x = -0.01;
  lin_vel.y = -0.02;
  lin_vel.z = -0.03;

  ASSERT_EQ(hxs_set_linear_velocity("wood_cube_5cm", &lin_vel), hxOK);
  world->Step(1);

  EXPECT_NEAR(model->GetWorldLinearVel().x, -0.01, 1e-3);
  EXPECT_NEAR(model->GetWorldLinearVel().y, -0.02, 1e-3);
  EXPECT_NEAR(model->GetWorldLinearVel().z, -0.03, 1e-3);
}

TEST_F(SimApiTest, HxsAngularVel)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  gazebo::math::Vector3 gzAngVel(-0.01, -0.02, -0.03);
  model->SetAngularVel(gzAngVel);

  hxsVector3 ang_vel;

  ASSERT_EQ(hxs_angular_velocity("wood_cube_5cm", &ang_vel), hxOK);

  EXPECT_EQ(model->GetWorldAngularVel(), gzAngVel);
}

TEST_F(SimApiTest, HxsSetAngularVel)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 ang_vel;
  ang_vel.x = -0.01;
  ang_vel.y = -0.02;
  ang_vel.z = -0.03;

  ASSERT_EQ(hxs_set_angular_velocity("wood_cube_5cm", &ang_vel), hxOK);
  world->Step(1);

  EXPECT_FLOAT_EQ(model->GetWorldAngularVel().x, -0.01);
  EXPECT_FLOAT_EQ(model->GetWorldAngularVel().y, -0.02);
  EXPECT_FLOAT_EQ(model->GetWorldAngularVel().z, -0.03);
}

TEST_F(SimApiTest, HxsForce)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity_mode makes it easier to verify test
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 force;
  force.x = -0.01;
  force.y = -0.02;
  force.z = 0.03;

  float duration = 1.0;
  gazebo::math::Vector3 gzForce(-0.01, -0.02, 0.03);

  gazebo::physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  ASSERT_EQ(hxs_apply_force("wood_cube_5cm", "link", &force, duration), hxOK);
  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), gzForce);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  gazebo::math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    world->Step(100);
  }
}

TEST_F(SimApiTest, MultipleForces)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity_mode makes it easier to verify test
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 force;
  force.x = -0.01;
  force.y = -0.02;
  force.z = 0.03;

  float duration = 1.0;
  gazebo::math::Vector3 gzForce(-0.01, -0.02, 0.03);

  gazebo::physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  for (int i = 0; i < 3; ++i)
  {
    ASSERT_EQ(hxs_apply_force("wood_cube_5cm", "link", &force, duration), hxOK);
  }

  for (int i = 0; i < 3; ++i)
  {
    ASSERT_EQ(hxs_apply_force("wood_cube_5cm", "link", &force, 2*duration),
        hxOK);
  }

  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), 6*gzForce);
    world->Step(100);
  }
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), 3*gzForce);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  gazebo::math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    world->Step(100);
  }
}

TEST_F(SimApiTest, HxsTorque)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity_mode makes it easier to verify test
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);
  model->SetGravityMode(0);

  hxsVector3 torque;
  torque.x = -0.01;
  torque.y = -0.02;
  torque.z = -0.03;

  float duration = 1.0;
  gazebo::math::Vector3 gzTorque(-0.01, -0.02, -0.03);

  gazebo::physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  ASSERT_EQ(hxs_apply_torque("wood_cube_5cm", "link", &torque, duration), hxOK);

  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldTorque(), gzTorque);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  gazebo::math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    world->Step(100);
  }
}

TEST_F(SimApiTest, HxsWrench)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // disabling gravity_mode makes it easier to verify test
  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
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
  gazebo::math::Vector3 gzForce(-0.01, -0.02, 0.03);
  gazebo::math::Vector3 gzTorque(-0.004, -0.005, 0.006);

  gazebo::physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  ASSERT_EQ(hxs_apply_wrench("wood_cube_5cm", "link", &wrench, duration), hxOK);
  world->Step(1);

  // Every 0.1 seconds
  gzdbg << "Start time: " << world->GetSimTime() << std::endl;
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_NEAR(link->GetRelativeForce().x, gzForce.x, 5e-3);
    EXPECT_NEAR(link->GetRelativeForce().y, gzForce.y, 5e-3);
    EXPECT_NEAR(link->GetRelativeForce().z, gzForce.z, 5e-3);
    EXPECT_EQ(link->GetRelativeTorque(), gzTorque);
    world->Step(100);
  }
  gzdbg << "End time: " << world->GetSimTime() << std::endl;

  gazebo::math::Vector3 empty(0, 0, 0);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(link->GetWorldForce(), empty);
    EXPECT_EQ(link->GetWorldTorque(), empty);
    world->Step(100);
  }
}

TEST_F(SimApiTest, HxsRemoveModel)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  ASSERT_EQ(hxs_remove_model("wood_cube_5cm"), hxOK);
  world->Step(20);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  EXPECT_FALSE(model);
}

TEST_F(SimApiTest, HxsReset)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != NULL);
  gazebo::math::Pose cameraPose(1, 2, 3, 3.14159, 0.707, -0.707);
  // Spawn a camera
  gazebo::rendering::UserCameraPtr camera =
      scene->CreateUserCamera("test_camera");
  camera->SetWorldPose(cameraPose);
  camera->Update();
  gazebo::gui::set_active_camera(camera);
  ASSERT_TRUE(gazebo::gui::get_active_camera() != NULL);
  gazebo::common::Time::Sleep(1);
  camera->SetWorldPose(gazebo::math::Pose::Zero);

  // Wait a little while for the world to initialize
  world->Step(20);
  // Get everything's initial pose
  std::map<std::string, gazebo::math::Pose> initialPoses;
  for (auto model : world->GetModels())
  {
    initialPoses[model->GetName()] = model->GetWorldPose();
  }
  // Move everything
  for (auto model : world->GetModels())
  {
    gazebo::math::Pose targetPose = initialPoses[model->GetName()];
    targetPose.pos += gazebo::math::Vector3(1, 2, 3);
    model->SetWorldPose(targetPose);
  }

  world->Step(1);

  ASSERT_EQ(hxs_reset(1), hxOK);
  world->Step(1);
  for (auto model : world->GetModels())
  {
    gazebo::math::Pose modelPose = model->GetWorldPose();
    EXPECT_NEAR(modelPose.pos.x, initialPoses[model->GetName()].pos.x, 5e-2);
    EXPECT_NEAR(modelPose.pos.y, initialPoses[model->GetName()].pos.y, 5e-2);
    EXPECT_NEAR(modelPose.pos.z, initialPoses[model->GetName()].pos.z, 5e-2);

    EXPECT_NEAR(modelPose.rot.w, initialPoses[model->GetName()].rot.w, 5e-2);
    EXPECT_NEAR(modelPose.rot.x, initialPoses[model->GetName()].rot.x, 5e-2);
    EXPECT_NEAR(modelPose.rot.y, initialPoses[model->GetName()].rot.y, 5e-2);
    EXPECT_NEAR(modelPose.rot.z, initialPoses[model->GetName()].rot.z, 5e-2);
  }
  // Expect that the camera pose was set back to its initial value
  gazebo::math::Pose currentCameraPose = camera->GetWorldPose();
  EXPECT_NEAR(currentCameraPose.pos.x, 1, 1e-6);
  EXPECT_NEAR(currentCameraPose.pos.y, 2, 1e-6);
  EXPECT_NEAR(currentCameraPose.pos.z, 3, 1e-6);
  EXPECT_NEAR(currentCameraPose.rot.GetAsEuler().x, 3.14159, 1e-6);
  EXPECT_NEAR(currentCameraPose.rot.GetAsEuler().y, 0.707, 1e-6);
  EXPECT_NEAR(currentCameraPose.rot.GetAsEuler().z, -0.707, 1e-6);

  // Move everything again
  for (auto model : world->GetModels())
  {
    gazebo::math::Pose targetPose = initialPoses[model->GetName()];
    targetPose.pos += gazebo::math::Vector3(1, 2, 3);
    model->SetWorldPose(targetPose);
  }
  world->Step(1);

  ASSERT_EQ(hxs_reset(0), hxOK);
  world->Step(1);
  // Expect that everything is in its initial state, except for the arm
  for (auto model : world->GetModels())
  {
    if (model->GetName() != "mpl_haptix_right_forearm")
    {
      gazebo::math::Pose modelPose = model->GetWorldPose();
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
      gazebo::math::Pose targetPose = initialPoses[model->GetName()];
      targetPose.pos += gazebo::math::Vector3(1, 2, 3);

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
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);
  int gravity_mode;
  ASSERT_EQ(hxs_model_gravity_mode("wood_cube_5cm", &gravity_mode), hxOK);

  EXPECT_EQ(gravity_mode, 1);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  model->SetGravityMode(0);

  ASSERT_EQ(hxs_model_gravity_mode("wood_cube_5cm", &gravity_mode), hxOK);

  EXPECT_EQ(gravity_mode, 0);
}

TEST_F(SimApiTest, HxsSetModelGravity)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  int gravity_mode = 0;
  ASSERT_EQ(hxs_set_model_gravity_mode("wood_cube_5cm", gravity_mode), hxOK);
  world->Step(1);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  for (auto link : model->GetLinks())
  {
    EXPECT_FALSE(link->GetGravityMode());
  }
}

TEST_F(SimApiTest, HxsSetModelColor)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != NULL);

  // Spawn a camera facing the box
  SpawnCamera("test_camera_model", "test_camera",
      gazebo::math::Vector3(0, 0, 0), gazebo::math::Vector3(0, 0, 0));

  int sleep = 0;
  int maxSleep = 5;
  gazebo::rendering::VisualPtr visual;
  while (!visual && sleep < maxSleep)
  {
    visual = scene->GetVisual("cricket_ball::link");
    gazebo::common::Time::MSleep(100);
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
  world->Step(2000);

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
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
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
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::ModelPtr model = world->GetModel("wood_cube_5cm");
  ASSERT_TRUE(model != NULL);

  // Set collide_without_contact
  hxsCollideMode req = hxsDETECTIONONLY;
  ASSERT_EQ(hxs_set_model_collide_mode("wood_cube_5cm", &req), hxOK);
  world->Step(1);

  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      EXPECT_TRUE(collision->GetSurface()->collideWithoutContact);
      EXPECT_GT(collision->GetSurface()->collideBitmask, 0x0u);
    }
  }

  // Set collide
  req = hxsCOLLIDE;
  ASSERT_EQ(hxs_set_model_collide_mode("wood_cube_5cm", &req), hxOK);
  world->Step(1);

  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      EXPECT_FALSE(collision->GetSurface()->collideWithoutContact);
      EXPECT_EQ(collision->GetSurface()->collideBitmask, 0x01u);
    }
  }

  // Set no_collide
  req = hxsNOCOLLIDE;
  ASSERT_EQ(hxs_set_model_collide_mode("wood_cube_5cm", &req), hxOK);
  world->Step(1);

  for (auto link : model->GetLinks())
  {
    for (auto collision : link->GetCollisions())
    {
      EXPECT_FALSE(collision->GetSurface()->collideWithoutContact);
      EXPECT_EQ(collision->GetSurface()->collideBitmask, 0x0u);
    }
  }
}

TEST_F(SimApiTest, HxsAddRemoveConstraint)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  // Wait a little while for the world to initialize
  world->Step(20);

  // test add, remove constraint
  std::string constraintSDF =
      "<sdf version=\"1.5\">"
      "  <joint name=\"test_constraint\" type=\"revolute\">"
      "    <parent>table::link</parent>"
      "    <child>wood_cube_5cm::link</child>"
      "    <axis>"
      "      <xyz>0 1 0</xyz>"
      "    </axis>"
      "  </joint>"
      "</sdf>";

  EXPECT_EQ(hxs_add_constraint(constraintSDF.c_str(), "wood_cube_5cm"), hxOK);
  std::cout << "hxs_add_constraint executed." << std::endl;
  world->Step(1000);
  EXPECT_TRUE(world->GetModel("wood_cube_5cm")->GetJoint("test_constraint")
              != NULL);
  EXPECT_EQ(hxs_remove_constraint("test_constraint", "wood_cube_5cm"), hxOK);
  std::cout << "hxs_remove_constraint executed." << std::endl;
  world->Step(1000);
  EXPECT_TRUE(world->GetModel("wood_cube_5cm")->GetJoint("test_constraint")
              == NULL);
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
  // Get a random partition name.
  partition = testing::getRandomNumber();

  // Set the partition name for this process.
  setenv("IGN_PARTITION", partition.c_str(), 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
