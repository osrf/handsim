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
#include <gazebo/sensors/sensors.hh>
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
  public: double GetDepth(const gazebo::msgs::Contacts &_contacts,
    const gazebo::physics::LinkPtr _link);
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

double PhysicsTest::GetDepth(const gazebo::msgs::Contacts &_contacts,
  const gazebo::physics::LinkPtr _link)
{
  // we must start at the end of the contacts.contact() array,
  // go backwards and aggregate all the wrenches that have the same
  // timestamp and same link name
  int numContacts = _contacts.contact().size();

  // initial timestamp with a negative value, initialize with the timestap of
  // last one in the buffer array (contacts.contact()), then
  // only aggregate the ones that have the same timestamp.
  double timestamp = -1;

  double depth = 0;
  bool depthInitialized = false;
  int depthCount = 0;

  while (numContacts > 0)
  {
    --numContacts;
    gazebo::msgs::Contact contact = _contacts.contact(numContacts);

    // if (contact.wrench().size() > 0)
    //   gzerr << "  name " << contactSensor->GetName() << "\n";

    // each contact can have multiple wrenches
    for (int k = 0; k < contact.wrench().size(); ++k)
    {
      gazebo::msgs::JointWrench wrenchMsg = contact.wrench(k);

      double t = contact.time().sec() + 1.0e-9*contact.time().nsec();
      if (timestamp < 0)
      {
        // timestamp uninitialized
        timestamp = t;
      }

      if (!gazebo::math::equal(t, timestamp))
      {
        // got an older timestamp, stop the while loop
        numContacts = 0;
      }
      else
      {
        // gzerr << "    contacts [" << numContacts
        //       << "] t ["  << timestamp
        //       << "] wrench size ["  << contact.wrench().size()
        //       << "] body 1 ["  << wrenchMsg.body_1_name()
        //       << "] body 2 ["  << wrenchMsg.body_2_name()
        //       << "]\n";

        // sum up all wrenches from body_1 or body_2
        // check with contact corresponds to the arm and which to the arm
        // compare body_1_name and body_2_name with model name

        std::string name = _link->GetScopedName();
        gazebo::msgs::Wrench wrench;
        if (strncmp(name.c_str(), wrenchMsg.body_1_name().c_str(),
                    name.size()) == 0)
        {
          wrench = wrenchMsg.body_1_wrench();
        }
        else if (strncmp(name.c_str(), wrenchMsg.body_2_name().c_str(),
                         name.size()) == 0)
        {
          wrench = wrenchMsg.body_2_wrench();
        }
        else
        {
          gzerr << "collision name does not match model name. This should "
                 << "never happen." << std::endl;
        }

        ignition::math::Pose3<double> fPose = _link->GetWorldPose().Ign();
        ignition::math::Vector3d fPos = fPose.Pos();
        ignition::math::Quaternion<double> fRot = fPose.Rot();

        // force and torque in Link frame
        ignition::math::Vector3d force =
          gazebo::msgs::ConvertIgn(wrench.force());
        ignition::math::Vector3d torque =
          gazebo::msgs::ConvertIgn(wrench.torque());

        // force and torque in inertial frame at Link origin
        ignition::math::Vector3d forceI = fRot.RotateVector(force);
        ignition::math::Vector3d torqueI = fRot.RotateVector(torque);

        // position and normal in inertial frame
        ignition::math::Vector3d position =
          gazebo::msgs::ConvertIgn(contact.position(k));
        ignition::math::Vector3d normal =
          gazebo::msgs::ConvertIgn(contact.normal(k));

        // force moment arm in inertial frame
        ignition::math::Vector3d forceArm = position - fPos;

        // compute force at contact in inertial frame
        ignition::math::Vector3d forceAtContact =
          forceI + torqueI.Cross(forceArm);

        // minimum of two contacting collisions
        // in this case, wood_cube_5cm is 0.001 and hand is 0.0015
        const double minDepth = 0.001;

        double fn = -forceAtContact.Dot(normal);
        double d0 = contact.depth(k) - minDepth;
        double kp = fn / d0;

        gzdbg << "name [" << name
              << "f_b [" << force
              << "] t_b [" << torque
              << "] p [" << position - fPos
              << "] tr [" << torque.Cross(position - fPos)
              << "] f [" << forceAtContact
              << "] n [" << normal
              << "] fn [" << fn
              << "] d [" << d0
              << "] kp [" << kp
              << "]\n";

        if (!depthInitialized)
        {
          depth = d0;
          ++depthCount;
          depthInitialized = true;
        }
        else
        {
          depth += d0;
          ++depthCount;
        }
      }
    }
  }
  return depth / static_cast<double>(depthCount);
}

TEST_F(PhysicsTest, Test1)
{
  gazebo::physics::WorldPtr world = this->InitWorld("worlds/arat_test.world");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();

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
    EXPECT_TRUE(::hx_update(&lastMotorCommand, &lastSensor) == ::hxOK);
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

  gzerr << "moved"; getchar();

  // set gravity to zero
  physics->SetGravity(gazebo::math::Vector3());

  // get contact sensors for depth
  gazebo::sensors::SensorManager *mgr =
    gazebo::sensors::SensorManager::Instance();
  gazebo::sensors::SensorPtr indexSensor =
    mgr->GetSensor("index3_contact_sensor");
  gazebo::sensors::SensorPtr thumbSensor =
    mgr->GetSensor("thumb3_contact_sensor");
  gazebo::sensors::ContactSensorPtr indexContactSensor =
    boost::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(indexSensor);
  gazebo::sensors::ContactSensorPtr thumbContactSensor =
    boost::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(thumbSensor);

  // pull on the cube
  for (int n = 0; n < 10000; ++n)
  {
    // get contact depth from physics engine
    gazebo::msgs::Contacts indexContacts = indexContactSensor->GetContacts();
    gazebo::msgs::Contacts thumbContacts = thumbContactSensor->GetContacts();
    gazebo::physics::LinkPtr indexLink =
      world->GetModel("mpl_haptix_right_forearm")->GetLink("index3");
    gazebo::physics::LinkPtr thumbLink =
      world->GetModel("mpl_haptix_right_forearm")->GetLink("thumb3");
    double indexDepth = this->GetDepth(indexContacts, indexLink);
    double thumbDepth = this->GetDepth(thumbContacts, thumbLink);

    // check force on the contact sensors
    ::hxSensor sensor;
    EXPECT_TRUE(::hx_read_sensors(&sensor) == ::hxOK);
    double thumbForce = sensor.contact[6];
    double indexForce = sensor.contact[9];

    // compute stiffness based on force and depth
    double thumbKp = thumbForce / thumbDepth;
    double indexKp = indexForce / indexDepth;

    gazebo::physics::LinkPtr link = world->GetModel("wood_cube_5cm")->GetLink();
    double f = std::max(indexForce, thumbForce);
    link->AddForce(gazebo::math::Vector3(0, 0, -f));
    world->Step(1);
    gzdbg << "finger contact force [" << indexForce
          << "] [" << thumbForce
          << "] cube pose [" << link->GetWorldPose()
          << "] indexDepth [" << indexDepth
          << "] thumbDepth [" << thumbDepth
          << "] index k [" << indexKp
          << "] thumb k [" << thumbKp
          << "]\n";
    getchar();
  }
  gzerr << "done"; getchar();
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  gazebo::math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
