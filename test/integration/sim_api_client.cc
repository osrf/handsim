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

#include <sys/prctl.h>
#include <haptix/comm/haptix_sim.h>
#include <gazebo/transport/Node.hh>
#include "gtest/gtest.h"

#include "test_config.h"

/// \brief This variable will have a random partition name for avoid collision
/// with other instances of the same test.
std::string partition;

/////////////////////////////////////////////////
TEST(SimApiClientTest, ThreeProcesses)
{
  pid_t PID_child = fork();
  if (PID_child == 0)
  {
    // Try to kill this process when parent dies
    prctl(PR_SET_PDEATHSIG, SIGKILL);
    // Exec gzserver
    char *args[] = {const_cast<char *>("--verbose"),
        const_cast<char *>("worlds/arat.world")};
    std::cout << "Launching gzserver." << std::endl;
    execvp("gzserver", args);
  }
  else
  {
    // Fork again
    pid_t PID_grandchild = fork();
    if (PID_grandchild == 0)
    {
      // Try to kill this process when parent dies
      prctl(PR_SET_PDEATHSIG, SIGKILL);
      // Exec gzclient
      char *args[] = {const_cast<char *>("--verbose")};
      std::cout << "Launching gzclient." << std::endl;
      execvp("gzclient", args);
    }
    else
    {
      ASSERT_GT(PID_grandchild, 0);

      // TODO Better way to know when gzclient and gzserver are ready
      sleep(5);

      hxsSimInfo simInfo;
      hxsTransform cameraTransform;
      hxsContactPoints contactPoints;
      hxsTransform transform;
      hxsVector3 lin_vel;
      hxsVector3 ang_vel;
      hxsWrench wrench;
      hxsModel model;
      hxsVector3 force;
      hxsVector3 torque;
      hxsColor color;
      int gravity_mode;

      EXPECT_EQ(hxs_sim_info(&simInfo), hxOK);
      std::cout << "hxs_sim_info executed." << std::endl;

      EXPECT_EQ(hxs_camera_transform(&cameraTransform), hxOK);
      std::cout << "hxs_camera_transform executed." << std::endl;

      EXPECT_EQ(hxs_set_camera_transform(&cameraTransform), hxOK);
      std::cout << "hxs_set_camera_transform executed." << std::endl;

      EXPECT_EQ(hxs_contacts("cricket_ball", &contactPoints), hxOK);
      std::cout << "hxs_contacts executed." << std::endl;

      // test joint manipulation

      // turn off gravity on case before setting state
      EXPECT_EQ(hxs_set_model_gravity_mode(
        "mpl_haptix_right_forearm", 0), hxOK);
      sleep(1);

      // turn off controller for the arm
      hxRobotInfo robotInfo;
      EXPECT_EQ(hx_robot_info(&robotInfo), hxOK);
      hxSensor sensor;
      hxCommand command;
      command.gain_pos_enabled = 1;
      command.gain_vel_enabled = 1;
      for (int i = 0; i < robotInfo.motor_count; ++i)
      {
        command.gain_pos[i] = 0;
        command.gain_vel[i] = 0;
      }
      EXPECT_EQ(hx_update(&command, &sensor), hxOK);
      sleep(1);

      // set target position and velocity for a finger
      const double pos_target = -0.3;
      const double vel_target = 0.0;
      EXPECT_EQ(hxs_set_model_joint_state("mpl_haptix_right_forearm",
          "index0", pos_target, vel_target), hxOK);
      std::cout << "hxs_set_model_joint_state executed." << std::endl;

      /// test alternative to sleep(1)
      /// check time using in hxSensor to see if physics has updated
      EXPECT_EQ(hx_read_sensors(&sensor), hxOK);
      unsigned int nsec0 = sensor.time_stamp.nsec;
      unsigned int nsec = nsec0;
      int count = 0;
      // wait for at least 1ms sim time
      while (nsec - nsec0 < 1000000 && count < 100)
      {
        if (++count >= 100)
          std::cout << "WARN: wait for sim time to advance failed" << std::endl;
        EXPECT_EQ(hx_read_sensors(&sensor), hxOK);
        nsec = sensor.time_stamp.nsec;
        sleep(0.01);
      }

      // get model joint state and check if setting it worked.
      EXPECT_EQ(hxs_model_joint_state("mpl_haptix_right_forearm",
        &model), hxOK);
      std::cout << "hxs_model_joint_state executed." << std::endl;
      int jointIndex = -1;
      for (int i = 0; i < model.joint_count; ++i)
      {
        if (strcmp(model.joints[i].name, "index0") == 0)
        {
          jointIndex = i;
          break;
        }
      }
      ASSERT_GT(jointIndex, -1);

      EXPECT_NEAR(model.joints[jointIndex].pos, pos_target, 1e-5);
      EXPECT_NEAR(model.joints[jointIndex].vel, vel_target, 1e-5);

      transform.pos.x = 1;
      transform.pos.y = 2;
      transform.pos.z = 3;
      transform.orient.w = 1;
      transform.orient.x = 0;
      transform.orient.y = 0;
      transform.orient.z = 0;
      lin_vel.x = 1.0;
      lin_vel.y = 1.1;
      lin_vel.z = 1.2;
      ang_vel.x = 1.4;
      ang_vel.y = 1.5;
      ang_vel.z = 1.5;

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
            "</model>"
          "</sdf>";


      EXPECT_EQ(hxs_remove_model("wooden_case"), hxOK);
      std::cout << "hxs_remove_model executed." << std::endl;

      EXPECT_EQ(hxs_add_model(modelSDF.c_str(), "new_model", 0, 0.1, 1, 3.14,
        1.57, -1.57, false, &model), hxOK);
      std::cout << "hxs_add_model executed." << std::endl;

      EXPECT_EQ(hxs_model_gravity_mode("cricket_ball", &gravity_mode), hxOK);
      std::cout << "hxs_model_gravity_mode executed." << std::endl;
      gravity_mode = 0;
      EXPECT_EQ(hxs_set_model_gravity_mode("cricket_ball", gravity_mode), hxOK);
      std::cout << "hxs_set_model_gravity_mode executed." << std::endl;

      EXPECT_EQ(hxs_set_linear_velocity("cricket_ball", &lin_vel), hxOK);
      std::cout << "hxs_set_linear_velocity executed." << std::endl;
      EXPECT_EQ(hxs_linear_velocity("cricket_ball", &lin_vel), hxOK);
      std::cout << "hxs_linear_velocity executed." << std::endl;
      EXPECT_EQ(hxs_set_angular_velocity("cricket_ball", &ang_vel), hxOK);
      sleep(5);
      std::cout << "hxs_set_angular_velocity executed." << std::endl;
      EXPECT_EQ(hxs_angular_velocity("cricket_ball", &ang_vel), hxOK);
      std::cout << "hxs_angular_velocity executed." << std::endl;
      force.x = 10;
      force.y = 12;
      force.z = 13;
      EXPECT_EQ(hxs_apply_force("cricket_ball", "link", &force, 1.0), hxOK);
      std::cout << "hxs_apply_force executed." << std::endl;

      torque.x = 14;
      torque.y = 15;
      torque.z = 16;
      EXPECT_EQ(hxs_apply_torque("cricket_ball", "link", &torque, 1.0), hxOK);
      std::cout << "hxs_apply_torque executed." << std::endl;
      wrench.force = force;
      wrench.torque = torque;
      EXPECT_EQ(hxs_apply_wrench("cricket_ball", "link", &wrench, 1.0), hxOK);
      std::cout << "hxs_apply_wrench executed." << std::endl;

      EXPECT_EQ(hxs_model_transform("cricket_ball", &transform), hxOK);
      std::cout << "hxs_model_transform executed." << std::endl;
      EXPECT_EQ(hxs_set_model_transform("cricket_ball", &transform), hxOK);
      std::cout << "hxs_set_model_transform executed." << std::endl;

      EXPECT_EQ(hxs_reset(0), hxOK);
      // sleep to check manually that it did the right thing

      EXPECT_EQ(hxs_reset(1), hxOK);
      std::cout << "hxs_reset executed." << std::endl;

      EXPECT_EQ(hxs_start_logging("logfile"), hxOK);
      std::cout << "hxs_start_logging executed." << std::endl;
      // TODO delete the logfile?

      int is_logging;
      EXPECT_EQ(hxs_is_logging(&is_logging), hxOK);
      std::cout << "hxs_is_logging executed." << std::endl;

      EXPECT_EQ(hxs_stop_logging(), hxOK);
      std::cout << "hxs_stop_logging executed." << std::endl;

      EXPECT_EQ(hxs_model_color("cricket_ball", &color), hxOK);
      std::cout << "hxs_model_color executed." << std::endl;

      EXPECT_EQ(hxs_set_model_color("cricket_ball", &color), hxOK);
      std::cout << "hxs_set_model_color executed." << std::endl;

      hxsCollideMode collide_mode;
      EXPECT_EQ(hxs_model_collide_mode("cricket_ball", &collide_mode), hxOK);
      std::cout << "hxs_model_collide_mode executed." << std::endl;
      collide_mode = hxsNOCOLLIDE;
      EXPECT_EQ(hxs_set_model_collide_mode("cricket_ball", &collide_mode),
          hxOK);
      std::cout << "hxs_set_model_collide_mode executed." << std::endl;

      kill(PID_child, SIGKILL);
    }
  }
}

int main(int argc, char **argv)
{
  // Get a random partition name.
  partition = testing::getRandomNumber();

  // Set the partition name for this process.
  setenv("IGN_PARTITION", partition.c_str(), 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
