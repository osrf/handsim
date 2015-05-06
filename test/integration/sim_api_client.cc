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
#include "haptix/comm/haptix_sim.h"
#include "gtest/gtest.h"
#include <gazebo/transport/Node.hh>

#include "test_config.h"

TEST(SimApiClientTest, TwoProcesses)
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
      char *args[] = {(char *) "--verbose"};
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
      std::cout << "hxs_sim_info successful." << std::endl;

      EXPECT_EQ(hxs_camera_transform(&cameraTransform), hxOK);
      std::cout << "hxs_camera_transform successful." << std::endl;

      EXPECT_EQ(hxs_set_camera_transform(&cameraTransform), hxOK);
      std::cout << "hxs_set_camera_transform successful." << std::endl;
      sleep(5);

      EXPECT_EQ(hxs_contacts("cricket_ball", &contactPoints), hxOK);
      std::cout << "hxs_contacts successful." << std::endl;

      EXPECT_EQ(hxs_set_model_joint_state("wooden_case", "lid_hinge", -1.58,
          0.01), hxOK);
      std::cout << "hxs_set_model_joint_state successful." << std::endl;

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

      EXPECT_EQ(hxs_set_model_link_state("cricket_ball", "link", &transform,
          &lin_vel, &ang_vel), hxOK);
      std::cout << "hxs_set_model_link_state successful." << std::endl;

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

      EXPECT_EQ(hxs_add_model(modelSDF.c_str(), "new_model", 0, 0.1, 0.2, 3.14,
        1.57, -1.57, false, &model), hxOK);
      std::cout << "hxs_add_model successful." << std::endl;

      EXPECT_EQ(hxs_remove_model("new_model"), hxOK);
      std::cout << "hxs_remove_model successful." << std::endl;

      EXPECT_EQ(hxs_model_gravity_mode("cricket_ball", &gravity_mode), hxOK);
      std::cout << "hxs_model_gravity_mode successful." << std::endl;
      gravity_mode = 0;
      EXPECT_EQ(hxs_set_model_gravity_mode("cricket_ball", gravity_mode), hxOK);
      std::cout << "hxs_set_model_gravity_mode successful." << std::endl;

      lin_vel.x = 0;
      lin_vel.y = 0;
      lin_vel.z = 0;
      EXPECT_EQ(hxs_set_linear_velocity("cricket_ball", &lin_vel), hxOK);
      std::cout << "hxs_set_linear_velocity successful." << std::endl;
      EXPECT_EQ(hxs_linear_velocity("cricket_ball", &lin_vel), hxOK);
      std::cout << "hxs_linear_velocity successful." << std::endl;
      ang_vel.x = 0;
      ang_vel.y = 0;
      ang_vel.z = 0;
      EXPECT_EQ(hxs_set_angular_velocity("cricket_ball", &ang_vel), hxOK);
      std::cout << "hxs_set_angular_velocity successful." << std::endl;
      EXPECT_EQ(hxs_angular_velocity("cricket_ball", &ang_vel), hxOK);
      std::cout << "hxs_angular_velocity successful." << std::endl;
      force.x = 10;
      force.y = 12;
      force.z = 13;
      EXPECT_EQ(hxs_apply_force("cricket_ball", "link", &force, 1.0), hxOK);
      std::cout << "hxs_apply_force successful." << std::endl;

      torque.x = 14;
      torque.y = 15;
      torque.z = 16;
      EXPECT_EQ(hxs_apply_torque("cricket_ball", "link", &torque, 1.0), hxOK);
      std::cout << "hxs_apply_torque successful." << std::endl;
      wrench.force = force;
      wrench.torque = torque;
      EXPECT_EQ(hxs_apply_wrench("cricket_ball", "link", &wrench, 1.0), hxOK);
      std::cout << "hxs_apply_wrench successful." << std::endl;

      EXPECT_EQ(hxs_model_transform("cricket_ball", &transform), hxOK);
      std::cout << "hxs_model_transform successful." << std::endl;
      EXPECT_EQ(hxs_set_model_transform("cricket_ball", &transform), hxOK);
      std::cout << "hxs_set_model_transform successful." << std::endl;

      EXPECT_EQ(hxs_reset(0), hxOK);
      // sleep to check manually that it did the right thing
      sleep(5);

      EXPECT_EQ(hxs_reset(1), hxOK);
      std::cout << "hxs_reset successful." << std::endl;

      EXPECT_EQ(hxs_start_logging("logfile"), hxOK);
      std::cout << "hxs_start_logging successful." << std::endl;
      // TODO delete the logfile?

      int is_logging;
      EXPECT_EQ(hxs_is_logging(&is_logging), hxOK);
      std::cout << "hxs_is_logging successful." << std::endl;

      EXPECT_EQ(hxs_stop_logging(), hxOK);
      std::cout << "hxs_stop_logging successful." << std::endl;

      EXPECT_EQ(hxs_model_color("cricket_ball", &color), hxOK);
      std::cout << "hxs_model_color successful." << std::endl;

      EXPECT_EQ(hxs_set_model_color("cricket_ball", &color), hxOK);
      std::cout << "hxs_set_model_color successful." << std::endl;

      hxsCollideMode collide_mode;
      EXPECT_EQ(hxs_model_collide_mode("cricket_ball", &collide_mode), hxOK);
      std::cout << "hxs_model_collide_mode successful." << std::endl;
      collide_mode = hxsNOCOLLIDE;
      EXPECT_EQ(hxs_set_model_collide_mode("cricket_ball", &collide_mode),
          hxOK);
      std::cout << "hxs_set_model_collide_mode successful." << std::endl;

      kill(PID_child, SIGKILL);
    }
  }
}
