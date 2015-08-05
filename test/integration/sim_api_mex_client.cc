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

#include <errno.h>
#include <sys/prctl.h>
#include <haptix/comm/haptix_sim.h>
#include <gazebo/transport/Node.hh>
#include "gtest/gtest.h"

#include "test_config.h"

/// \brief This variable will have a random partition name for avoid collision
/// with other instances of the same test.
std::string partition;

/////////////////////////////////////////////////
TEST(SimApiMexClientTest, ThreeProcesses)
{
  pid_t PID_child = fork();
  if (PID_child == 0)
  {
    // Try to kill this process when parent dies
    prctl(PR_SET_PDEATHSIG, SIGKILL);
    // Exec gzserver
    char *args[] = {const_cast<char *>("--verbose"),
        const_cast<char *>("worlds/arat.world"), static_cast<char *>(NULL)};
    std::cout << "Launching gzserver." << std::endl;
    if (execvp("gzserver", args) == -1)
    {
      std::cerr << "gzserver failed with error code: " << errno;
      FAIL();
    }
  }
  else
  {
    ASSERT_GT(PID_child, 0);
    // Fork again
    pid_t PID_grandchild = fork();
    if (PID_grandchild == 0)
    {
      // Try to kill this process when parent dies
      prctl(PR_SET_PDEATHSIG, SIGKILL);
      // Exec gzclient
      char *args[] = {const_cast<char *>("--verbose"),
        static_cast<char *>(NULL)};
      std::cout << "Launching gzclient." << std::endl;
      if (execvp("gzclient", args) == -1)
      {
        std::cerr << "gzclient failed with error code: " << errno;
        FAIL();
      }
    }
    else
    {
      ASSERT_GT(PID_grandchild, 0);

      // TODO Better way to know when gzclient and gzserver are ready
      sleep(5);

      // Call out to Octave, run a script, and check the exit code.  The idea
      // is that, if the exit code is 0, then nothing in the script threw an
      // error.  Not an exhaustive check, and also it doesn't tell you which
      // line in the script failed, but it's better than no tests.
      int result = system("cd " CMAKE_INSTALL_PREFIX "/" CMAKE_INSTALL_LIBDIR
                          "/haptix-comm/octave && octave hxs_example.m");

      EXPECT_EQ(result, 0);

      kill(PID_child, SIGKILL);
      kill(PID_grandchild, SIGKILL);
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Get a random partition name.
  partition = testing::getRandomNumber();

  // Set the partition name for this process.
  setenv("IGN_PARTITION", partition.c_str(), 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
