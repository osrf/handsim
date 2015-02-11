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

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include "handsim/handsim/OptitrackBridge.hh"

using namespace haptix;
using namespace tracking;

static bool terminate = false;

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that processes frames
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    terminate = true;
}

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Usage: optitrackBridge <configFile>" << std::endl
            << std::endl
            << "\t<configFile>   Motive configuration file."
            << std::endl;
}

//////////////////////////////////////////////////
void test()
{
   RigidBody_M trackingInfo;
   OptitrackBridgeComms comms;

   // Try to send an empty map.
   if (comms.Send(trackingInfo))
    std::cerr << "FAIL: Send an empty map test failed" << std::endl;

  std::string head = "head";
  std::string monitor = "monitor";
  std::string hand = "hand";
  std::array<float, 7> headPose = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  std::array<float, 7> monitorPose = {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0};
  std::array<float, 7> handPose = {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0};
  RigidBody_M m;
  m[head] = headPose;
  m[monitor] = monitorPose;
  m[hand] = handPose;

  // Send some data.
  comms.Send(m);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  test();
  return 0;

  // Sanity check: Verify that we have the expected command line args.
  if (argc != 2)
  {
    usage();
    return -1;
  }

  try
  {
    OptitrackBridgeComms comms;
    std::string confFile = std::string(argv[1]);
    OptitrackBridge optitrack(confFile);
    RigidBody_M trackingInfo;

    // Get the tracking information from Optitrack and send it over the network.
    while (!terminate)
    {
      if (optitrack.Update(trackingInfo))
      {
        if (!comms.Send(trackingInfo))
          std::cerr << "OptitrackBridge: Error sending a frame." << std::endl;
      }
      else
        std::cerr << "OptitrackBridge: Error processing a frame." << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  catch (const std::runtime_error &_excep)
  {
    return -1;
  }

  return 0;
}