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
#include "handsim/OptitrackBridge.hh"

using namespace haptix;
using namespace tracking;

static bool hxTerminate = false;

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that processes frames
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    hxTerminate = true;
}

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Usage: optitrackBridge <configFile> [-d]" << std::endl
            << std::endl
            << "\t<configFile>   Motive configuration file." << std::endl
            << "\t -d            Disable custom streaming."
            << std::endl;
}

//////////////////////////////////////////////////
void test()
{
  TrackingInfo_t trackingInfo;
  OptitrackBridgeComms comms;

  std::string head    = "head";
  std::string monitor = "monitor";
  std::string hand    = "hand";
  Pose_t headPose     = { 1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f};
  Marker_t headM1     = { 1.1f,  2.1f,  3.1f};
  Marker_t headM2     = { 4.1f,  5.1f,  6.1f};
  Marker_t headM3     = { 7.1f,  8.1f,  9.1f};
  Pose_t monitorPose  = {11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f};
  Marker_t monitorM1  = {11.1f, 12.1f, 13.1f};
  Marker_t monitorM2  = {14.1f, 15.1f, 16.1f};
  Marker_t monitorM3  = {17.1f, 18.1f, 19.1f};
  Pose_t handPose     = {21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f};
  Marker_t handM1     = {21.1f, 22.1f, 23.1f};
  Marker_t handM2     = {24.1f, 25.1f, 26.1f};
  Marker_t handM3     = {27.1f, 28.1f, 29.1f};
  double timestamp    = 0.5;
  trackingInfo.timestamp               = timestamp;
  trackingInfo.bodies[head].body       = headPose;
  trackingInfo.bodies[head].markers    = {headM1, headM2, headM3};
  trackingInfo.bodies[monitor].body    = monitorPose;
  trackingInfo.bodies[monitor].markers = {monitorM1, monitorM2, monitorM3};
  trackingInfo.bodies[hand].body       = handPose;
  trackingInfo.bodies[hand].markers    = {handM1, handM2, handM3};

  // Send some data.
  if (!comms.Send(trackingInfo))
    std::cerr << "FAIL: Send fake tracking info failed" << std::endl;

  std::cout << "Press any key to exit." << std::endl;
  getchar();
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
#ifdef __linux__
  // We keep this fragment for testing on Linux.
  test();
  return 0;
#endif
  // Sanity check: Verify that we have the expected command line args.
  if (argc < 2 || argc > 3)
  {
    usage();
    return -1;
  }

  bool streamingEnabled = true;

  // Sanity check: Verify that the optional third argument is "-d".
  if (argc == 3)
  {
    std::string arg = std::string(argv[2]);
    if (arg != "-d")
    {
      usage();
      return -1;
    }
    streamingEnabled = false;
  }

  try
  {
    OptitrackBridgeComms comms;
    std::string confFile = std::string(argv[1]);
    OptitrackBridge optitrack(confFile);
    TrackingInfo_t trackingInfo;

    // Get the tracking information from Optitrack and send it over the network.
    while (!hxTerminate)
    {
      if (optitrack.Update(trackingInfo))
      {
        if (streamingEnabled && !trackingInfo.bodies.empty())
        {
          if (!comms.Send(trackingInfo))
            std::cerr << "OptitrackBridge: Error sending a frame." << std::endl;
        }
      }

      // We're running at 500Hz. According to Natural Point we should call
      // TT_Update() periodically every 1ms to 10ms.
      // https://www.naturalpoint.com/optitrack/static/documents/Tracking%20Tools%202.0%20User%20Guide.pdf
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  catch (const std::runtime_error &/*_excep*/)
  {
    return -1;
  }

  return 0;
}