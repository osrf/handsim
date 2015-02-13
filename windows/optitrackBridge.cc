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
#include "NPTrackingTools.h"

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
  std::cerr << "Usage: optitrackBridge <configFile>" << std::endl
            << std::endl
            << "\t<configFile>   Motive configuration file."
            << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{

  // Sanity check: Verify that we have the expected command line args.
  if (argc != 2)
  {
    usage();
    return -1;
  }

  if (TT_Initialize() != NPRESULT_SUCCESS)
  {
    std::cerr << "TT_Initialize() error" << std::endl;
    return -1;
  }

  std::string confFile = std::string(argv[1]);
  if (TT_LoadProject(confFile.c_str()))
  {
    std::cerr << "TT_LoadProject() error" << std::endl;
    return -1;
  }

  // Get the tracking information from Optitrack and send it over the network.
  while (!hxTerminate)
  {
    TT_Update();

    // We're running at 500Hz. According to Natural Point we should call
    // TT_Update() periodically every 1ms to 10ms.
    // https://www.naturalpoint.com/optitrack/static/documents/Tracking%20Tools%202.0%20User%20Guide.pdf
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  return 0;
}
