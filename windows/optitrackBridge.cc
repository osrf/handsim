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

#ifdef _WIN32
  // For socket(), connect(), send(), and recv().
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
#else
  // For data types
  #include <sys/types.h>
  // For socket(), connect(), send(), and recv()
  #include <sys/socket.h>
  // For gethostbyname()
  #include <netdb.h>
  // For inet_addr()
  #include <arpa/inet.h>
  // For close()
  #include <unistd.h>
  // For sockaddr_in
  #include <netinet/in.h>
#endif
#include <chrono>
#include <exception>
#include <iostream>
#include <vector>
//#include "NPTrackingTools.h"

static bool terminate = false;

/// \brief New exception.
class SocketException : public std::runtime_error
{
  public: SocketException() : std::runtime_error("SocketException") = default;
};

/// \brief Class that implements a simple multicast sender.
class Comms: public exception
{
  /// \brief Constructor.
  /// \throw SocketException If a socket error occurs during initialization.
  Comms()
  {
    WSADATA wsaData;

    // Request WinSock v2.0.
    WORD wVersionRequested = MAKEWORD(2, 0);
    // Load WinSock DLL.
    if (WSAStartup(wVersionRequested, &wsaData) != 0)
    {
     std::cerr << "Unable to load WinSock DLL" << std::endl;
     throw SocketException();
    }

    // Make a new socket for sending OptiTrack updates.
    if ((socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
      std::cerr << "Socket creation failed." << std::endl;
      throw SocketException();
    }

    // Set up the destination address.
    struct sockaddr_in mySocketAddr;
    memset(&mySocketAddr, 0, sizeof(mySocketAddr));
    mySocketAddr.sin_family = AF_INET;
    mySocketAddr.sin_port = htons(zPortData);
    mySocketAddr.sin_addr.s_addr = inet_addr(zMulticastAddress);
  }

  /// \brief Destructor.
  ~Comms()
  {
    close(socket);
  }

  /// \brief Send a new message over the network.
  public: bool Send(const std::vector<float> &_data)
  {
    // Wire protocol:
    // 1. Number of rigid bodies tracked (int16_t)
    // For each rigid body:
    //   2. x, y, z, roll, pitch, yaw
    std::cout << "Send" << std::endl;
  }

  /// \brief Optitrack multicast address.
  private: static const std::string zMulticastAddress = "239.255.42.99";

  /// \brief Port used for sending/receiving tracking updates.
  private: static const int zPortData = 1511;

  /// \brief UDP socket used for sending tracking updates.
  private: int socket;
};

/// \brief New exception.
class OptitrackException : public std::runtime_error
{
  public: OptitrackException()
    : std::runtime_error("OptiTrackException") = default;
};

/// \brief Class that implements a simple multicast sender.
class Optitrack
{
  /// \brief Initializes the OptiTrack device and loads its configuration.
  /// \param[in] _confFile Motive project file containing camera
  /// parameters and rigid body configuration.
  /// \throw OptitrackException If a socket error occurs during initialization.
  Optitrack(const std::string &_configFile)
  {
    /*if (TT_Initialize() != NPRESULT_SUCCESS))
    {
      std::cerr << "TT_Initialize() error" << std::endl;
      throw OptitrackException();
    }*/

    /*if (TT_LoadProject(_confFile))
    {
      std::cerr << "TT_LoadProject() error" << std::endl;
      throw OptitrackException();
    }*/
  }

  /// \brief Destructor.
  ~Optitrack()
  {
    /*TT_Shutdown();
    TT_FinalCleanup();*/
  }

  /// \brief Process a new camera frame and apply 3D reconstruction and rigid
  /// body tracking.
  /// \param[out] _data Return the vector of rigid bodies tracked.
  /// Each rigid body contains six floats (x, y, z, roll, pitch, yaw).
  public: bool Update(std::vector<std::array<float, 6>> &_data)
  {
    _data.clear();

    /*if (TT_Update() != NPRESULT_SUCCESS)
    {
      std::cerr << "TT_Update() error" << std::endl;
      return false;
    }*/

    std::cout << "Update()" << std::endl;
    return true;
  }
};

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
int main(int argc, char **argv)
{
  // Sanity check: Verify that we have the expected command line args.
  if (argc != 2)
  {
    usage();
    return -1;
  }

  try
  {
    Comms comms;
    Optitrack optitrack(std::string(argv[1]));
  }
  catch (const &std::runtime_error)
  {
    return -=1;
  }

  // \brief We will store in this variable all the tracking information.
  // Each element of the vector is a tracked rigid body. For each rigid body
  // we store 6 floats (x, y, z, roll, pitch, yaw).
  std::vector<std::array<float, 6>> trackingInfo;

  // Get the tracking information from Optitrack and send it over the network.
  while (!terminate)
  {
    if (optitrack.Update(trackinfInfo))
    {
      if (!comms.Send(trackingInfo))
        std::cerr << "Error sending a frame over the network." << std::endl;
    }
    else
      std::cerr << "Error processing a frame." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}