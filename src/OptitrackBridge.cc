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
  // Type used for raw data on this platform.
  typedef char raw_type;
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
  // Type used for raw data on this platform
  typedef void raw_type;
#endif
#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <map>
#include <vector>

//#include "NPTrackingTools.h"
#include "handsim/OptitrackBridge.hh"

using namespace haptix;
using namespace tracking;

/// \brief Optitrack multicast address.
static const std::string zMulticastAddress = "239.255.42.99";

/////////////////////////////////////////////////
OptitrackBridgeComms::OptitrackBridgeComms()
{
#ifdef _WIN32
  WSADATA wsaData;

  // Request WinSock v2.0.
  WORD wVersionRequested = MAKEWORD(2, 0);
  // Load WinSock DLL.
  if (WSAStartup(wVersionRequested, &wsaData) != 0)
  {
    std::cerr << "OptitrackBridgeComms: Unable to load WinSock" << std::endl;
    throw std::runtime_error("Socket exception");
  }
#endif

  // Make a new socket for sending OptiTrack updates.
  if ((this->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
  {
    std::cerr << "OptitrackBridgeComms: Socket creation failed." << std::endl;
    throw std::runtime_error("Socket exception");
  }

  // Set up the destination address.
  memset(&this->mySocketAddr, 0, sizeof(this->mySocketAddr));
  this->mySocketAddr.sin_family = AF_INET;
  this->mySocketAddr.sin_port = htons(zPortData);
  this->mySocketAddr.sin_addr.s_addr = inet_addr(zMulticastAddress.c_str());
}

/////////////////////////////////////////////////
OptitrackBridgeComms::~OptitrackBridgeComms()
{
  close(this->sock);
}

/////////////////////////////////////////////////
size_t OptitrackBridgeComms::MsgLength(const RigidBody_M &_trackingInfo)
{
  size_t len = 0;

  // Message ID (NatNet compatibility).
  len += sizeof(uint16_t);

  // Size (NatNet compatibility).
  len += sizeof(uint16_t);

  // Number of objects.
  len += sizeof(uint16_t);

  for (const auto &body : _trackingInfo)
  {
    // Rigid body name length.
    len += sizeof(uint64_t);
    // Rigid body name
    len += body.first.size();
    // All the stored fields for each rigid body.
    len += sizeof(float) * body.second.size();
  }
  return len;
}

/////////////////////////////////////////////////
size_t OptitrackBridgeComms::Pack(const RigidBody_M &_trackingInfo,
  std::vector<char> &_buffer)
{
  // Empty map.
  if (_trackingInfo.size() == 0)
  {
    std::cerr << "OptitrackBridgeComms::Pack() error: empty input" << std::endl;
    return 0;
  }

  // Allocate space in the buffer for the serialized data.
  size_t len = this->MsgLength(_trackingInfo);
  _buffer.resize(len);

  char *ptr = &_buffer[0];

  // Pack the Message ID (NatNet compatibility).
  uint16_t messageId = 666;
  memcpy(ptr, &messageId, sizeof(messageId));
  ptr += sizeof(messageId);

  // Pack the size (NatNet compatibility, not used here).
  uint16_t notused = 0;
  memcpy(ptr, &notused, sizeof(notused));
  ptr += sizeof(notused);

  // Pack the number of rigid bodies, which is a uint16_t
  uint16_t numBodies = _trackingInfo.size();
  memcpy(ptr, &numBodies, sizeof(numBodies));
  ptr += sizeof(numBodies);

  for (const auto &body : _trackingInfo)
  {
    // Pack the Rigid body name length.
    uint64_t nameLength = body.first.size();
    memcpy(ptr, &nameLength, sizeof(nameLength));
    ptr += sizeof(nameLength);

    // Pack the Rigid body name.
    memcpy(ptr, body.first.data(), static_cast<size_t>(nameLength));
    ptr += nameLength;

    // Pack the rigid body pose.
    for (const auto &elem : body.second)
    {
      std::cout << "Packing " << elem << std::endl;
      memcpy(ptr, &elem, sizeof(float));
      ptr += sizeof(float);
    }
  }

  return len;
}

/////////////////////////////////////////////////
bool OptitrackBridgeComms::Unpack(const char *_buffer,
  RigidBody_M &_trackingInfo)
{
  // null buffer.
  if (!_buffer)
  {
    std::cerr << "OptitrackBridgeComms::Unpack() error: NULL input"
              << std::endl;
    return false;
  }

  // Unpack the Message ID (NatNet compatibility).
  uint16_t messageId;
  memcpy(&messageId, _buffer, sizeof(messageId));
  _buffer += sizeof(messageId);

  // Unpack the size (NatNet compatibility, not used here).
  uint16_t notused;
  memcpy(&notused, _buffer, sizeof(notused));
  _buffer += sizeof(notused);

  // Unpack the number of rigid bodies.
  uint16_t numBodies;
  memcpy(&numBodies, _buffer, sizeof(numBodies));
  _buffer += sizeof(numBodies);

  for (int i = 0; i < numBodies; ++i)
  {
    // Unpack the rigid body name length.
    uint64_t nameLength;
    memcpy(&nameLength, _buffer, sizeof(nameLength));
    _buffer += sizeof(nameLength);

    // Unpack the rigid body name.
    std::string name = std::string(_buffer, _buffer + nameLength);
    _buffer += nameLength;

    std::array<float, 7> pose;
    for (int j = 0; j < 7; ++j)
    {
      // Unpack an element of the array.
      float v;
      memcpy(&v, _buffer, sizeof(v));
      _buffer += sizeof(v);

      // Update the array.
      pose[i] = v;
    }

    // Add the new rigid body entry to the output.
    _trackingInfo[name] = pose;
  }

  return true;
}

/////////////////////////////////////////////////
bool OptitrackBridgeComms::Send(const RigidBody_M &_trackingInfo)
{
  // Create a buffer.
  std::vector<char> buffer;

  // Allocate and serialize.
  int msgLength = this->Pack(_trackingInfo, buffer);
  if (msgLength <= 0)
  {
    std::cerr << "OptitrackBridgeComms::Send() Serialize error." << std::endl;
    return false;
  }

  // Send.
  if (sendto(this->sock, reinterpret_cast<const raw_type *>(
    reinterpret_cast<unsigned char*>(&buffer[0])),
    msgLength, 0, reinterpret_cast<sockaddr *>(&this->mySocketAddr),
    sizeof(this->mySocketAddr)) != msgLength)
  {
    std::cerr << "OptitrackBridgeComms:Send() Send error" << std::endl;
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
OptitrackBridge::OptitrackBridge(const std::string &/*_configFile*/)
{
  /*if (TT_Initialize() != NPRESULT_SUCCESS))
  {
    std::cerr << "TT_Initialize() error" << std::endl;
    throw std::runtime_error("OptiTrack exception");
  }*/

  /*if (TT_LoadProject(_confFile))
  {
    std::cerr << "TT_LoadProject() error" << std::endl;
    throw std::runtime_error("OptiTrack exception");
  }*/
}

/////////////////////////////////////////////////
OptitrackBridge::~OptitrackBridge()
{
  /*TT_Shutdown();
  TT_FinalCleanup();*/
}

/////////////////////////////////////////////////
bool OptitrackBridge::Update(RigidBody_M &_trackingInfo)
{
  _trackingInfo.clear();

  /*if (TT_Update() != NPRESULT_SUCCESS)
  {
    std::cerr << "TT_Update() error" << std::endl;
    return false;
  }

  // Ignore this frame if we are not tracking all the objects.
  if (TT_TrackableCount() != 3)
  {
    return false;
  }

  for (int i = 0; i < TT_TrackableCount(); ++i)
  {
    float x, y, z, qx, qy, qz, qw, yaw, pitch, roll;
    TT_TrackableLocation(i, &x, &y, &z, &qz, &qy, &qz, &qw,
      &yaw &pitch, &roll);

    // Store the rigid body pose.
    _trackingInfo[TT_TrackableName(i)] = {x, y, z, qx, qy, qz, qw};
  }
  */

  std::cout << "Update()" << std::endl;
  return true;
}

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that processes frames
/// and exit the program smoothly.
/*void signal_handler(int _signal)
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
   std::map<std::string, std::array<float, 7>> trackingInfo;
   Comms comms;

   // Try to send an empty map.
   if (comms.Send(trackingInfo))
    std::cerr << "FAIL: Send an empty map test failed" << std::endl;

  std::string head = "head";
  std::string monitor = "monitor";
  std::string hand = "hand";
  std::array<float, 7> headPose = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  std::array<float, 7> monitorPose = {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0};
  std::array<float, 7> handPose = {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0};
  std::map<std::string, std::array<float, 7>> m;
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
    Comms comms;
    std::string confFile = std::string(argv[1]);
    Optitrack optitrack(confFile);

    // \brief We will store in this variable all the tracking information.
    // Each element of the vector is a tracked rigid body. For each rigid body
    // we store 7 floats (x, y, z, qx, qy, qz, qw).
    std::map<std::string, std::array<float, 7>> trackingInfo;

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
}*/
