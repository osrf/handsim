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
#include <stdint.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "handsim/OptitrackBridge.hh"

#ifdef _WIN32
  #include "NPTrackingTools.h"
#endif

using namespace haptix;
using namespace tracking;

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
#ifdef _WIN32
  closesocket(this->sock);
#else
  close(this->sock);
#endif
}

/////////////////////////////////////////////////
size_t OptitrackBridgeComms::MsgLength(const TrackingInfo_t &_trackingInfo)
{
  size_t len = 0;

  // Message ID (NatNet compatibility).
  len += sizeof(uint16_t);

  // Size (NatNet compatibility).
  len += sizeof(uint16_t);

  // Timestamp.
  len += sizeof(double);

  // Number of objects.
  len += sizeof(uint16_t);

  for (const auto &body : _trackingInfo.bodies)
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
size_t OptitrackBridgeComms::Pack(const TrackingInfo_t &_trackingInfo,
  std::vector<char> &_buffer)
{
  // No rigid body information.
  if (_trackingInfo.bodies.size() == 0)
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

  // Pack the timestamp.
  memcpy(ptr, &_trackingInfo.timestamp, sizeof(_trackingInfo.timestamp));
  ptr += sizeof(_trackingInfo.timestamp);

  // Pack the number of rigid bodies, which is a uint16_t
  uint16_t numBodies = _trackingInfo.bodies.size();
  memcpy(ptr, &numBodies, sizeof(numBodies));
  ptr += sizeof(numBodies);

  for (const auto &body : _trackingInfo.bodies)
  {
    // Pack the Rigid body name length.
    uint64_t nameLength = body.first.size();
    memcpy(ptr, &nameLength, sizeof(nameLength));
    ptr += sizeof(nameLength);

    // Pack the Rigid body name.
    memcpy(ptr, body.first.data(), static_cast<size_t>(nameLength));
    ptr += nameLength;

    // Pack the rigid body pose.
    RigidBody_A pose = body.second;
    memcpy(ptr, &pose[0], sizeof(float) * pose.size());
    ptr += sizeof(float) * pose.size();
  }

  return len;
}

/////////////////////////////////////////////////
bool OptitrackBridgeComms::Unpack(const char *_buffer,
  TrackingInfo_t &_trackingInfo)
{
  // null buffer.
  if (!_buffer)
  {
    std::cerr << "OptitrackBridgeComms::Unpack() error: NULL input"
              << std::endl;
    return false;
  }

  _trackingInfo.bodies.clear();

  // Unpack the Message ID (NatNet compatibility).
  uint16_t messageId;
  memcpy(&messageId, _buffer, sizeof(messageId));
  _buffer += sizeof(messageId);

  // Unpack the size (NatNet compatibility, not used here).
  uint16_t notused;
  memcpy(&notused, _buffer, sizeof(notused));
  _buffer += sizeof(notused);

  // Unpack the timestamp.
  double timestamp;
  memcpy(&timestamp, _buffer, sizeof(timestamp));
  _buffer += sizeof(timestamp);

  // Add the new timestamp.
  _trackingInfo.timestamp = timestamp;

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

    // Unpack the rigid body pose.
    RigidBody_A pose;
    memcpy(&pose[0], _buffer, pose.size() * sizeof(float));
    _buffer += pose.size() * sizeof(float);

    // Add the new rigid body entry to the output.
    _trackingInfo.bodies[name] = pose;
  }

  return true;
}

/////////////////////////////////////////////////
bool OptitrackBridgeComms::Send(const TrackingInfo_t &_trackingInfo)
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
OptitrackBridge::OptitrackBridge(const std::string &_motiveConfFile)
{
#ifdef _WIN32
  if (TT_Initialize() != NPRESULT_SUCCESS)
  {
    std::cerr << "TT_Initialize() error" << std::endl;
    throw std::runtime_error("OptiTrack exception");
  }

  if (TT_LoadProject(_motiveConfFile.c_str()))
  {
    std::cerr << "TT_LoadProject() error" << std::endl;
    throw std::runtime_error("OptiTrack exception");
  }
#elif __linux__
  (void)_motiveConfFile;
#endif
}

/////////////////////////////////////////////////
OptitrackBridge::~OptitrackBridge()
{
#ifdef _WIN32
  TT_Shutdown();
  TT_FinalCleanup();
#endif
}

/////////////////////////////////////////////////
bool OptitrackBridge::Update(TrackingInfo_t &_trackingInfo)
{
  _trackingInfo.bodies.clear();

#ifdef _WIN32
  if (TT_Update() != NPRESULT_SUCCESS)
    return false;

  // Store the timestamp.
  _trackingInfo.timestamp = TT_FrameTimeStamp();

  for (int i = 0; i < TT_TrackableCount(); ++i)
  {
    if(TT_IsTrackableTracked(i))
    {
      float x, y, z;
      float qx, qy, qz, qw;
      float yaw, pitch, roll;
      TT_TrackableLocation(i, &x, &y, &z, &qx, &qy, &qz, &qw,
        &yaw, &pitch, &roll);

      // Store the rigid body pose.
      _trackingInfo.bodies[TT_TrackableName(i)] = {-x, y, z, -qx, qy, qz, -qw};
    }
  }
#endif

  return true;
}
