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

#ifndef _HANDSIM_OPTITRACKBRIDGE_HH_
#define _HANDSIM_OPTITRACKBRIDGE_HH_

#ifdef _WIN32
  #include <Winsock2.h>
#else
  #include <arpa/inet.h>
#endif

#include <array>
#include <string>
#include <map>
#include <vector>

namespace haptix
{
  namespace tracking
  {
    /// \brief Array that stores the position and orientation of a rigid body
    /// tracked by OptiTrack with the following format:
    /// (x, y, z, qx, qy, qz, qw).
    typedef std::array<float, 7> RigidBody_A;
    /// \brief Map that contains rigid body information received from OptiTrack.
    /// The number of elements stored in the map are the number of rigid bodies
    /// tracked. For each element in the map, the key is a string with the name
    /// of the rigid body and the value is an array with floats storing the
    /// position and orientation of the object with the following format:
    /// (x, y, z, qx, qy, qz, qw).
    typedef std::map<std::string, RigidBody_A> RigidBody_M;

    /// \brief Class that implements some utilities to communicate with the
    /// Optitrack bridge.
    class OptitrackBridgeComms
    {
      // \brief Constructor.
      // \throw SocketException If a socket error occurs during initialization.
      public: OptitrackBridgeComms();

      /// \brief Destructor.
      public: ~OptitrackBridgeComms();

      /// \brief Get the length of a serialized message.
      /// \param[in] _trackingInfo Tracking information.
      /// \return Length in bytes.
      public: size_t MsgLength(const RigidBody_M &_trackingInfo);

      /// \brief Serialize a message containing rigid body information.
      /// \param[in] _trackingInfo Tracking information to serialize.
      /// \param[out] _buffer Output buffer with the information serialized.
      /// \return Length of the serialized data in bytes or 0 if there was
      /// any error during serialization.
      public: size_t Pack(const RigidBody_M &_trackingInfo,
                          std::vector<char> &_buffer);

      /// \brief Unserialize a buffer containing tracking information.
      /// \param[in] _buffer Buffer containing the serialized message.
      /// \param[out] _trackingInfo Tracking information.
      public: bool Unpack(const char *_buffer,
                          RigidBody_M &_trackingInfo);

      /// \brief Send a new message over the network.
      /// \param[in] _trackingInfo Tracking information to send over the
      /// network.
      ///
      /// Wire protocol:
      /// 1. Message ID (666 - this runs on Windows, you know). This field is
      //// here to make this protocol compatible with the NatNet header.
      /// 2. Size (not used). This field is
      //// here to make this protocol compatible with the NatNet header.
      /// 3. Number of rigid bodies tracked.
      /// For each rigid body:
      ///   4.  Name's length (uint64_t).
      ///   5.  Name (size in octets).
      ///   6.  x  (float).
      ///   7.  y  (float).
      ///   8.  z  (float).
      ///   9.  qx (float).
      ///   10. qy (float).
      ///   11. qz (float).
      ///   12. qw (float).
      public: bool Send(const RigidBody_M &_trackingInfo);

      /// \brief Port used for sending/receiving tracking updates.
      private: static const int zPortData = 1511;

      /// \brief UDP socket used for sending tracking updates.
      private: int sock;

      /// \brief Internet socket address for sending to the multicast group.
      private: struct sockaddr_in mySocketAddr;
    };

#ifdef _WIN32
    /// \brief Class that implements some utilites to interact with an Optitrack
    /// device.
    class OptitrackBridge
    {
      /// \brief Initializes the OptiTrack device and loads its configuration.
      /// \param[in] _motiveConfFile Motive project file containing camera
      /// parameters and rigid body configuration.
      /// \throw OptitrackException If a socket error occurs during
      /// initialization.
      public: OptitrackBridge(const std::string &_motiveConfFile);

      /// \brief Destructor.
      public: ~OptitrackBridge();

      /// \brief Process a new camera frame and apply 3D reconstruction and
      /// rigid body tracking.
      /// \param[out] _trackingInfo Return the vector of rigid bodies tracked.
      /// Each rigid body contains six floats (x, y, z, roll, pitch, yaw).
      public: bool Update(RigidBody_M &_trackingInfo);
    };
#endif
  }
}
#endif
