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

    /// \brief Array that stores the position of a marker.
    typedef std::array<float, 3> Marker_A;

    /// \brief Stores all the information about a rigid body.
    struct RigidBody_t
    {
      /// \brief Rigid body names, pose and orientation.
      RigidBody_A body;

      /// \brief Position of the markers contained in this rigid body.
      std::vector<Marker_A> markers;
    };

    /// \brief Map that contains rigid body information received from OptiTrack.
    /// The number of elements stored in the map are the number of rigid bodies
    /// tracked. For each element in the map, the key is a string with the name
    /// of the rigid body and the value is an array with floats storing the
    /// position and orientation of the object with the following format:
    /// (x, y, z, qx, qy, qz, qw).
    /// \sa RigidBody_A.
    typedef std::map<std::string, RigidBody_t> RigidBody_M;

    /// \brief Snapshot of all the information captured from Optitrack.
    /// It contains a timestamp and information about rigid bodies.
    /// \sa RigidBody_M.
    struct TrackingInfo_t
    {
      /// \brief Timestamp that shows when the data was captured.
      double timestamp;

      /// \brief Rigid body information.
      RigidBody_M bodies;
    };

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
      public: size_t MsgLength(const TrackingInfo_t &_trackingInfo);

      /// \brief Serialize a message containing tracking information.
      /// \param[in] _trackingInfo Tracking information to serialize.
      /// \param[out] _buffer Output buffer with the information serialized.
      /// \return Length of the serialized data in bytes or 0 if there was
      /// any error during serialization.
      public: size_t Pack(const TrackingInfo_t &_trackingInfo,
                          std::vector<char> &_buffer);

      /// \brief Unserialize a buffer containing tracking information.
      /// \param[in] _buffer Buffer containing the serialized message.
      /// \param[out] _trackingInfo Tracking information.
      public: bool Unpack(const char *_buffer,
                          TrackingInfo_t &_trackingInfo);

      /// \brief Send a new message over the network.
      /// \param[in] _trackingInfo Tracking information to send over the
      /// network.
      ///
      /// Wire protocol:
      /// 1. Message ID (666 - this runs on Windows, you know).
      ///    This field is here to make this protocol compatible with the NatNet
      ///    header (uint16_t)
      /// 2. Size (not used).
      ///    This field is here to make this protocol compatible with the NatNet
      ///    header (uint16_t).
      /// 3. Timestamp (double)
      /// 4. Number of rigid bodies tracked (uint16_t).
      /// For each rigid body:
      ///   5.  Name's length (uint64_t).
      ///   6.  Name (size in octets).
      ///   7.  x  (float).
      ///   8.  y  (float).
      ///   9.  z  (float).
      ///   10. qx (float).
      ///   11. qy (float).
      ///   12. qz (float).
      ///   13. qw (float).
      ///   14. Number of markers (uint64_t)
      ///   For each marker:
      ///     15. x (float)
      ///     16. y (float)
      ///     17. z (float)
      public: bool Send(const TrackingInfo_t &_trackingInfo);

      /// \brief Optitrack multicast address.
      private: const std::string zMulticastAddress = "239.255.42.99";

      /// \brief Port used for sending/receiving tracking updates.
      private: const int zPortData = 1511;

      /// \brief UDP socket used for sending tracking updates.
      private: int sock;

      /// \brief Internet socket address for sending to the multicast group.
      private: struct sockaddr_in mySocketAddr;
    };

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
      public: bool Update(TrackingInfo_t &_trackingInfo);
    };
  }
}
#endif
