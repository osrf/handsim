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

#ifndef _HANDSIM_OPTITRACK_HH_
#define _HANDSIM_OPTITRACK_HH_

#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <gazebo/transport/transport.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

#include "handsim/OptitrackBridge.hh"

namespace haptix
{
  namespace tracking
  {
    typedef std::map<std::string, std::vector<gazebo::math::Vector3> >
              ModelMarkers;

    typedef std::map<std::string, gazebo::math::Pose> ModelPoses;

    class Optitrack
    {
      #define MAX_NAMELENGTH              256

      /// \brief Creates an Optitrack object able to receive multicast updates
      /// from the Optitrack server containing the tracking information.
      /// \param[in] _server IP address of the optitrack server. This might be
      /// needed for requesting commands for tweaking the tracking behavior. The
      /// server IP is not needed for receiving tracking messages. These
      /// messages are received via multicast.
      /// \param[in] _verbose Whether or not to print incoming packets.
      public: Optitrack(const std::string &_serverIP = "",
                        const bool _verbose = false,
                        const std::string &_world="");

      /// \brief Copy constructor. Currently unimplemented
      /// \param[in] Optitrack object to copy
      public: Optitrack(const Optitrack &_optitrack);

      /// \brief Default destructor.
      public: ~Optitrack() = default;

      /// \brief Start receiving tracking updates. Each tracking update will be
      /// published as a Gazebo message on topic '~/optitrack'.
      public: void StartReception();

      /// \brief Receive tracking updates and publish them using Gazebo messages
      private: void RunReceptionTask();

      /// \brief Unpack the data received from the network.
      /// \param[in] _data Buffer received.
      private: void Unpack(char *_data);

      /// \brief Return the status of the Optitrack client initialization.
      /// \return True if Optitrack data reception is active..
      public: bool IsActive();

      /// \brief Stop activity.
      public: void Stop();

      /// \brief Set the name of the world associated with the gz publishers.
      public: void SetWorld(const std::string &_world);

      /// \brief Set whether or not to print packets as they come.
      public: void SetVerbose(const bool _verbose);

      /// \brief True if Optitrack data reception is active
      private: bool active;

      /// \brief Optitrack multicast address.
      private: const std::string multicastAddress = "239.255.42.99";

      /// \brief Port used for sending/receiving commands.
      private: const int portCommand = 1510;

      /// \brief Port used for sending/receiving tracking updates.
      private: const int portData = 1511;

      /// \brief NatNet major version.
      private: const int natNetVersionMajor = 2;

      /// \brief NatNet minor version.
      private: const int natNetVersionMinor = 7;

      /// \brief How long to sleep between packet updates
      private: const int sleepMicroseconds = 500;

      /// \brief IP address of the optitrack server.
      private: std::string serverIP;

      /// \brief True if incoming packets will be printed
      private: bool verbose;

      /// \brief UDP socket used to received tracking updates.
      private: int dataSocket;

      /// \brief List of network interfaces associated to the multicast socket.
      private: std::vector<std::string> myNetworkInterfaces;

      /// \brief Name of the world associated with the Gazebo publishers.
      private: std::string world;

      /// \brief Gazebo transport node used to publish tracker poses.
      private: gazebo::transport::NodePtr gzNode;

      /// \brief Gazebo publisher for head tracker pose
      private: gazebo::transport::PublisherPtr headPub;

      /// \brief Gazebo publisher for arm tracker pose
      private: gazebo::transport::PublisherPtr armPub;

      /// \brief Gazebo publisher for monitor tracker pose
      private: gazebo::transport::PublisherPtr originPub;

      /// \brief Gazebo publisher for tracker data liveliness
      private: gazebo::transport::PublisherPtr optitrackAlivePub;

      /// \brief Name of head tracker rigid body
      public: static const std::string headTrackerName;

      /// \brief Name of arm tracker rigid body
      public: static const std::string armTrackerName;

      /// \brief Name of monitor tracker rigid body
      public: static const std::string originTrackerName;

      /// \brief Name of topic for liveliness
      public: static const std::string optitrackAliveTopic;

      /// \brief Store names and poses of tracker models to be published
      private: ModelPoses lastModelMap;

      /// \brief Allow communication with the OptiTrack bridge.
      private: OptitrackBridgeComms comms;

      /// \brief Mutex to protect "active" boolean
      private: std::mutex activeMutex;
    };
  }
}
#endif
