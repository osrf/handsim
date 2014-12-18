 /*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <string>
#include <thread>

namespace haptix
{
  namespace tracking
  {
    class Optitrack
    {
      #define MAX_NAMELENGTH              256

      /// \brief Creates an Optitrack object able to receive multicast updates
      /// from the Optitrack server containing the tracking information.
      /// \param[in] _server IP address of the optitrack server. This might be
      /// needed for requesting commands for tweaking the tracking behavior. The
      /// server IP is not needed for receiving tracking messages. These messages
      /// are received via multicast.
      public: Optitrack(const std::string &_serverIP = "");

      /// \brief Default destructor.
      public: ~Optitrack() = default;

      /// \brief Start receiving tracking updates. Each tracking update will be
      /// published as a Gazebo message on topic '~/haptix/optitrack'.
      public: void StartReception();

      /// \brief Receive tracking updates and publish them using Gazebo messages.
      private: void RunReceptionTask();

      /// \brief Unpack the data received from the network.
      /// \param[in] _data Buffer received.
      private: void Unpack(char *_data);

      /// ToDo.
      private: bool TimecodeStringify(unsigned int _inTimecode,
                                      unsigned int _inTimecodeSubframe,
                                      char *_buffer,
                                      int _bufferSize);
      /// ToDo.
      private: bool DecodeTimecode(unsigned int _inTimecode,
                                   unsigned int _inTimecodeSubframe,
                                   int *_hour,
                                   int *_minute,
                                   int *_second,
                                   int *_frame,
                                   int *_subframe);

      /// \brief Optitrack multicast address.
      private: const std::string MulticastAddress = "239.255.42.99";

      /// \brief Port used for sending/receiving commands.
      private: const int PortCommand = 1510;

      /// \brief Port used for sending/receiving tracking updates.
      private: const int PortData = 1511;

      /// \brief NatNet major version.
      private: const int NatNetVersionMajor = 0;

      /// \brief NatNet minor version.
      private: const int NatNetVersionMinor = 0;

      /// \brief IP address of the optitrack server.
      private: std::string serverIP;

      /// \brief UDP socket used to received tracking updates.
      private: int dataSocket;

      /// \brief IP address associated to the multicast socket.
      /// ToDo: For now, this is hardcoded but we should read it using
      /// ign-transport, environment variable, etc.
      private: const std::string myIPAddress = "172.23.2.37";

      /// \brief Thread used for receiving tracking updates.
      private: std::thread *dataThread = nullptr;
    };
  }
}
#endif
