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

#include <array>
#include <map>
#include "handsim/OptitrackBridge.hh"
#include "gtest/gtest.h"

//////////////////////////////////////////////////
/// \brief Check the serialization and unserialization of a header.
TEST(OptitrackBridgeTest, IO)
{
  std::map<std::string, std::array<float, 7>> trackingInfo;
  //Comms comms;

  // Try to send an empty map.
  //EXPECT_FALSE(comms.Send(trackingInfo));

  /*uint16_t numRigidBodies = 3;*/

  /*std::vector<char> buffer(emptyHeader.GetHeaderLength());
  EXPECT_EQ(emptyHeader.Pack(&buffer[0]), 0u);

  // Pack a Header.
  transport::Header header(version, pUuid, transport::AdvSrvType, 2);

  buffer.resize(header.GetHeaderLength());
  int bytes = header.Pack(&buffer[0]);
  EXPECT_EQ(bytes, header.GetHeaderLength());

  // Unpack the Header.
  transport::Header otherHeader;
  otherHeader.Unpack(&buffer[0]);

  // Check that after Pack() and Unpack() the Header remains the same.
  EXPECT_EQ(header.GetVersion(), otherHeader.GetVersion());
  EXPECT_EQ(header.GetPUuid(), otherHeader.GetPUuid());
  EXPECT_EQ(header.GetType(), otherHeader.GetType());
  EXPECT_EQ(header.GetFlags(), otherHeader.GetFlags());
  EXPECT_EQ(header.GetHeaderLength(), otherHeader.GetHeaderLength());

  // Try to pack a header passing a NULL buffer.
  EXPECT_EQ(otherHeader.Pack(nullptr), 0u);

  // Try to unpack a header passing a NULL buffer.
  EXPECT_EQ(otherHeader.Unpack(nullptr), 0u);*/
}
