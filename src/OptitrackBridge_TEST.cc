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

#include <string>
#include <vector>
#include "handsim/OptitrackBridge.hh"
#include "gtest/gtest.h"

using namespace haptix;
using namespace tracking;

//////////////////////////////////////////////////
/// \brief Check the OptitrackBridgeComms API.
TEST(OptitrackBridgeTest, IO)
{
  RigidBody_M trackingInfo;
  OptitrackBridgeComms comms;

  // Try to send an empty map.
  EXPECT_FALSE(comms.Send(trackingInfo));

  std::string head        = "head";
  std::string monitor     = "monitor";
  std::string hand        = "hand";
  RigidBody_A headPose    = { 1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0};
  RigidBody_A monitorPose = {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0};
  RigidBody_A handPose    = {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0};
  trackingInfo[head]      = headPose;
  trackingInfo[monitor]   = monitorPose;
  trackingInfo[hand]      = handPose;

  // Create a buffer.
  std::vector<char> buffer;

  size_t expectedSize = sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t) +
      sizeof(uint64_t) + head.size() + (headPose.size() * sizeof(float)) +
      sizeof(uint64_t) + monitor.size() + (monitorPose.size() * sizeof(float)) +
      sizeof(uint64_t) + hand.size() + (handPose.size() * sizeof(float));

  EXPECT_EQ(comms.MsgLength(trackingInfo), expectedSize);

  // Allocate and serialize.
  int msgLength = comms.Pack(trackingInfo, buffer);
  ASSERT_EQ(msgLength, expectedSize);

  RigidBody_M anotherTrackingInfo;
  EXPECT_TRUE(comms.Unpack(&buffer[0], anotherTrackingInfo));
  ASSERT_EQ(anotherTrackingInfo.size(), 3);

  // Send some data.
  EXPECT_TRUE(comms.Send(trackingInfo));

  // Unpack data.
  EXPECT_TRUE(trackingInfo.find(head) != trackingInfo.end());
  EXPECT_TRUE(trackingInfo.find(monitor) != trackingInfo.end());
  EXPECT_TRUE(trackingInfo.find(hand) != trackingInfo.end());
  auto pose = trackingInfo[head];
  ASSERT_EQ(pose.size(), 7);

  float value = 1.0;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    ASSERT_FLOAT_EQ(pose.at(i), value);
    value += 1.0;
  }
  pose = trackingInfo[monitor];
  ASSERT_EQ(pose.size(), 7);

  value = 11.0;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    ASSERT_FLOAT_EQ(pose.at(i), value);
    value += 1.0;
  }
  pose = trackingInfo[hand];
  ASSERT_EQ(pose.size(), 7);

  value = 21.0;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    ASSERT_FLOAT_EQ(pose.at(i), value);
    value += 1.0;
  }
}
