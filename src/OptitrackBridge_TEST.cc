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

#include <cstdint>
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
  TrackingInfo_t tracking1;
  OptitrackBridgeComms comms;

  // Try to send an empty map.
  EXPECT_FALSE(comms.Send(tracking1));

  std::string head               = "head";
  std::string monitor            = "monitor";
  std::string hand               = "hand";
  RigidBody_A headPose           = { 1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0};
  Marker_A    headM1             = { 1.1,  2.1,  3.1};
  Marker_A    headM2             = { 4.1,  5.1,  6.1};
  Marker_A    headM3             = { 7.1,  8.1,  9.1};
  RigidBody_A monitorPose        = {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0};
  Marker_A    monitorM1          = {11.1, 12.1, 13.1};
  Marker_A    monitorM2          = {14.1, 15.1, 16.1};
  Marker_A    monitorM3          = {17.1, 18.1, 19.1};
  RigidBody_A handPose           = {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0};
  Marker_A    handM1             = {21.1, 22.1, 23.1};
  Marker_A    handM2             = {24.1, 25.1, 26.1};
  Marker_A    handM3             = {27.1, 28.1, 29.1};
  double timestamp               = 0.5;
  tracking1.timestamp               = timestamp;
  tracking1.bodies[head].body       = headPose;
  tracking1.bodies[head].markers    = {headM1, headM2, headM3};
  tracking1.bodies[monitor].body    = monitorPose;
  tracking1.bodies[monitor].markers = {monitorM1, monitorM2, monitorM3};
  tracking1.bodies[hand].body       = handPose;
  tracking1.bodies[hand].markers    = {handM1, handM2, handM3};

  // Create a buffer.
  std::vector<char> buffer;

  size_t expectedSize = sizeof(uint16_t) + sizeof(uint16_t) +
    sizeof(double)   + sizeof(uint16_t) +
    sizeof(uint64_t) + head.size()      + (headPose.size()    * sizeof(float)) +
    sizeof(uint64_t) + sizeof(Marker_A) * 3 +
    sizeof(uint64_t) + monitor.size()   + (monitorPose.size() * sizeof(float)) +
    sizeof(uint64_t) + sizeof(Marker_A) * 3 +
    sizeof(uint64_t) + hand.size()      + (handPose.size()    * sizeof(float)) +
    sizeof(uint64_t) + sizeof(Marker_A) * 3;

  EXPECT_EQ(comms.MsgLength(tracking1), expectedSize);

  // Allocate and serialize.
  int msgLength = comms.Pack(tracking1, buffer);
  ASSERT_EQ(msgLength, expectedSize);

  // Unpack data.
  TrackingInfo_t tracking2;
  EXPECT_TRUE(comms.Unpack(&buffer[0], tracking2));
  ASSERT_EQ(tracking2.bodies.size(), 3);

  // Check data.
  EXPECT_DOUBLE_EQ(tracking2.timestamp, tracking1.timestamp);
  EXPECT_TRUE(tracking2.bodies.find(head)    != tracking1.bodies.end());
  EXPECT_TRUE(tracking2.bodies.find(monitor) != tracking1.bodies.end());
  EXPECT_TRUE(tracking2.bodies.find(hand)    != tracking1.bodies.end());

  // Head rigid body.
  auto pose = tracking2.bodies[head].body;
  ASSERT_EQ(pose.size(), 7);
  auto markers = tracking2.bodies[head].markers;
  ASSERT_EQ(markers.size(), 3);

  float value = 1.0;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    ASSERT_FLOAT_EQ(pose.at(i), value);
    value += 1.0;
  }
  value = 1.1;
  for (const auto &marker : markers)
  {
    for (const auto &elem : marker)
    {
      ASSERT_FLOAT_EQ(elem, value);
      value += 1.0;
    }
  }

  // Monitor rigid body.
  pose = tracking2.bodies[monitor].body;
  ASSERT_EQ(pose.size(), 7);
  markers = tracking2.bodies[monitor].markers;
  ASSERT_EQ(markers.size(), 3);

  value = 11.0;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    ASSERT_FLOAT_EQ(pose.at(i), value);
    value += 1.0;
  }
  value = 11.1;
  for (const auto &marker : markers)
  {
    for (const auto &elem : marker)
    {
      ASSERT_FLOAT_EQ(elem, value);
      value += 1.0;
    }
  }

  // Hand rigid body.
  pose = tracking2.bodies[hand].body;
  ASSERT_EQ(pose.size(), 7);
  markers = tracking2.bodies[hand].markers;
  ASSERT_EQ(markers.size(), 3);

  value = 21.0;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    ASSERT_FLOAT_EQ(pose.at(i), value);
    value += 1.0;
  }
  value = 21.1;
  for (const auto &marker : markers)
  {
    for (const auto &elem : marker)
    {
      ASSERT_FLOAT_EQ(elem, value);
      value += 1.0;
    }
  }

  // Send some data.
  EXPECT_TRUE(comms.Send(tracking1));
}
