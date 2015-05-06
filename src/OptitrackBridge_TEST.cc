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

  std::string head    = "head";
  std::string monitor = "monitor";
  std::string hand    = "hand";
  Pose_t headPose     = { 1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f};
  Marker_t headM1     = { 1.1f,  2.1f,  3.1f};
  Marker_t headM2     = { 4.1f,  5.1f,  6.1f};
  Marker_t headM3     = { 7.1f,  8.1f,  9.1f};
  Pose_t monitorPose  = {11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f};
  Marker_t monitorM1  = {11.1f, 12.1f, 13.1f};
  Marker_t monitorM2  = {14.1f, 15.1f, 16.1f};
  Marker_t monitorM3  = {17.1f, 18.1f, 19.1f};
  Pose_t handPose     = {21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f};
  Marker_t handM1     = {21.1f, 22.1f, 23.1f};
  Marker_t handM2     = {24.1f, 25.1f, 26.1f};
  Marker_t handM3     = {27.1f, 28.1f, 29.1f};
  double timestamp    = 0.5;
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
    sizeof(uint64_t) + sizeof(Marker_t) * 3 +
    sizeof(uint64_t) + monitor.size()   + (monitorPose.size() * sizeof(float)) +
    sizeof(uint64_t) + sizeof(Marker_t) * 3 +
    sizeof(uint64_t) + hand.size()      + (handPose.size()    * sizeof(float)) +
    sizeof(uint64_t) + sizeof(Marker_t) * 3;

  EXPECT_EQ(comms.MsgLength(tracking1), expectedSize);

  // Allocate and serialize.
  size_t msgLength = comms.Pack(tracking1, buffer);
  ASSERT_EQ(msgLength, expectedSize);

  // Unpack data.
  TrackingInfo_t tracking2;
  EXPECT_TRUE(comms.Unpack(&buffer[0], tracking2));
  ASSERT_EQ(tracking2.bodies.size(), 3u);

  // Check data.
  EXPECT_DOUBLE_EQ(tracking2.timestamp, tracking1.timestamp);
  EXPECT_TRUE(tracking2.bodies.find(head)    != tracking1.bodies.end());
  EXPECT_TRUE(tracking2.bodies.find(monitor) != tracking1.bodies.end());
  EXPECT_TRUE(tracking2.bodies.find(hand)    != tracking1.bodies.end());

  // Head rigid body.
  auto pose = tracking2.bodies[head].body;
  ASSERT_EQ(pose.size(), 7u);
  auto markers = tracking2.bodies[head].markers;
  ASSERT_EQ(markers.size(), 3u);

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
  ASSERT_EQ(pose.size(), 7u);
  markers = tracking2.bodies[monitor].markers;
  ASSERT_EQ(markers.size(), 3u);

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
  ASSERT_EQ(pose.size(), 7u);
  markers = tracking2.bodies[hand].markers;
  ASSERT_EQ(markers.size(), 3u);

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
