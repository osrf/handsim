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

#include <cstring>
#include <iostream>
#include <string>

#include <ignition/transport/NetUtils.hh>
#include <gazebo/msgs/msgs.hh>

#include "handsim/Optitrack.hh"

using namespace haptix;
using namespace tracking;

const std::string Optitrack::headTrackerName = "HeadTracker";
const std::string Optitrack::armTrackerName = "ArmTracker";
const std::string Optitrack::originTrackerName = "MonitorTracker";

/////////////////////////////////////////////////
Optitrack::Optitrack(const std::string &_serverIP)
  : serverIP(_serverIP)
{
  this->active = false;
  //this->myIPAddress = ignition::transport::determineHost();
  this->myIPAddress = "172.23.3.172";

  // Create a socket for receiving tracking updates.
  this->dataSocket = socket(AF_INET, SOCK_DGRAM, 0);

  // Socket option: SO_REUSEADDR.
  int value = 1;
  if (setsockopt(this->dataSocket, SOL_SOCKET, SO_REUSEADDR,
    reinterpret_cast<const char *>(&value), sizeof(value)) != 0)
  {
    std::cerr << "Error setting socket option (SO_REUSEADDR)." << std::endl;
    close(this->dataSocket);
    return;
  }

  // Bind the socket to the "Optitrack tracking updates" port.
  struct sockaddr_in mySocketAddr;
  memset(&mySocketAddr, 0, sizeof(mySocketAddr));
  mySocketAddr.sin_family = AF_INET;
  mySocketAddr.sin_port = htons(this->PortData);
  mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);;
  if (bind(this->dataSocket, (struct sockaddr *)&mySocketAddr,
    sizeof(struct sockaddr)) < 0)
  {
    std::cerr << "Binding to a local port failed." << std::endl;
    return;
  }
  // Join the multicast group.
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(this->MulticastAddress.c_str());
  mreq.imr_interface.s_addr = inet_addr(this->myIPAddress.c_str());
  if (setsockopt(this->dataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
    reinterpret_cast<const char*>(&mreq), sizeof(mreq)) != 0)
  {
    std::cerr << "Error setting socket option (IP_ADD_MEMBERSHIP)."
              << std::endl;
    return;
  }
  this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  //std::thread DataListenThread_Handle(DataListenThread);
}

/////////////////////////////////////////////////
void Optitrack::StartReception(const std::string &_world)
{
  if (_world.size() > 0)
  {
    this->gzNode->Init(_world);
  }

  this->headPub = this->gzNode->Advertise<gazebo::msgs::Pose>
                    ("~/optitrack/"+headTrackerName);
  this->armPub = this->gzNode->Advertise<gazebo::msgs::Pose>
                    ("~/optitrack/"+armTrackerName);
  this->originPub = this->gzNode->Advertise<gazebo::msgs::Pose>
                    ("~/optitrack/"+originTrackerName);

  if (!this->dataThread)
  {
    this->dataThread = new std::thread(&Optitrack::RunReceptionTask, this);
    this->active = true;
  }
}

/////////////////////////////////////////////////
bool Optitrack::IsActive()
{
  return this->active;
}

/////////////////////////////////////////////////
void Optitrack::RunReceptionTask()
{
  char buffer[20000];
  socklen_t addr_len = sizeof(struct sockaddr);
  sockaddr_in theirAddress;

  while (1)
  {
    // Block until we receive a datagram from the network (from anyone
    //  including ourselves)
    if (recvfrom(this->dataSocket, buffer, sizeof(buffer), 0,
         (sockaddr *)&theirAddress, &addr_len) < 0)
    {
      std::cerr << "Optitrack::RunReceptionTask() Recvfrom failed" << std::endl;
      continue;
    }

    // Dispach the data received.
    this->Unpack(buffer);

    // Publish messages

    for (ModelPoses::iterator it = this->lastModelMap.begin();
         it != this->lastModelMap.end(); it++)
    {
      if (it->first.compare(headTrackerName) == 0)
      {
        this->headPub->Publish(gazebo::msgs::Convert(it->second));
      }
      else if (it->first.compare(armTrackerName) == 0)
      {
        this->armPub->Publish(gazebo::msgs::Convert(it->second));
      }
      else if (it->first.compare(originTrackerName) == 0)
      {
        this->originPub->Publish(gazebo::msgs::Convert(it->second));
      }
      else
      {
        std::cout << "Model name " << it->first << " not found!" << std::endl;
      }
    }

    this->lastModelMap.clear();
    // probably want to sleep here
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  this->active = false;
}

/////////////////////////////////////////////////
void Optitrack::Unpack(char *pData)
{
  int major = this->NatNetVersionMajor;
  int minor = this->NatNetVersionMinor;

  char *ptr = pData;

  // message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2); ptr += 2;

  // size
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2); ptr += 2;

  if(MessageID == 7)      // FRAME OF MOCAP DATA packet
  {
    // frame number
    int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
    printf("Frame # : %d\n", frameNumber);

    // number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
    printf("Marker Set Count : %d\n", nMarkerSets);

    ModelMarkers markerSets;

    for (int i=0; i < nMarkerSets; i++)
    {
      // Markerset name
      std::string szName(ptr);
      //strcpy_s(szName, ptr);
      int nDataBytes = (int) strlen(ptr) + 1;
      ptr += nDataBytes;
      std::cout << "Model Name: " << szName << std::endl;
      markerSets[szName] = std::vector<gazebo::math::Vector3>();

      // marker data
      int nMarkers = 0;
      memcpy(&nMarkers, ptr, 4);
      ptr += 4;
      printf("Marker Count : %d\n", nMarkers);

      for(int j=0; j < nMarkers; j++)
      {
        float x = 0; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0; memcpy(&z, ptr, 4); ptr += 4;
        printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n",j,x,y,z);
        markerSets[szName].push_back(gazebo::math::Vector3(x, y, z));
      }
    }

    // unidentified markers
    int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
    for(int j=0; j < nOtherMarkers; j++)
    {
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
    }

    // rigid bodies
    int nRigidBodies = 0;
    memcpy(&nRigidBodies, ptr, 4); ptr += 4;
    printf("Rigid Body Count : %d\n", nRigidBodies);
    for (int j=0; j < nRigidBodies; j++)
    {
      // rigid body pos/ori
      int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
      float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
      float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
      float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
      float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
      printf("ID : %d\n", ID);
      printf("pos: [%3.2f,%3.2f,%3.2f]\n", x,y,z);
      printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);

      gazebo::math::Pose rigidBodyPose(gazebo::math::Vector3(x, y, z),
                             gazebo::math::Quaternion(qw, qx, qy, qz));

      // associated marker positions
      int nRigidMarkers = 0; 
      memcpy(&nRigidMarkers, ptr, 4);
      ptr += 4;
      printf("Marker Count: %d\n", nRigidMarkers);
      int nBytes = nRigidMarkers*3*sizeof(float);
      float* markerData = (float*)malloc(nBytes);
      memcpy(markerData, ptr, nBytes);
      ptr += nBytes;


      // This is terrible and the NatNet packet format sucks
      for (ModelMarkers::iterator it = markerSets.begin();
           it != markerSets.end(); it++)
      {
        if (it->second.size() == nRigidMarkers)
        {
          // Compare all the points
          int k = 0;
          for (k = 0; k < nRigidMarkers; k++)
          {
            if (std::fabs(it->second[k][0] - markerData[k*3]) > FLT_EPSILON ||
              std::fabs(it->second[k][1] - markerData[k*3+1]) > FLT_EPSILON ||
              std::fabs(it->second[k][2] - markerData[k*3+2]) > FLT_EPSILON)
            {
              break;
            }
          }
          if (k == nRigidMarkers)
          {
            std::cout << "Pushing back to map: " << it->first << std::endl;
            this->lastModelMap[it->first] = rigidBodyPose;
          }
        }
      }

      if(major >= 2)
      {
        // associated marker IDs
        nBytes = nRigidMarkers*sizeof(int);
        int* markerIDs = (int*)malloc(nBytes);
        memcpy(markerIDs, ptr, nBytes);
        ptr += nBytes;

        // associated marker sizes
        nBytes = nRigidMarkers*sizeof(float);
        float* markerSizes = (float*)malloc(nBytes);
        memcpy(markerSizes, ptr, nBytes);
        ptr += nBytes;

        /*for(int k=0; k < nRigidMarkers; k++)
        {
          printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n",
                 k, markerIDs[k], markerSizes[k],
                 markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
        }*/

        if(markerIDs)
          free(markerIDs);
        if(markerSizes)
          free(markerSizes);

      }
      else
      {
        for(int k=0; k < nRigidMarkers; k++)
        {
            printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k, markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
        }
      }
      if(markerData)
        free(markerData);

      if(major >= 2)
      {
        // Mean marker error
        float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
        printf("Mean marker error: %3.2f\n", fError);
      }

      // 2.6 and later
      if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) )
      {
        // params
        short params = 0; memcpy(&params, ptr, 2); ptr += 2;
        bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
      }

    } // next rigid body


    // skeletons (version 2.1 and later)
    if( ((major == 2)&&(minor>0)) || (major>2))
    {
      int nSkeletons = 0;
      memcpy(&nSkeletons, ptr, 4); ptr += 4;
      printf("Skeleton Count : %d\n", nSkeletons);
      for (int j=0; j < nSkeletons; j++)
      {
        // skeleton id
        int skeletonID = 0;
        memcpy(&skeletonID, ptr, 4); ptr += 4;
        // # of rigid bodies (bones) in skeleton
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        printf("Rigid Body Count : %d\n", nRigidBodies);
        for (int j=0; j < nRigidBodies; j++)
        {
          // rigid body pos/ori
          int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
          float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
          float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
          float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
          float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
          float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
          float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
          float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
          printf("ID : %d\n", ID);
          printf("pos: [%3.2f,%3.2f,%3.2f]\n", x,y,z);
          printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);

          // associated marker positions
          int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
          printf("Marker Count: %d\n", nRigidMarkers);
          int nBytes = nRigidMarkers*3*sizeof(float);
          float* markerData = (float*)malloc(nBytes);
          memcpy(markerData, ptr, nBytes);
          ptr += nBytes;

          // associated marker IDs
          nBytes = nRigidMarkers*sizeof(int);
          int* markerIDs = (int*)malloc(nBytes);
          memcpy(markerIDs, ptr, nBytes);
          ptr += nBytes;

          // associated marker sizes
          nBytes = nRigidMarkers*sizeof(float);
          float* markerSizes = (float*)malloc(nBytes);
          memcpy(markerSizes, ptr, nBytes);
          ptr += nBytes;

          for(int k=0; k < nRigidMarkers; k++)
          {
            printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
          }

          // Mean marker error
          float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
          printf("Mean marker error: %3.2f\n", fError);

          // release resources
          if(markerIDs)
            free(markerIDs);
          if(markerSizes)
            free(markerSizes);
          if(markerData)
            free(markerData);

        } // next rigid body

      } // next skeleton
    }

  // labeled markers (version 2.3 and later)
  if( ((major == 2)&&(minor>=3)) || (major>2))
  {
    int nLabeledMarkers = 0;
    memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
    printf("Labeled Marker Count : %d\n", nLabeledMarkers);
    for (int j=0; j < nLabeledMarkers; j++)
    {
      // id
      int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
      // x
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      // y
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      // z
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
      // size
      float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;

              // 2.6 and later
              if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) )
              {
                  // marker params
                  short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                  bool bOccluded = params & 0x01;     // marker was not visible (occluded) in this frame
                  bool bPCSolved = params & 0x02;     // position provided by point cloud solve
                  bool bModelSolved = params & 0x04;  // position provided by model solve
              }

      printf("ID  : %d\n", ID);
      printf("pos : [%3.2f,%3.2f,%3.2f]\n", x,y,z);
      printf("size: [%3.2f]\n", size);
    }
  }

  // latency
      float latency = 0.0f; memcpy(&latency, ptr, 4); ptr += 4;
      printf("latency : %3.3f\n", latency);

  // timecode
  unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);  ptr += 4;
  unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
  char szTimecode[128] = "";
  this->TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

      // timestamp
      double timestamp = 0.0f;
      // 2.7 and later - increased from single to double precision
      if( ((major == 2)&&(minor>=7)) || (major>2))
      {
          memcpy(&timestamp, ptr, 8); ptr += 8;
      }
      else
      {
          float fTemp = 0.0f;
          memcpy(&fTemp, ptr, 4); ptr += 4;
          timestamp = (double)fTemp;
      }

      // frame params
      short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
      bool bIsRecording = params & 0x01;                  // 0x01 Motive is recording
      bool bTrackedModelsChanged = params & 0x02;         // 0x02 Actively tracked model list has changed


  // end of data tag
      int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
      printf("End Packet\n-------------\n");

  }
  else if(MessageID == 5) // Data Descriptions
  {
      // number of datasets
      int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
      printf("Dataset Count : %d\n", nDatasets);

      for(int i=0; i < nDatasets; i++)
      {
        printf("Dataset %d\n", i);

        int type = 0; memcpy(&type, ptr, 4); ptr += 4;
        printf("Type : %d %d\n", i, type);

        if(type == 0)   // markerset
        {
          // name
          std::string szName(ptr);
          int nDataBytes = (int) strlen(ptr) + 1;
          ptr += nDataBytes;
          std::cout << "Markerset Name: " << szName << std::endl;

          // marker data
          int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
          printf("Marker Count : %d\n", nMarkers);

          for(int j=0; j < nMarkers; j++)
          {
              std::string szName(ptr);
              int nDataBytes = (int) strlen(ptr) + 1;
              ptr += nDataBytes;
              std::cout << "Marker Name: " << szName << std::endl;
          }
        }
        else if(type ==1)   // rigid body
        {
          if(major >= 2)
          {
              // name
              char szName[MAX_NAMELENGTH];
              strcpy(szName, ptr);
              ptr += strlen(ptr) + 1;
              printf("Name: %s\n", szName);
          }

          int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
          printf("ID : %d\n", ID);

          int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
          printf("Parent ID : %d\n", parentID);

          float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
          printf("X Offset : %3.2f\n", xoffset);

          float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
          printf("Y Offset : %3.2f\n", yoffset);

          float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
          printf("Z Offset : %3.2f\n", zoffset);

        }
        else if(type ==2)   // skeleton
        {
          char szName[MAX_NAMELENGTH];
          strcpy(szName, ptr);
          ptr += strlen(ptr) + 1;
          printf("Name: %s\n", szName);

          int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
          printf("ID : %d\n", ID);

          int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
          printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

          for(int i=0; i< nRigidBodies; i++)
          {
            if(major >= 2)
            {
              // RB name
              char szName[MAX_NAMELENGTH];
              strcpy(szName, ptr);
              ptr += strlen(ptr) + 1;
              printf("Rigid Body Name: %s\n", szName);
            }

            int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
            printf("RigidBody ID : %d\n", ID);

            int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
            printf("Parent ID : %d\n", parentID);

            float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
            printf("X Offset : %3.2f\n", xoffset);

            float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
            printf("Y Offset : %3.2f\n", yoffset);

            float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
            printf("Z Offset : %3.2f\n", zoffset);
          }
        }

      }   // next dataset

     printf("End Packet\n-------------\n");

  }
  else
  {
      printf("Unrecognized Packet Type.\n");
  }
}

/////////////////////////////////////////////////
bool Optitrack::TimecodeStringify(unsigned int _inTimecode,
  unsigned int _inTimecodeSubframe, char *_buffer, int _bufferSize)
{
  bool bValid;
  int hour, minute, second, frame, subframe;
  bValid = this->DecodeTimecode(_inTimecode, _inTimecodeSubframe,
    &hour, &minute, &second, &frame, &subframe);

  //ToDo(caguero): Fix this.
  /*sprintf(Buffer, BufferSize, "%2d:%2d:%2d:%2d.%d", hour, minute, second, frame, subframe);
  for (unsigned int i=0; i<strlen(Buffer); i++)
    if (Buffer[i]==' ')
      Buffer[i]='0';*/

  return bValid;
}

/////////////////////////////////////////////////
bool Optitrack::DecodeTimecode(unsigned int _inTimecode,
  unsigned int _inTimecodeSubframe, int *_hour, int *_minute, int *_second,
  int *_frame, int *_subframe)
{
  bool bValid = true;

  *_hour     = (_inTimecode >> 24) & 255;
  *_minute   = (_inTimecode >> 16) & 255;
  *_second   = (_inTimecode >> 8) & 255;
  *_frame    = _inTimecode & 255;
  *_subframe = _inTimecodeSubframe;

  return bValid;
}
