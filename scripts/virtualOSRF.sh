#!/bin/sh

## Copyright (C) 2015 Open Source Robotics Foundation
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.
##
## Author: Open Source Robotics Foundation.
## Description: Activate the remote control of this computer.

# Trap ctrl-c and call ctrl_c()
trap ctrl_c INT

# Callback executed when CTRL-C is captured.
ctrl_c ()
{
  kill -9 $PID
}

# Set the terminal title.
echo '\033]0;OSRF virtual support\007'

ssh -nNT -o "StrictHostKeyChecking no" -L localhost:5515:localhost:5500 -p 8188 haptix@support.osrfoundation.org &

# Get the PID of the ssh command for being able to kill it when CTRL-C the script.
PID=$!

# Give me some time to make the tunnel.
sleep 1

x11vnc -connect localhost:5515
