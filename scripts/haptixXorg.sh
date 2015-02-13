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
## Description: Verifies that /etc/X11/xorg.conf is a symbolink link and points
## to /etc/X11/xorg.conf.haptix . We are trying to force the system to use
## always the HAPTIX xorg.conf configuration file.

if [ "$(readlink /etc/X11/xorg.conf)" != "/etc/X11/xorg.conf.haptix" ]
then
  ln -sf /etc/X11/xorg.conf.haptix /etc/X11/xorg.conf
  logger -s "Restoring HAPTIX xorg.conf file"
fi

