# HAPTIX Simulator #

## Installation instructions: using catkin_make ##
To compile, first install gazebo from haptix branch, ign-transport and haptix-comm from default branch.  Next:

~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
hg clone https://bitbucket.org/osrf/handsim
hg clone https://bitbucket.org/osrf/[haptix_model]
cd ~/catkin_ws
source /opt/ros/indigo/setup.bash
catkin_make install
~~~

To run:

~~~
source /usr/share/gazebo/setup.sh
source ~/catkin_ws/install/share/haptix_gazebo_plugins/setup.sh
gazebo --verbose worlds/arat_test.world
~~~

## Installation instructions: using catkin_make_isolated ##
All the handsim dependencies can be installed in an isolated catkin workspace
 using the following instructions.

First create the workspace folder and clone the source repositories:
~~~
export WS=$HOME/ws/handsim
mkdir -p ${WS}/src
cd ${WS}/src
git clone https://github.com/ros/catkin.git
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
hg clone https://bitbucket.org/osrf/haptix-comm
hg clone https://bitbucket.org/osrf/gazebo
hg clone https://bitbucket.org/osrf/handsim
hg clone https://bitbucket.org/osrf/[haptix_model]
~~~

Then add `package.xml` files for the plain cmake packages.
~~~
PACKAGE="ign-transport" && curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_${PACKAGE}.xml > ${WS}/src/${PACKAGE}/package.xml
PACKAGE="haptix-comm" && curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_${PACKAGE}.xml > ${WS}/src/${PACKAGE}/package.xml
PACKAGE="gazebo" && curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_${PACKAGE}.xml > ${WS}/src/${PACKAGE}/package.xml
~~~

Set the desired branch for each package:
~~~
cd ${WS}/src/gazebo
hg up haptix
~~~

Then do an isolated catkin install build:
~~~
cd ${WS}
./src/catkin/bin/catkin_make_isolated --install
~~~

To run:

~~~
. ${WS}/install_isolated/setup.bash
. ${WS}/install_isolated/share/gazebo/setup.sh
. ${WS}/install_isolated/share/haptix_gazebo_plugins/setup.sh
gazebo --verbose worlds/arat_test.world
~~~

![handsim_startup.png](https://bitbucket.org/repo/rnoXja/images/2595820742-handsim_startup.png)
