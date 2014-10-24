# HAPTIX Simulator #

To compile, first install gazebo from haptix branch, ign-transport and haptix_comm from default branch.  Next:

~~~
mkdir ~/catkin_ws
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
hg clone http://bitbucket.org/osrf/handsim
hg clone http://bitbucket.org/osrf/[haptix_model]
cd ~/catkin
source /opt/ros/indigo/setup.bash
catkin_make install
~~~

To run:

~~~
cd ~/catkin_ws
source install/share/haptix_gazebo_plugins/setup.sh
gazebo --verbose worlds/arat_test.world
~~~

![handsim_startup.png](https://bitbucket.org/repo/rnoXja/images/2595820742-handsim_startup.png)