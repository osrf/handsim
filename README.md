# HAPTIX Simulator #

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

to run:

~~~
cd ~/catkin_ws
source install/share/haptix_gazebo_plugins/setup.sh

gazebo --verbose worlds/arat_test.world
~~~

