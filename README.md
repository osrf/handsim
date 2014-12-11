# HAPTIX Simulator #

## Installation instructions: ##
To compile, first install gazebo from haptix branch, ign-transport and haptix-comm from default branch.  Next:

~~~
hg clone https://bitbucket.org/osrf/handsim
cd handsim 
mkdir build
cd build
cmake ../
make
sudo make install
~~~

To run:

~~~
gazebo worlds/arat.world
~~~

![handsim_startup.png](https://bitbucket.org/repo/rnoXja/images/2595820742-handsim_startup.png)
