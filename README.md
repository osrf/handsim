# HAPTIX Simulator #

## Dependencies

This repository requires `gazebo`, `ign-transport` and `haptix-comm` from default branch.

Additionally, make sure to have `libusb` installed:

    sudo apt-get install libusb-1.0-0-dev

## Installation instructions: ##

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
