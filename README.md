yaskawa_ethercat
----------------

# Overview

This is an implementation of a ROS-wrapped EtherCAT Master controller specifically for [Yaskawa Sigma 7 Servopacks](https://www.yaskawa.com/products/motion/sigma-7-servo-products/servopacks/sgd7s-ethercat). It may work for other CiA 402 motion controllers, however this is untested. 

The controller is built using Simple Open Ethercat Master (SOEM) maintained by [Open EtherCAT Society](https://openethercatsociety.github.io/) and can be downloaded from [here](https://github.com/OpenEtherCATsociety/SOEM)

# Installation procedures
## ROS

To install ROS, please refer to the original ROS installation documentation at [http://www.ros.org/install/](http://www.ros.org/install/)

We have developed and tested this package in Ubuntu 20.04 and ROS Noetic.

## SOEM

```sh
$ git clone https://github.com/OpenEtherCATsociety/SOEM.git
$ mkdir build
$ cd build
$ cmake ..
$ make
```

After installing SOEM, you can run a test to ensure EtherCAT communication is working. First, find you network interface card (NIC) name

```sh
$ ip link show
```

And then run the test using the desired interface (if you have more than one):

```sh
$ simple_test[ifname]
```

## This ROS package
### Building
At this point you should be all set to start using this ROS package to control your devices. If you haven't yet cloned it, do so into your [catkin configured workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). 
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/SchultzKyle/yaskawa-ros-ethercat-control.git
$ cd ~/catkin_ws
$ catkin build
```

# Usage
Since the program must be run as root to access socket communication, this package does not currently use launch files. There are ways to use a launch file (e.g. [here](https://answers.ros.org/question/165246/launch-node-with-root-permissions/)), but these may pose security risks. You can run the EtherCAT master controller directly with:

```sh
$ sudo chrt 90 ./devel/lib/ethercat_master/run <ifname>
```

This controller supports the the synchronous drive modes: Position (CSP), Velocity (CSV), and Torque (CST). The default mode is CSP but this can be changed dynamically using modes entry in the message, where CSP = 7, CSV = 8, CST = 9.  

Motor specific parameters of gear ratio and rated torque must be set in common.h. Optional torque, velocity, and position limits can also be added.

