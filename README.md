
# mrs_formation_aquatic_sim

The repository presents a implementation in ROS, of an approach using Potential Field Approach
with attractive/repulsive behaviours Formation Control techniques with Multi-Robot System (MRS), using Gazebo Simulation with emulated aquatic environment, as a testbed  to deploy a team of **U**nmanned **S**urface **V**ehicles (USVs).

The simulation plugins and the USV controllers and model descriptions, were based on the work of [Brian Bingham](https://github.com/bsb808/usv_gazebo_plugins).  

## Requirements
The presented packages in this repository were developed using *ROS Kinetic* , running in [Ubuntu 16.04.5 LTS](http://releases.ubuntu.com/16.04/) (Xenial). Furthermore to proceed with the remaining simulation environment, it is necessary to have install in your machine the following:    

- [git](https://git-scm.com/downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) 
- [Gazebo 7](http://gazebosim.org): Usually available after installation of ROS **Desktop-Full Install**.

After ROS installation do **not forget to configure** your [workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). To test if your Gazebo installation simple run in a terminal:
```
roscore &
rosrun gazebo_ros gazebo
```

Moving on to the necessary ROS packages, 
```
sudo apt-get install ros-kinetic-hector-gazebo-plugins  ros-kinetic-geographic-info ros-kinetic-tf2-geometry-msgs ros-kinetic-move-base ros-kinetic-interactive-marker-twist-server python-pip
```

Then after get the following repos into your ROS workspace (by default *~/catkin_ws/src/*) using git commands:
```
git clone https://github.com/kf/kingfisher.git
git clone https://github.com/bsb808/geonav_transform
git clone https://github.com/bsb808/kingfisher_control.git
git clone https://github.com/bsb808/kingfisher_utils.git
git clone https://github.com/aaraujo11/mrs_formation_aquatic_sim.git
```

Build the downloaded ROS packages:
```
cd ~/catkin_ws
catkin_make
```

In summary the downloaded ROS packages:
- **kingfisher** - provides the URDF description of the shape and size of the USV, in this case of the Heron Kingfisher from Clearpath Robotics;
- **hector_gazebo_plugins** -  Used to simulate sensor measurements from the Gazebo plugins for the IMU and GPS;
- **kingfisher_control** - Used for the USV PID controller;
- **geonav_transform** - This package transforms UTM GPS coordinates, into odometry messages. 
- **kingfisher_utils** - Tool to convert *cmd_drive* into ROS Twist commands *cmd_vel*.

For this work implementation we have:

- **kingfisher_navigation** - Package dedicated to navigation purposes, using navigation stack.
- **usv_gazebo_plugins** - Sets up the simulated USV with the USV dynamics;
- **kingfisher_gazebo** - Set of launch files to simplify the execution of all ROS nodes and tools;
- **mrs_formation_control** - Contain the ROS node with the formation control approach.

To install the application responsible to use Python controller, in any folder besides (~/catkin_ws/src):

```
git clone https://github.com/bsb808/pypid.git
python setup.py develop --user
sudo python setup.py develop
```


# Quick Start

To launch one robot with prefix (robot_0) and move_base:
```
roslaunch kingfisher_gazebo one_base_gazebo.launch 
roslaunch kingfisher_navigation one_nav.launch
```

To launch four robots with prefix (robot_*n*) and move_base, and MRS formation control node:
```
roslaunch kingfisher_gazebo multi_base_gazebo.launch 
roslaunch kingfisher_navigation multi_nav.launch
rosrun mrs_formation_control mrs_formation_control

```

Multi-robot launch file architecture:


If you have all dependencies and ROS packages installed, you should see something similar to:



To send a new goal to the team of robots, use the **2D Nav Goal** of Rviz.

