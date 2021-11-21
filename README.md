# Designing a purely reactive wall follower robot

Hello! Welcome to this repository. You can find here the turtlebot3_wall package which enables a reactive wall follower turtlebot running on Gazebo simulation.

## Directory Structure

This package is organised as recommended by ROS documentation. The structure is as follows:

```
turtlebot3_wall
│   README.md
│   package.xml
|   LICENSE
|   CMakeLists.txt
|   .gitignore
|   
└───eval
|   │   evaluation.py
|   │   range_errors.png
|   │   range_errors.txt
|   │   
|  
└───include
│   │
│   └───turtlebot3_wall
│   
└───launch
|   │   turtlebot3_wall_follower.launch
|   │   wall.launch
|   │   
|  
└───models
|   │   bwall-round.stl
|   │   bwall-sharp.stl
|   │   bwall-stretch.stl
|   │   bwall-verbana.stl
|   │   bwall.stl
|   │   
│   
└───rviz
|   │   turtlebot3_gazebo_rviz.rviz
|   │   
│   
└───src
|   │   wallFollower.cpp
|   │   wallFollower.h
|   │   evaluation.cpp
|   │   evaluation.h
|   │   
│   
└───worlds
|   │   bwall-round.world
|   │   bwall-sharp.world
|   │   bwall-stretch.world
|   │   bwall-verbana.world
|   │   bwall.world
|   │   
|
```

## Dependencies

The only dependencies of the package are entirely related to ROS, with its installation and some turtlebot3 packages needed for execution.

- ROS
- turtlebot3 packages
    - turtlebot3_gazebo
        - simulation environment
    - turtlebot3_bringup
        - turtlebot sensor modelling
    - turtlebot3_description
        - turtlebot simulation model (URDF)

This project was tested on a system with the following characteristics:
- Ubuntu 20.04.3 LTS
- ROS Noetic

# Installation

In order to meet dependencies, first install the turtlebot package from your preferred package manager. The showcased instructions are compatible with any Debian based distribution.

> sudo apt install ros-<ros_distro>-turtlebot3

Download the turtlebot3_wall package from github or clone this repository. Place the turtlebot3_wall folder under ~/catkin_ws/src. Then the package needs to be built.

> cd ~/catkin_ws && catkin_make

And we are done!

# Execution

All the needed modules are conveniently loaded directly through a launch file. As such, you simply need to run:

> roslaunch turtlebot3_wall turtlebot3_wall_follower.launch

This will launch the default world. 3 other worlds are available for testing. To swith worlds, simply add the argument to the command with the name of the desired world. For example, we can open the sharp corners B wall world by calling:

> roslaunch turtlebot3_wall turtlebot3_wall_follower.launch world:=bwall-sharp

The robot will be spawned on the default location (along the straight segment) on the inside of the B-shaped wall unless we specify otherwise by making the inside parameter false:

> roslaunch turtlebot3_wall turtlebot3_wall_follower.launch world:=bwall-sharp inside:=false