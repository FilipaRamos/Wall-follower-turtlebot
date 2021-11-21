# Designing a purely reactive wall follower robot

Hello! Welcome to this repository. You can find here the turtlebot3_wall package which enables a reactive wall follower turtlebot on a Gazebo simulation.

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