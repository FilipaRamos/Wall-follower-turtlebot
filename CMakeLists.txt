cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_wall)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wallf src/wallFollower.cpp src/wallFollower.h src/evaluation.cpp src/evaluation.h)
target_link_libraries(wallf ${catkin_LIBRARIES})