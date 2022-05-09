cmake_minimum_required(VERSION 2.8.3)
project(waypoint_saver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_saver
  CATKIN_DEPENDS geometry_msgs roscpp tf nav_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_saver_node src/waypoint_saver_node.cpp src/waypoint_saver.cpp)
target_link_libraries(waypoint_saver_node ${catkin_LIBRARIES})

