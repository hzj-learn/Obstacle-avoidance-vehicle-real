cmake_minimum_required(VERSION 2.8.3)
project(waypoint_loader)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_loader
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_loader_node src/waypoint_loader_node.cpp src/waypoint_loader.cpp)
target_link_libraries(waypoint_loader_node ${catkin_LIBRARIES})

