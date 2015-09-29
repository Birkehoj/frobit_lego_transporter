cmake_minimum_required(VERSION 2.8.3)
project(cross_bridge_mission)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(cross_bridge_mission_node src/cross_bridge_mission_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cross_bridge_mission_node
  ${catkin_LIBRARIES}
)
