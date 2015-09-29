cmake_minimum_required(VERSION 2.8.3)
project(drive_straight)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  msgs
  roscpp
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ executable
add_executable(drive_straight_node src/drive_straight_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(drive_straight_node
  ${catkin_LIBRARIES}
)

