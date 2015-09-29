cmake_minimum_required(VERSION 2.8.3)
project(odom_disable_safety)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(odom_disable_safety_node src/odom_disable_safety_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odom_disable_safety_node
  ${catkin_LIBRARIES}
)
