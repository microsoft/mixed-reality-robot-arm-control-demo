cmake_minimum_required(VERSION 3.0.2)
project(mrirac_pick_and_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  kinova_msgs
  mrirac_msgs
  moveit_ros_planning_interface
  mrirac_lib
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(pick_and_place_node src/pick_and_place_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pick_and_place_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pick_and_place_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pick_and_place_node
  ${catkin_LIBRARIES}
)
