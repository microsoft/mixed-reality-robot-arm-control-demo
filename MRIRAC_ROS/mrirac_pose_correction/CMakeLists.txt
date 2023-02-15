cmake_minimum_required(VERSION 3.0.2)
project(mrirac_pose_correction)

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
  mrirac_lib
  moveit_ros_planning_interface
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(pose_correction_node src/pose_correction_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pose_correction_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pose_correction_node
  ${catkin_LIBRARIES}
)
