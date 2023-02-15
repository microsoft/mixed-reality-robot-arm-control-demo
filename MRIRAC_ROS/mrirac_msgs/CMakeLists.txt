cmake_minimum_required(VERSION 3.0.2)
project(mrirac_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  moveit_msgs
)

add_message_files(
  FILES
  MeshObstacle.msg
  MeshObstacles.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  TrajectoryPlan.srv
  StartPickAndPlace.srv
  PlanPickAndPlace.srv
)

add_action_files(
  DIRECTORY action
  FILES 
  PoseCorrection.action
  PickAndPlace.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs geometry_msgs moveit_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
