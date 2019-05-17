cmake_minimum_required(VERSION 3.7.2)
project(sensor_interfaces)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  genmsg
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   WebcamData.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs message_runtime
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)
