cmake_minimum_required(VERSION 2.8.3)
project(sonar_interpreter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -Wextra -Wconversion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here


include_directories(
  include
  SYSTEM # everything after system is seen as system include (no compiler warnings)
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  ObjectSpeed.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS include
)

add_executable(${PROJECT_NAME}_node src/sonar_interpreter_node.cpp src/SonarInterpreter.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})