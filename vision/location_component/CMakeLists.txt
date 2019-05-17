cmake_minimum_required(VERSION 3.7.2)
project(location_component)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  genmsg
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Threads REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(DetectCup src/Main.cpp src/CupScanner.cpp src/CupScanner.cpp)
target_compile_options(DetectCup PRIVATE -std=c++14 -Wall -Wextra -Wconversion)
target_link_libraries(DetectCup ${OpenCV_LIBS} ${ZBAR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
