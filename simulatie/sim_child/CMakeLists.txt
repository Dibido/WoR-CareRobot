cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sim_child)

catkin_package(
    DEPENDS
    roscpp
    gazebo
    gazebo_ros)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    gazebo
    gazebo_ros
)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_child SHARED src/child.cpp)

target_link_libraries(model_child ${GAZEBO_LIBRARIES})