cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sim_child)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_child SHARED src/child.cpp)
target_link_libraries(model_child ${GAZEBO_LIBRARIES})