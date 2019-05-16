cmake_minimum_required(VERSION 3.7.2)
project(sim_world)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros)

find_package(gazebo REQUIRED)

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS})
