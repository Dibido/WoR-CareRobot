cmake_minimum_required(VERSION 2.8.3)
project(sim_kinect)
add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
        gazebo_msgs
        gazebo_plugins
        gazebo_ros
        message_runtime
        roscpp
        sensor_msgs
        std_msgs)

find_package(Gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        include
        SYSTEM # everything after system is seen as system include (no compiler warnings)
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugin SHARED
        src/KinectPlugin.cpp)

target_link_libraries(${PROJECT_NAME}_plugin
        ${GAZEBO_LIBRARIES}
        ${catkin_LIBRARIES}
        DepthCameraPlugin)

catkin_package()
