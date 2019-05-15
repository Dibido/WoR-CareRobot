cmake_minimum_required(VERSION 2.8.3)
project(sim_sonar)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(Gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    message_runtime
    roscpp
    sensor_msgs
    std_msgs)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        include
        SYSTEM
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

add_library(gazebo_ros_sonar SHARED
        src/GazeboRosSonar.cpp)

target_link_libraries(gazebo_ros_sonar
        ${GAZEBO_LIBRARIES}
        ${catkin_LIBRARIES}
        SonarPlugin)

add_library(${PROJECT_NAME}_plugin SHARED
        src/CustomGazeboRosSonar.cpp)

target_link_libraries(${PROJECT_NAME}_plugin
        gazebo_ros_sonar)

catkin_package()

