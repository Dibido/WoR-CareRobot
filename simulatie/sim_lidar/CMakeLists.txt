cmake_minimum_required(VERSION 2.8.3)
project(sim_lidar)
add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    gazebo_ros
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        include
        SYSTEM # everything after system is seen as system include (no compiler warnings)
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugin SHARED
        src/LidarPlugin.cpp)

target_link_libraries(${PROJECT_NAME}_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        RayPlugin)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros)