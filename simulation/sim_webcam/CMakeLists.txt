cmake_minimum_required(VERSION 3.7.2)
project(sim_webcam)
add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
        gazebo_msgs
        gazebo_plugins
        gazebo_ros
        message_runtime
        roscpp
        sensor_msgs
        std_msgs
	sensor_interfaces)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  unit_test_framework
  regex
  signals
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        include
        SYSTEM # everything after system is seen as system include (no compiler warnings)
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})
		
catkin_package(
    DEPENDS
    roscpp
    gazebo_ros
    sensor_interfaces)

add_library(${PROJECT_NAME}_plugin SHARED
src/WebcamPlugin.cpp)

add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})

		
#add_dependencies(${PROJECT_NAME}_plugin sensor_interfaces)

target_link_libraries(${PROJECT_NAME}_plugin
        ${GAZEBO_LIBRARIES}
        ${catkin_LIBRARIES}
        )
