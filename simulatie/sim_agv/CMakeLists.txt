cmake_minimum_required(VERSION 2.8.3)
project(sim_agv)

 add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp
	std_msgs
	message_generation
)

find_package(Gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
		include
		SYSTEM # everything after system is seen as system include (no compiler warnings)
		${GAZEBO_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_message_files(
  FILES
  agv_path.msg
  agv_speed.msg)

generate_messages(
  DEPENDENCIES)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_agv
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_library(${PROJECT_NAME}_plugin SHARED src/automated_guided_vehicle_plugin.cpp)

target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

