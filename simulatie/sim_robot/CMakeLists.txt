cmake_minimum_required(VERSION 3.7.2)
project(sim_robot)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)
set(CMAKE_CXX_CPPCHECK "cppcheck" "–enable=all --inline-suppr")
find_package(catkin REQUIRED COMPONENTS
	message_generation
	message_runtime
	gazebo_ros
	rospy
	roscpp
	std_msgs
	visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

link_libraries(gcov)


add_message_files(
  FILES
  commands.msg
  stopCommand.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	gazebo_ros
	roscpp
	visualization_msgs
	message_runtime
	std_msgs
)
include_directories(
		include
		SYSTEM # everything after system is seen as system include (no compiler warnings)
		${catkin_INCLUDE_DIRS}
		${GAZEBO_INCLUDE_DIRS}
		${Boost_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_plugin
	src/robot_controller_plugin.cpp
	src/Command.cpp
	src/CommandParser.cpp
	src/JointController.cpp)
add_dependencies(${PROJECT_NAME}_plugin sim_robot_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES}
		${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_publisher
	src/publisher.cpp)
add_dependencies(${PROJECT_NAME}_publisher sim_robot_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_publisher
		${catkin_LIBRARIES}
		${Boost_LIBRARIES})



