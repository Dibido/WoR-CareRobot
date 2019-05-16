cmake_minimum_required(VERSION 3.7.2)
project(sim_robot)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	rospy
	roscpp
	std_msgs
	visualization_msgs
	message_generation)

find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

link_libraries(gcov)


add_message_files(
  FILES
  commandT.msg
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

target_link_libraries(${PROJECT_NAME}_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES}
		${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_parser_test
	test/al5d_CommandParser_test.cpp
	src/Command.cpp
	src/CommandParser.cpp)
add_dependencies(_parser_test sim_robot_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_parser_test
		${catkin_LIBRARIES}
		${Boost_LIBRARIES})
		
target_link_libraries(${PROJECT_NAME}_parser_test gcov ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_options(${PROJECT_NAME}_parser_test PUBLIC -g -O0 --coverage)

add_executable(${PROJECT_NAME}_joint_test
	test/JointController_test.cpp
	src/JointController.cpp)
add_dependencies(_joint_test sim_robot_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_joint_test
		${catkin_LIBRARIES}
		${Boost_LIBRARIES})
		
target_link_libraries(${PROJECT_NAME}_joint_test gcov ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_options(${PROJECT_NAME}_joint_test PUBLIC -g -O0 --coverage)
