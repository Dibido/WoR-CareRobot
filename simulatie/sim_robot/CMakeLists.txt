cmake_minimum_required(VERSION 2.8.3)
project(sim_robot)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp
	std_msgs
	visualization_msgs)

find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

link_libraries(gcov)

include_directories(
		include
		SYSTEM # everything after system is seen as system include (no compiler warnings)
		${catkin_INCLUDE_DIRS}
		${GAZEBO_INCLUDE_DIRS}
		${Boost_LIBRARY_DIRS})

catkin_package(
	CATKIN_DEPENDS
	gazebo_ros
	roscpp
	visualization_msgs)

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

target_link_libraries(${PROJECT_NAME}_parser_test
		${catkin_LIBRARIES}
		${Boost_LIBRARIES})
		
target_link_libraries(${PROJECT_NAME}_parser_test gcov ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_options(${PROJECT_NAME}_parser_test PUBLIC -g -O0 --coverage)

add_executable(${PROJECT_NAME}_joint_test
	test/JointController_test.cpp
	src/JointController.cpp)

target_link_libraries(${PROJECT_NAME}_joint_test
		${catkin_LIBRARIES}
		${Boost_LIBRARIES})
		
target_link_libraries(${PROJECT_NAME}_joint_test gcov ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_options(${PROJECT_NAME}_joint_test PUBLIC -g -O0 --coverage)
