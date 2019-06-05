cmake_minimum_required(VERSION 3.7.2)
project(sim_torque)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
    message_generation
		roslaunch)

find_package(Boost REQUIRED COMPONENTS
  system
  unit_test_framework
)

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()

find_package(gazebo REQUIRED)

link_libraries(gcov)

include_directories(include
	SYSTEM # everything after system is seen as system include (no compiler warnings)
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS})

link_directories(SYSTEM
	${GAZEBO_LIBRARY_DIRS})

add_message_files(
  FILES
  collision_behavior.msg)

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
	gazebo_ros
	roscpp
	message_runtime
	std_msgs
)

add_library(${PROJECT_NAME}_plugin
    src/CollisionBehaviorData.cpp
    src/CollisionBehaviorSubscriber.cpp
    )

add_dependencies(${PROJECT_NAME}_plugin
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES})
