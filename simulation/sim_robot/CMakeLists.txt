cmake_minimum_required(VERSION 3.7.2)
project(sim_robot)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)
set(CMAKE_CXX_CPPCHECK "cppcheck" "–enable=all -Wall -Wextra -Wconversion --inline-suppr")
find_package(catkin REQUIRED COMPONENTS
	message_generation
  message_runtime
  robotcontroller_msgs
	gazebo_ros
	rospy
	roscpp
	std_msgs
	visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  unit_test_framework
  regex
  signals
)

find_package(gazebo REQUIRED)

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()

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
  robotcontroller_msgs
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


#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp src/Command.cpp src/CommandParser.cpp)
target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES})
  
if(CODE_COVERAGE)
  #We want to measure the codecoverage with a Debug build
  set(CMAKE_BUILD_TYPE Debug)


  include(../../cmake/CodeCoverage.cmake)
    
  #Files which will be excluded for the codecoverage
  set(COVERAGE_LCOV_EXCLUDES
    '*/gtest/*'
    '*/boost/*'
    '*/gazebo/*'
    '*/opt/*'
    '*/usr/*'
    '*/ignition/*'
    '*/test/*'
    '/*/c++/*'
    '*/*.py'
    '*/unitTests/*.cpp'
    '*/*main.cpp'
  )

  APPEND_COVERAGE_COMPILER_FLAGS()


  SETUP_TARGET_FOR_COVERAGE_LCOV(
          NAME ${PROJECT_NAME}_coverage
          EXECUTABLE ctest -j ${n_cores}
          DEPENDENCIES
          ${PROJECT_NAME}_unit_tests
             )   
endif()
