cmake_minimum_required(VERSION 2.8.3)
project(sim_robot)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp
	std_msgs
	visualization_msgs)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  unit_test_framework
)

find_package(gazebo REQUIRED)

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()

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

#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp)
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
