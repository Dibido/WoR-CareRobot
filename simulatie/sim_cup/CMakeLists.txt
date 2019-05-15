cmake_minimum_required(VERSION 2.8.3)
project(sim_cup)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		message_generation
		geometry_msgs
		roslaunch)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
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
	cup_info.msg)


generate_messages(
	DEPENDENCIES
	geometry_msgs)


catkin_package()

add_library(${PROJECT_NAME}_plugin
		src/CupPlugin.cpp)

add_dependencies(${PROJECT_NAME}_plugin
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES})

add_executable(${PROJECT_NAME}_test
		test/cup_plugin_test.cpp)

target_link_libraries(${PROJECT_NAME}_test
		${PROJECT_NAME}_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_test gcov ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_options(${PROJECT_NAME}_test PUBLIC -g -O0 --coverage)

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
