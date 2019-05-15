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

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  unit_test_framework
  regex
  signals
)

find_package(Gazebo REQUIRED)

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()

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

