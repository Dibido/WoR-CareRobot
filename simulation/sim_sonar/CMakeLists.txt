cmake_minimum_required(VERSION 3.7.2)
project(sim_sonar)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    message_runtime
    roscpp
    sensor_msgs
    std_msgs)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  unit_test_framework
  regex
  signals
)

find_package(Gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()

include_directories(
        include
        SYSTEM
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

add_library(gazebo_ros_sonar SHARED
        src/GazeboRosSonar.cpp)

target_link_libraries(gazebo_ros_sonar
        ${GAZEBO_LIBRARIES}
        ${catkin_LIBRARIES}
        SonarPlugin)

add_library(${PROJECT_NAME}_plugin SHARED
        src/CustomGazeboRosSonar.cpp)

target_link_libraries(${PROJECT_NAME}_plugin
        gazebo_ros_sonar)

catkin_package()

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

