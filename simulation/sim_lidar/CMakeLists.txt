cmake_minimum_required(VERSION 3.7.2)
project(sim_lidar)
add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    gazebo_ros
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

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        include
        SYSTEM # everything after system is seen as system include (no compiler warnings)
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugin SHARED
        src/LidarPlugin.cpp)

target_link_libraries(${PROJECT_NAME}_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        RayPlugin)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros)

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