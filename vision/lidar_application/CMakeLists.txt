cmake_minimum_required(VERSION 2.8.3)
project(lidar_application)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  unit_test_framework
)

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Main application
add_executable(${PROJECT_NAME}_main src/main.cpp src/ObjectDetection.cpp)

#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp unitTests/gtest2.cpp src/ObjectDetection.cpp)
target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES})
  
if(CODE_COVERAGE)
  #We want to measure the codecoverage with a Debug build
  set(CMAKE_BUILD_TYPE Debug)


  include(../../cmake/CodeCoverage.cmake)
    
  #Files which will be excluded for the codecoverage
  set(COVERAGE_LCOV_EXCLUDES
    '*/gtest/*'
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
