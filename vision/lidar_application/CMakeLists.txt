cmake_minimum_required(VERSION 3.7.2)
project(lidar_application)
add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_interfaces
  kinematica_msgs
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
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp
 DEPENDS roscpp std_msgs message_runtime sensor_interfaces kinematica_msgs
)

include_directories(
  include/${PROJECT_NAME}
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)



#Main application
add_executable(${PROJECT_NAME}_main src/${PROJECT_NAME}/main.cpp src/${PROJECT_NAME}/ObjectDetection.cpp src/${PROJECT_NAME}/DataHandler.cpp src/${PROJECT_NAME}/LidarData.cpp)

add_dependencies(${PROJECT_NAME}_main ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES})


#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp unitTests/ObjectDetection_Unittests.cpp unitTests/DataHandler_Unittests.cpp unitTests/LidarData_Unittests.cpp src/${PROJECT_NAME}/ObjectDetection.cpp src/${PROJECT_NAME}/DataHandler.cpp src/${PROJECT_NAME}/LidarData.cpp)
target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES})
  
if(CODE_COVERAGE)
  #We want to measure the codecoverage with a Debug build
  set(CMAKE_BUILD_TYPE Debug)


  include(../../cmake/CodeCoverage.cmake)
    
  #Files which will be excluded for the codecoverage
  set(COVERAGE_LCOV_EXCLUDES
    '*/gtest/*'
    '/*/c++/*'
    '*/boost/*'
    '*/ros/*'
    '*/geometry_msgs/*'
    '*/kinematica_msgs'
    '*/sensor_interfaces'
    '*/include/*'
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
