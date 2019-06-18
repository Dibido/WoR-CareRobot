cmake_minimum_required(VERSION 3.7.2)
project(kinect_cup_detector)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)
set(CMAKE_CXX_CPPCHECK "cppcheck" "–enable=all -Wall -Wextra -Wconversion --inline-suppr")
SET(GCC_OPENCV_FLAGS "-lopencv_core -lopencv_highgui")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_OPENCV_FLAGS}")

find_package(OpenCV 3.2.0 REQUIRED)
include_directories(include)

find_package(catkin REQUIRED COMPONENTS 
  message_generation  
  roscpp
  std_msgs
  sensor_interfaces
  cv_bridge
  image_transport
  kinematica_msgs
  environment_controller
)

find_package(Boost REQUIRED COMPONENTS
system
filesystem
unit_test_framework
)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS roscpp std_msgs message_runtime sensor_interfaces environment_controller cv_bridge image_transport)

#Enable CTest which is used for the codecoverage
include(CTest)
enable_testing()

include_directories(
  include
  SYSTEM # everything after system is seen as system include (no compiler warnings)
  ${catkin_INCLUDE_DIRS})

  add_executable(kinect_cup_detector src/${PROJECT_NAME}/main.cpp)

  add_dependencies(kinect_cup_detector ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(kinect_cup_detector ${catkin_LIBRARIES})
  
  target_compile_options(kinect_cup_detector PRIVATE -std=c++14 -Wall -Wextra -Wconversion)
  target_link_libraries(kinect_cup_detector ${OpenCV_LIBS} ${freenect2_LIBRARIES})
  
  #Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
  catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp)
  
  add_dependencies(${PROJECT_NAME}_unit_tests ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_main)
  
  target_link_libraries(${PROJECT_NAME}_unit_tests
   ${catkin_LIBRARIES})
  
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