cmake_minimum_required(VERSION 2.8)
project(location_component)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY 
   msg
   FILES ResizedImage.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )


catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(Threads REQUIRED)
find_package(sensor_msgs)
find_package(OpenCV)

include(CTest)
enable_testing()


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

# add the publisher example
add_executable(DetectCup src/main.cpp src/DetectAGV.cpp src/CupScanner.cpp)
add_dependencies(DetectCup ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_options(DetectCup PRIVATE -std=c++14 -Wall -Wextra -Wconversion)
target_link_libraries(DetectCup ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp unitTests/DetectAGV_test.cpp src/DetectAGV.cpp src/CupScanner.cpp)
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

# Mark executables and/or libraries for installation
install(TARGETS DetectCup
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
