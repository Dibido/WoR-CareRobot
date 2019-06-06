cmake_minimum_required(VERSION 2.8)
project(webcam_driver)

SET(GCC_OPENCV_FLAGS "-lopencv_core -lopencv_highgui")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_OPENCV_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	image_transport
	message_generation
	kinematica_msgs
	sensor_msgs
)



## Generate messages in the 'msg' folder
 # add_message_files(
 #   DIRECTORY
 #   msg
 #   FILES ResizedImage.msg Cup.msg
 # )


## Generate added messages and services with any dependencies listed here
 # generate_messages(
 #   DEPENDENCIES
 #   std_msgs
 #   geometry_msgs
 #   sensor_msgs
 # )

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs geometry_msgs)

find_package(Threads REQUIRED)
find_package(sensor_msgs)
find_package(OpenCV 3.2.0 REQUIRED)

include(CTest)
enable_testing()


include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(webcam_driver src/main.cpp src/webcam_driver/WebcamDriver.cpp src/webcam_driver/WebcamPublisher.cpp)
add_dependencies(webcam_driver ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_options(webcam_driver PRIVATE -std=c++14 -Wall -Wextra -Wconversion)
target_link_libraries(webcam_driver ${catkin_LIBRARIES} ${OpenCV_LIBS})

#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!

catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp unitTests/WebcamDriver_test.cpp src/webcam_driver/WebcamDriver.cpp src/webcam_driver/WebcamPublisher.cpp)
target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES} ${OpenCV_LIBS})

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
install(TARGETS webcam_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
