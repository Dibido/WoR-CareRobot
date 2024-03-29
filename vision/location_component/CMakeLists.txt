cmake_minimum_required(VERSION 2.8)
project(location_component)

SET(GCC_OPENCV_FLAGS "-lopencv_core -lopencv_highgui")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_OPENCV_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	image_transport
	message_generation
	kinematica_msgs
	sensor_msgs
	environment_controller
	sensor_interfaces
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

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs geometry_msgs environment_controller sensor_interfaces)

find_package(Threads REQUIRED)
find_package(sensor_msgs)
find_package(OpenCV 3.2.0 REQUIRED)

include(CTest)
enable_testing()


include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(location_component 
                src/main.cpp 
                src/location_component/DetectAGV.cpp 
                src/location_component/CupScanner.cpp 
                src/location_component/PosCalculation.cpp 
                src/location_component/RosServiceCup.cpp 
                src/location_component/AGV.cpp 
                src/location_component/AGVSubscriber.cpp 
                src/location_component/FrameCalibration.cpp 
                src/location_component/LocationComponent.cpp 
                src/location_component/AGVFrameCalibration.cpp 
                src/location_component/CupDetectionCalibration.cpp
                src/location_component/GoalSubscriberLocationComp.cpp
                )
                
add_dependencies(location_component ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_options(location_component PRIVATE -std=c++14 -Wall -Wextra -Wconversion)
target_link_libraries(location_component ${catkin_LIBRARIES} ${OpenCV_LIBS})

#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!

catkin_add_gtest(${PROJECT_NAME}_unit_tests 
                  unitTests/gtest.cpp 
                  unitTests/DetectAGV_test.cpp 
                  unitTests/AGVSubscriber_test.cpp 
                  unitTests/ValueGuardTest.cpp 
                  unitTests/AGV_test.cpp 
                  unitTests/ValueTest.cpp 
                  unitTests/IAGVProvider_test.cpp 
                  unitTests/FrameCalibration_test.cpp 
                  unitTests/CupScanner_test.cpp 
                  unitTests/PosCalculation_test.cpp 
                  unitTests/ImagePath.cpp 
                  src/location_component/DetectAGV.cpp 
                  src/location_component/CupScanner.cpp 
                  src/location_component/PosCalculation.cpp 
                  src/location_component/RosServiceCup.cpp 
                  src/location_component/AGV.cpp 
                  src/location_component/AGVSubscriber.cpp 
                  src/location_component/LocationComponent.cpp 
                  src/location_component/FrameCalibration.cpp 
                  src/location_component/AGVFrameCalibration.cpp 
                  src/location_component/CupDetectionCalibration.cpp
                  src/location_component/GoalSubscriberLocationComp.cpp
                  )

target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME}_unit_tests ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

if(CODE_COVERAGE)
  #We want to measure the codecoverage with a Debug build
  set(CMAKE_BUILD_TYPE Debug)

  include(../../cmake/CodeCoverage.cmake)

  #Files which will be excluded for the codecoverage
  set(COVERAGE_LCOV_EXCLUDES
    '*/devel/*'
    '*/boost/*'
    '*/opencv2/*'
    '*/gcc/*'
    '*/melodic/*'
    '*/kinematica/*'
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
install(TARGETS location_component
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
