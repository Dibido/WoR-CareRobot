cmake_minimum_required(VERSION 2.8.3)
project(rplidar_hardware)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  sensor_interfaces
)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(node src/rplidar_hardware/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(node ${catkin_LIBRARIES})

add_dependencies(node ${catkin_EXPORTED_TARGETS})

install(TARGETS node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
