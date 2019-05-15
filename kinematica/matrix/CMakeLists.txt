cmake_minimum_required(VERSION 3.7.2)
project(matrix)

add_compile_options(-std=c++14 -Wall -Wextra -Wconversion)

find_package(catkin REQUIRED COMPONENTS
)

## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIR}
)

add_executable(UnitTest unittest/Main.cpp unittest/Matrix_test.cpp)
target_link_libraries(UnitTest
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)
target_compile_options(UnitTest PRIVATE -std=c++14 -Wall -Wextra -Wconversion)
