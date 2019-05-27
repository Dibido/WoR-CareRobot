cmake_minimum_required(VERSION 3.7.2 FATAL_ERROR)
project(sim_child)
add_compile_options(-Wall -Wconversion -Wextra -I -isystem -std=c++14)
set(CMAKE_CXX_CPPCHECK "cppcheck" "–enable=all -Wall -Wextra -Wconversion --inline-suppr")

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    gazebo_ros
)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros)
    cmake_minimum_required(VERSION 3.7.2 FATAL_ERROR)
    project(sim_child)
    add_compile_options(-Wall -Wconversion -Wextra -I -isystem -std=c++14)
    set(CMAKE_CXX_CPPCHECK "cppcheck" "–enable=all -Wall -Wextra -Wconversion --inline-suppr")
    
    find_package(gazebo REQUIRED)
    
    find_package(catkin REQUIRED COMPONENTS 
        SYSTEM
        roscpp
        gazebo_ros
    )
    
    catkin_package(
        DEPENDS
        roscpp
        gazebo_ros)
    
    include_directories(
        include
        SYSTEM
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
    
    add_library(model_child SHARED src/child.cpp)
    
    target_link_libraries(model_child ${GAZEBO_LIBRARIES})
    
    include(CTest)
    enable_testing()
    
    #Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
    catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp unitTests/child.cpp)
    
    target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} model_child)
    add_dependencies(${PROJECT_NAME}_unit_tests ${catkin_EXPORTED_TARGETS} ${GAZEBO_EXPORTED_TARGETS})
    
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
include_directories(
    include
    SYSTEM
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_child SHARED src/child.cpp)

target_link_libraries(model_child ${GAZEBO_LIBRARIES})

include(CTest)
enable_testing()

#Only the first gtest.cpp file has a main, the second, third .. fifth should and must not have a main function!
catkin_add_gtest(${PROJECT_NAME}_unit_tests unitTests/gtest.cpp unitTests/child.cpp)

target_link_libraries(${PROJECT_NAME}_unit_tests ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} model_child)
add_dependencies(${PROJECT_NAME}_unit_tests ${catkin_EXPORTED_TARGETS} ${GAZEBO_EXPORTED_TARGETS})

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