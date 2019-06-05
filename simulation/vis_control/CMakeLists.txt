cmake_minimum_required(VERSION 3.7.2)
project(vis_control)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(
        catkin
        REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        tf
        visualization_msgs
        message_generation
        path_planner
        vision)

find_package(
        Boost
        REQUIRED COMPONENTS
        system
        unit_test_framework)
        
link_libraries(gcov)


include_directories(
        include
        SYSTEM # everything after system is seen as system include (no compiler warnings)
        ${catkin_INCLUDE_DIRS})

catkin_package(
        CATKIN_DEPENDS
        roscpp
        message_runtime)

# Collect all src files
file(GLOB vis_control_src_files src/*.cpp)

# Exclude the src file with main()
list(REMOVE_ITEM vis_control_src_files ${CMAKE_CURRENT_SOURCE_DIR}/src/visualization_control_node.cpp)

# Create the node executable
add_executable(${PROJECT_NAME}_node
        ${vis_control_src_files}
        src/visualization_control_node.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES})

add_dependencies(
        ${PROJECT_NAME}_node
        ${catkin_EXPORTED_TARGETS})

# Collect all cpp files in the /test folder
# This list also contains the main()
file(GLOB vis_control_test_files test/*.cpp)

# Create the unittest executable
add_executable(${PROJECT_NAME}_test
        ${vis_control_src_files}
        ${vis_control_test_files})

target_link_libraries(
        ${PROJECT_NAME}_test
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})


add_dependencies(
        ${PROJECT_NAME}_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

add_dependencies(
        ${PROJECT_NAME}_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_test gcov ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_options(${PROJECT_NAME}_test PUBLIC -g -O0 --coverage)
