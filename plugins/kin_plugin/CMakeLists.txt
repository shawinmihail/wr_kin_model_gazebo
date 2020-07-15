add_definitions(-std=c++11)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(include eigen ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(wr_kin_plugin SHARED src/gazeboWrKinPlugin.cpp src/curve_math.cpp)
target_link_libraries(wr_kin_plugin ${GAZEBO_LIBRARIES})

add_library(wr_spawn_plugin SHARED src/gazeboWrSpawnRoute.cpp src/curve_math.cpp)
target_link_libraries(wr_spawn_plugin ${GAZEBO_LIBRARIES})
