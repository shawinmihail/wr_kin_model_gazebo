cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(kin_plugin)
add_definitions(-std=c++11)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package(gazebo REQUIRED)
find_package(
    catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs wr_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        sensor_msgs
    DEPENDS
        ${EIGEN_PACKAGE}
)

include_directories(
include
../../models/surf/generated
${GAZEBO_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(surf_lib SHARED IMPORTED)
set_property(TARGET surf_lib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../models/surf/generated/surf_fcn.so)

add_library(wr_kin_plugin SHARED src/gazeboWrKinPlugin.cpp src/curve_math.cpp)
target_link_libraries(wr_kin_plugin surf_lib ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(wr_spawn_plugin SHARED src/gazeboWrSpawnRoute.cpp src/curve_math.cpp)
target_link_libraries(wr_spawn_plugin surf_lib ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
