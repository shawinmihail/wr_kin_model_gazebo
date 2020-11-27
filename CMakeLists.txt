cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(kin_model)
add_definitions(-std=c++11)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs wr_msgs)
find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

#catkin_python_setup()
catkin_package(
    INCLUDE_DIRS
        plugins/kin_plugin/include
        plugins/surf_lib/include
    CATKIN_DEPENDS
        sensor_msgs
    DEPENDS
        ${EIGEN_PACKAGE}
)

include_directories(
plugins/kin_plugin/include
plugins/surf_lib/include
models/surf/generated
${GAZEBO_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -Wall")

add_library(surf_lib SHARED plugins/surf_lib/src/surf_fcn.cpp plugins/surf_lib/src/surf_fcn_initialize.cpp plugins/surf_lib/src/surf_fcn_terminate.cpp)

add_library(wr_kin_plugin SHARED plugins/kin_plugin/src/gazeboWrKinPlugin.cpp plugins/kin_plugin/src/curve_math.cpp)
target_link_libraries(wr_kin_plugin surf_lib ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(wr_spawn_plugin SHARED plugins/kin_plugin/src/gazeboWrSpawnRoute.cpp plugins/kin_plugin/src/curve_math.cpp)
target_link_libraries(wr_spawn_plugin surf_lib ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_executable(gazebo_spawn_node plugins/kin_plugin/src/gazebo_spawn_node.cpp)
target_link_libraries(gazebo_spawn_node ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(surf_lib_python MODULE plugins/surf_lib_python/python_wrapper.cpp)
target_link_libraries(surf_lib_python surf_lib ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(surf_lib_python PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS gazebo_spawn_node surf_lib surf_lib_python
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                
install(TARGETS surf_lib_python
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
