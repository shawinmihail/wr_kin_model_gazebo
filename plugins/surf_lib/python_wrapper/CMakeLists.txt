cmake_minimum_required(VERSION 3.5)

find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

set(CMAKE_SHARED_MODULE_PREFIX "")



include_directories(
../generated
)


add_library(surf_lib SHARED IMPORTED)
set_property(TARGET surf_lib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/surf_fcn.so)

add_library(python_wrapper MODULE python_wrapper.cpp)
target_link_libraries(python_wrapper surf_lib ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(python_wrapper PRIVATE ${PYTHON_INCLUDE_DIRS})
