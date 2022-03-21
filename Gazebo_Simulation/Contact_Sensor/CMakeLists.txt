cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(coordinates SHARED CoordinatesPlugin.cc CoordinatesPlugin.hh)
target_link_libraries(coordinates ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} pthread)
