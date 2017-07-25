cmake_minimum_required(VERSION 2.8.11)

project( DisplayImage )

find_package( OpenCV REQUIRED )

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )

add_executable( track_image src/track_image.cpp )
message(STATUS "${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( track_image ${OpenCV_LIBS} )

