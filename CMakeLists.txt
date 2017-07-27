cmake_minimum_required(VERSION 2.8.11)

project( DisplayImage )

find_package( OpenCV REQUIRED )

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )

set(HEADERS
    "include/buffered_tracker.hpp"
    "include/buffered_video.hpp"
)

set(SOURCES
    "src/buffered_tracker.cpp"
    "src/buffered_video.cpp"
    "src/track_image.cpp"
)

add_executable( track_image ${HEADERS} ${SOURCES} )
target_include_directories(track_image PUBLIC "include")
message(STATUS "${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( track_image ${OpenCV_LIBS} )

