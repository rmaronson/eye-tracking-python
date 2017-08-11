cmake_minimum_required(VERSION 2.8.11)

project( DisplayImage )

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED COMPONENTS filesystem)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )

set(HEADERS
    "include/buffered_tracker.hpp"
    "include/buffered_video.hpp"
    "include/template_match.hpp"
    "include/robot_position.hpp"
)

set(SOURCES
    "src/buffered_tracker.cpp"
    "src/buffered_video.cpp"
    "src/template_match.cpp"
    "src/robot_position.cpp"
)

add_library(track_image_lib ${HEADERS} ${SOURCES} )

target_include_directories(track_image_lib PUBLIC "include" ${OpenCV_INCLUDE_DIRS})

target_link_libraries( track_image_lib ${OpenCV_LIBS} )



add_executable( track_image "src/track_image.cpp" )
target_link_libraries( track_image track_image_lib )


add_executable( extract_video "src/extract_video.cpp" )
target_link_libraries( extract_video track_image_lib )

add_executable(extract_templates "src/extract_templates.cpp")
target_link_libraries( extract_templates track_image_lib ${Boost_LIBRARIES} )


add_executable(template_match "src/do_template_match.cpp")
target_link_libraries( template_match track_image_lib ${Boost_LIBRARIES} )

add_executable(position_test "src/position_test.cpp")
target_link_libraries( position_test track_image_lib ${Boost_LIBRARIES} )
