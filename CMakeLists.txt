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
    "include/kalman_filter.hpp"
    "include/stabilizer.hpp"
)

set(SOURCES
    "src/buffered_tracker.cpp"
    "src/buffered_video.cpp"
    "src/template_match.cpp"
    "src/robot_position.cpp"
    "src/kalman_filter.cpp"
    "src/stabilizer.cpp"
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

add_executable(convert_pnp "src/convert_pnp.cpp")
target_link_libraries( convert_pnp track_image_lib ${Boost_LIBRARIES} )

add_executable(kf_test "src/kf_test.cpp")
target_link_libraries( kf_test track_image_lib ${Boost_LIBRARIES} )

add_executable(stabilizer_test "src/stabilizer_test.cpp")
target_link_libraries( stabilizer_test track_image_lib ${Boost_LIBRARIES} )

