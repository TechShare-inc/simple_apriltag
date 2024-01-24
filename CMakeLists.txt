cmake_minimum_required( VERSION 2.8 )
project(simple_apriltag)

find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()


include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  /usr/local/include/apriltag
)

add_executable(example_detect example/example_detect.cpp src/simple_tag.cpp)
target_link_libraries(example_detect ${OpenCV_LIBS} apriltag)

add_executable(example_detect_gst example/example_detect_gst.cpp src/simple_tag.cpp)
target_link_libraries(example_detect_gst ${OpenCV_LIBS} apriltag)
