cmake_minimum_required(VERSION 3.5)
project(apriltag_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED) # find OpenCV
find_package(rosidl_default_generators REQUIRED)
find_package(apriltag_service REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectApriltag.srv"
)

# include directories
include_directories(
  include
  simple_apriltag/include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# apriltagのinclude指定
include_directories(/usr/local/include/apriltag)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# declare a C++ executable
add_executable(service_server src/service_server.cpp simple_apriltag/src/simple_tag.cpp)
ament_target_dependencies(service_server rclcpp OpenCV apriltag_service)
target_link_libraries(service_server ${rclcpp_LIBRARIES} ${OpenCV_LIBS} opencv_highgui apriltag)

# install the launch directory
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

# Install C++ executables
install(TARGETS
  service_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
