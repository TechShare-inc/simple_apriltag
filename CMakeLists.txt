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
find_package(apriltag REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectApriltag.srv"
)

# include directories
include_directories(
  simple_apriltag/include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# declare a C++ executable
add_executable(service_server src/service_server.cpp simple_apriltag/src/simple_tag.cpp)
ament_target_dependencies(service_server rclcpp OpenCV)
rosidl_target_interfaces(service_server ${PROJECT_NAME} "rosidl_typesupport_cpp") # 生成されたメッセージヘッダーに依存することを指定
target_link_libraries(service_server ${rclcpp_LIBRARIES} ${OpenCV_LIBS} apriltag)

# declare a C++ executable
add_executable(service_server_gst src/service_server_gst.cpp simple_apriltag/src/simple_tag.cpp)
ament_target_dependencies(service_server_gst rclcpp OpenCV)
rosidl_target_interfaces(service_server_gst ${PROJECT_NAME} "rosidl_typesupport_cpp") # 生成されたメッセージヘッダーに依存することを指定
target_link_libraries(service_server_gst ${rclcpp_LIBRARIES} ${OpenCV_LIBS} apriltag)

# declare a C++ executable
add_executable(service_client src/service_client.cpp simple_apriltag/src/simple_tag.cpp)
ament_target_dependencies(service_client rclcpp OpenCV)
rosidl_target_interfaces(service_client ${PROJECT_NAME} "rosidl_typesupport_cpp") # 生成されたメッセージヘッダーに依存することを指定
target_link_libraries(service_client ${rclcpp_LIBRARIES} ${OpenCV_LIBS} apriltag)

# Install C++ executables
install(TARGETS
  service_server
  service_server_gst
  service_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
