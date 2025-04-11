cmake_minimum_required(VERSION 3.5)
project(marker_pose_estimation)

set(CMAKE_CXX_STANDARD 17)

# ROS2の依存関係を追加
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OpenCVの依存関係を追加
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
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

# AprilTagの依存関係を追加
find_package(apriltag REQUIRED)

# インクルードディレクトリを指定
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

# 実行ファイルの追加
add_executable(test_pose_utils example/test_pose_utils.cpp src/pose_utils.cpp)
add_executable(example_detect example/example_detect.cpp src/simple_tag.cpp)
add_executable(example_detect_gst example/example_detect_gst.cpp src/simple_tag.cpp)
add_executable(example_detect_pic example/example_detect_pic.cpp src/simple_tag.cpp)
add_executable(example_multi_detect_pic example/example_multi_detect_pic.cpp src/simple_tag.cpp src/multi_marker_pose_estimator.cpp src/pose_utils.cpp)
add_executable(example_multi_detect example/example_multi_detect.cpp src/simple_tag.cpp src/multi_marker_pose_estimator.cpp)

# リンクするライブラリを指定
ament_target_dependencies(test_pose_utils
  "tf2"
  "tf2_geometry_msgs"
)

target_link_libraries(example_detect ${OpenCV_LIBS} apriltag)
target_link_libraries(example_detect_gst ${OpenCV_LIBS} apriltag)
target_link_libraries(example_detect_pic ${OpenCV_LIBS} apriltag)
target_link_libraries(example_multi_detect ${OpenCV_LIBS} apriltag)


ament_target_dependencies(example_multi_detect_pic
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(example_multi_detect_pic ${OpenCV_LIBS} apriltag)

# インストール
install(TARGETS
  test_pose_utils
  example_detect
  example_detect_gst
  example_detect_pic
  example_multi_detect_pic
  example_multi_detect
  DESTINATION lib/${PROJECT_NAME}
)
