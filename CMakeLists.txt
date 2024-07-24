cmake_minimum_required(VERSION 3.5)
project(test_pose_utils)

set(CMAKE_CXX_STANDARD 14)

# ROS2の依存関係を追加
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# インクルードディレクトリを指定
include_directories(include) # ヘッダーのディレクトリ

# 実行ファイルの追加
add_executable(test_pose_utils example/test_pose_utils.cpp)

# リンクするライブラリを指定
ament_target_dependencies(test_pose_utils
  "tf2"
  "tf2_geometry_msgs"
)

# インストール
install(TARGETS
  test_pose_utils
  DESTINATION lib/${PROJECT_NAME})
