cmake_minimum_required(VERSION 3.8)

project(engineer_vision_tf_bridge)

# ==============================
# 1. C++ 编译设置
# ==============================

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ==============================
# 2. ROS2 依赖
# ==============================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# ==============================
# 3. TF 坐标转换节点
# ==============================

add_executable(
  tf_bridge
  src/tf_bridge.cpp
)

ament_target_dependencies(
  tf_bridge
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# ==============================
# 4. 候选目标位姿预览节点
# ==============================

add_executable(
  target_pose_preview
  src/target_pose_preview.cpp
)

ament_target_dependencies(
  target_pose_preview
  rclcpp
  geometry_msgs
  tf2
)


# ==============================
# 5. 单次规划触发节点（新）
# ==============================

add_executable(
  planning_trigger
  src/planning_trigger.cpp
)

ament_target_dependencies(
  planning_trigger
  rclcpp
  geometry_msgs
  std_srvs
)


# ==============================
# 6. 安装三个可执行程序
# ==============================

install(
  TARGETS
    tf_bridge
    target_pose_preview
    planning_trigger
  DESTINATION lib/${PROJECT_NAME}
)


# ==============================
# 7. ROS2 软件包声明
# ==============================

ament_package()