cmake_minimum_required(VERSION 3.8)

project(engineer_vision_moveit_bridge)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt 2 依赖
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# 创建可执行程序
add_executable(
    planner_bridge
    src/planner_bridge.cpp
)

# 链接依赖
ament_target_dependencies(
    planner_bridge

    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_msgs
)

# 安装 ROS2 可执行程序
install(
    TARGETS planner_bridge
    DESTINATION lib/${PROJECT_NAME}
)
# 安装 Launch 文件
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)
ament_package()