cmake_minimum_required(VERSION 3.8)
project(panthera_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(panthera_interfaces REQUIRED)

# find_package(hightorque_robot REQUIRED)
# Package not found, commented out


add_executable(test_moveit
  src/test_moveit.cpp
)
ament_target_dependencies(
  test_moveit
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(test_moveit_by_cartesian
  src/test_moveit_by_cartesian.cpp
)
ament_target_dependencies(
  test_moveit_by_cartesian
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(test_moveit_by_jointspace
  src/test_moveit_by_jointspace.cpp
)
ament_target_dependencies(
  test_moveit_by_jointspace
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(test_moveit_by_siyuanshu
  src/test_moveit_by_siyuanshu.cpp
)
ament_target_dependencies(
  test_moveit_by_siyuanshu
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(test_gripper
  src/test_gripper.cpp
)
ament_target_dependencies(
  test_gripper
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(test_gripper_detailed
  src/test_gripper_detailed.cpp
)
ament_target_dependencies(
  test_gripper_detailed
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(test_cartesian_from_pose1
  src/test_cartesian_from_pose1.cpp
)
ament_target_dependencies(
  test_cartesian_from_pose1
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(commander_template
  src/commander_template.cpp
)
ament_target_dependencies(
  commander_template
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  panthera_interfaces
)


add_executable(store_auto
  src/store_auto.cpp
)
ament_target_dependencies(
  store_auto
  rclcpp
  moveit_ros_planning_interface
)


# T2:
# READY_WITH_ORE -> PRE_STORE -> APPROACH
add_executable(test_ready_pre_approach
  src/test_ready_pre_approach.cpp
)
ament_target_dependencies(
  test_ready_pre_approach
  rclcpp
  moveit_ros_planning_interface
)

add_executable(test_full_store_sequence
  src/test_full_store_sequence.cpp
)

ament_target_dependencies(
  test_full_store_sequence
  rclcpp
  moveit_ros_planning_interface
)
add_executable(draw_circle
  src/draw_circle.cpp
)
ament_target_dependencies(
  draw_circle
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(draw_horizontal_vertical_circles
  src/draw_horizontal_vertical_circles.cpp
)
ament_target_dependencies(
  draw_horizontal_vertical_circles
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


add_executable(draw_three_circles
  src/draw_three_circles.cpp
)
ament_target_dependencies(
  draw_three_circles
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
)


# add_executable(sin_trajectory_control
#   src/sin_trajectory_control.cpp
# )
# target_link_libraries(
#   sin_trajectory_control
#   hightorque_robot::hightorque_robot
# )
# ament_target_dependencies(
#   sin_trajectory_control
#   rclcpp
# )
# Commented out: depends on missing hightorque_robot package


add_executable(sin_trajectory_moveit
  src/sin_trajectory_moveit.cpp
)
ament_target_dependencies(
  sin_trajectory_moveit
  rclcpp
  moveit_ros_planning_interface
)


install(TARGETS
  test_moveit
  test_moveit_by_cartesian
  test_moveit_by_jointspace
  test_moveit_by_siyuanshu
  test_gripper
  store_auto
  test_ready_pre_approach
  test_full_store_sequence
  test_gripper_detailed
  test_cartesian_from_pose1
  commander_template
  draw_circle
  draw_horizontal_vertical_circles
  draw_three_circles
  # sin_trajectory_control
  sin_trajectory_moveit

  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


ament_package()
