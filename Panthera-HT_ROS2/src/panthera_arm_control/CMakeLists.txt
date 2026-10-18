cmake_minimum_required(VERSION 3.8)
project(panthera_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hightorque_robot REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(panthera_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)

add_executable(arm_control_node src/arm_control_node.cpp)
add_executable(test_kdl src/test_kdl.cpp)
ament_target_dependencies(arm_control_node
  rclcpp
  ament_index_cpp
  example_interfaces
  panthera_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  kdl_parser
  orocos_kdl_vendor
  geometry_msgs
  tf2
  tf2_kdl
)
target_link_libraries(arm_control_node hightorque_robot::hightorque_robot)

ament_target_dependencies(test_kdl
  ament_index_cpp
  kdl_parser
  orocos_kdl_vendor
)

install(TARGETS
  arm_control_node
  test_kdl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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
