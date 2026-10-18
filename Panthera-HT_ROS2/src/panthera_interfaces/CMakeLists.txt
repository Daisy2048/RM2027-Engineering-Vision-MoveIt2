cmake_minimum_required(VERSION 3.8)
project(panthera_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPose.msg"
  "msg/ArmStatus.msg"
  "msg/EndPoseEuler.msg"
  "msg/PosCmd.msg"
  "srv/MoveToJoint.srv"
  "srv/MoveToPose.srv"
  "srv/GripperControl.srv"
  "srv/Enable.srv"
  "srv/GoZero.srv"
  "srv/GripperSrv.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
