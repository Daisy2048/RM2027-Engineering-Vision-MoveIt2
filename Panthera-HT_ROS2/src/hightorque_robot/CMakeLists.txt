cmake_minimum_required(VERSION 3.5)
project(hightorque_robot)

# Find ament_cmake for ROS2 integration
find_package(ament_cmake REQUIRED)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install path prefix" FORCE)
endif()

set(VERSION_MAJOR 4)
set(VERSION_MINOR 4)
set(VERSION_PATCH 7)
set(PROJECT_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

add_subdirectory(third_part/lcm)
add_subdirectory(third_part/serial_cmake)

# find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found! Please install libserialport-dev.")
else()
  message(STATUS "Found libserialport library: ${SERIALPORT_LIBRARY}")
endif()

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)

set(serial_SRCS
    src/serial_driver.cpp
    src/parse_robot_params.cpp
    src/hardware/motor.cpp
    src/hardware/canport.cpp
    src/hardware/canboard.cpp
    src/hardware/robot.cpp
    src/crc/crc8.cpp
    src/crc/crc16.cpp
    src/panthera/Panthera.cpp
)

add_library(hightorque_robot SHARED ${serial_SRCS})

target_include_directories(hightorque_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hardware>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/panthera>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msg>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/crc>
  $<INSTALL_INTERFACE:include/hardware>
  $<INSTALL_INTERFACE:include/panthera>
  $<INSTALL_INTERFACE:include/motor_msg>
)

target_link_libraries(hightorque_robot 
    PUBLIC 
        lcm
        yaml-cpp
        serial_cmake
        ament_index_cpp::ament_index_cpp
        ${SERIALPORT_LIBRARY}   # libserialport
    PRIVATE 
        rt 
        pthread
)


add_executable(canboard_update example/canboard_update.cpp)
target_link_libraries(canboard_update hightorque_robot)

add_executable(motor_feedack example/motor_feedack.cpp)
target_link_libraries(motor_feedack hightorque_robot)

add_executable(motor_move_zero example/motor_move_zero.cpp)
target_link_libraries(motor_move_zero hightorque_robot)

add_executable(motor_run example/motor_run.cpp)
target_link_libraries(motor_run hightorque_robot)

add_executable(motor_msg_subscriber example/motor_msg_subscriber.cpp)
target_link_libraries(motor_msg_subscriber hightorque_robot)

add_executable(parse_demo example/parse_demo.cpp)
target_link_libraries(parse_demo hightorque_robot)

set_target_properties(hightorque_robot PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${VERSION_MAJOR}
    PUBLIC_HEADER "include/hardware/robot.hpp;include/hardware/motor.hpp;include/hardware/canport.hpp;include/hardware/canboard.hpp;include/parse_robot_params.hpp;include/serial_struct.hpp;include/serial_driver.hpp;include/crc/crc8.hpp;include/crc/crc16.hpp;include/panthera/Panthera.hpp;msg/motor_msg/motor_msg.hpp"
)

install(TARGETS hightorque_robot
    EXPORT hightorque_robotTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

install(DIRECTORY include/ DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY msg/motor_msg/ DESTINATION include/motor_msg 
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY robot_param/ DESTINATION share/hightorque_robot/robot_param
    FILES_MATCHING PATTERN "*.yaml")
# Old Panthera-HT_description URDF removed - using panthera_ht_ros_description instead
# install(DIRECTORY Panthera-HT_description/urdf/ DESTINATION share/hightorque_robot/urdf
#     FILES_MATCHING PATTERN "*.urdf")

install(EXPORT hightorque_robotTargets
    FILE hightorque_robotTargets.cmake
    NAMESPACE hightorque_robot::
    DESTINATION lib/cmake/hightorque_robot
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/hightorque_robotConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_robotConfig.cmake
    INSTALL_DESTINATION lib/cmake/hightorque_robot
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_robotConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_robotConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_robotConfigVersion.cmake
    DESTINATION lib/cmake/hightorque_robot
)
add_executable(motor_set_zero example/motor_set_zero.cpp)
target_link_libraries(motor_set_zero hightorque_robot)

# Panthera examples
add_executable(0_robot_get_state examples/0_robot_get_state.cpp)
target_link_libraries(0_robot_get_state hightorque_robot ament_index_cpp::ament_index_cpp)

add_executable(0_robot_set_zero examples/0_robot_set_zero.cpp)
target_link_libraries(0_robot_set_zero hightorque_robot ament_index_cpp::ament_index_cpp)

add_executable(1_PD_control examples/1_PD_control.cpp)
target_link_libraries(1_PD_control hightorque_robot ament_index_cpp::ament_index_cpp)

add_executable(1_PosVel_control examples/1_PosVel_control.cpp)
target_link_libraries(1_PosVel_control hightorque_robot ament_index_cpp::ament_index_cpp)

# Find Pinocchio for dynamics computation (optional)
find_package(pinocchio QUIET)

if(pinocchio_FOUND)
  message(STATUS "Found pinocchio, building 2_joint_impedance_control example")
  add_executable(2_joint_impedance_control examples/2_joint_impedance_control.cpp)
  target_link_libraries(2_joint_impedance_control hightorque_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building 3_cartesian_impedance_control example")
  add_executable(3_cartesian_impedance_control examples/3_cartesian_impedance_control.cpp)
  target_link_libraries(3_cartesian_impedance_control hightorque_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building pure_cartesian_impedance_control example")
  add_executable(pure_cartesian_impedance_control examples/pure_cartesian_impedance_control.cpp)
  target_link_libraries(pure_cartesian_impedance_control hightorque_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building cartesian_impedance_ab_motion example")
  add_executable(cartesian_impedance_ab_motion examples/cartesian_impedance_ab_motion.cpp)
  target_link_libraries(cartesian_impedance_ab_motion hightorque_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)
else()
  message(STATUS "pinocchio not found, skipping impedance control examples")
endif()


# Install example executables so ros2 run can find them
install(TARGETS
    canboard_update
    motor_feedack
    motor_move_zero
    motor_run
    motor_msg_subscriber
    parse_demo
    motor_set_zero
    0_robot_get_state
    0_robot_set_zero
    1_PD_control
    1_PosVel_control
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(pinocchio_FOUND)
  install(TARGETS
      2_joint_impedance_control
      3_cartesian_impedance_control
      pure_cartesian_impedance_control
      cartesian_impedance_ab_motion
      RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Ament package setup for ROS2
ament_package()
