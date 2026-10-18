#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("test_moveit_by_cartesian", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&executor]() { executor.spin(); });

  // Give time for current_state_monitor to receive first joint_states
  RCLCPP_INFO(node->get_logger(), "Waiting for current state monitor to initialize...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);

  // Check if we can get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  try {
    current_pose = arm.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get current pose: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Create waypoints with smaller movements
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto target_pose = current_pose.pose;

  // First waypoint: move down 5cm
  target_pose.position.z -= 0.05;
  waypoints.push_back(target_pose);
  RCLCPP_INFO(node->get_logger(), "Waypoint 1: x=%.3f, y=%.3f, z=%.3f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Second waypoint: move right 5cm from start
  target_pose = current_pose.pose;
  target_pose.position.y += 0.05;
  waypoints.push_back(target_pose);
  RCLCPP_INFO(node->get_logger(), "Waypoint 2: x=%.3f, y=%.3f, z=%.3f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Third waypoint: move left 5cm from start
  target_pose = current_pose.pose;
  target_pose.position.y -= 0.05;
  waypoints.push_back(target_pose);
  RCLCPP_INFO(node->get_logger(), "Waypoint 3: x=%.3f, y=%.3f, z=%.3f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;  // 1cm resolution
  const double jump_threshold = 0.0;  // Disable jump threshold
  const bool avoid_collisions = true;

  RCLCPP_INFO(node->get_logger(), "Computing Cartesian path...");
  double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

  RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (fraction > 0.8)  // Accept if at least 80% of path is achievable
  {
    RCLCPP_INFO(node->get_logger(), "Executing cartesian path...");
    arm.execute(trajectory);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Cartesian path failed! fraction = %.3f", fraction);
    RCLCPP_ERROR(node->get_logger(), "This might mean the waypoints are outside the workspace.");
    RCLCPP_INFO(node->get_logger(), "Try moving the robot to a different starting position first.");
  }

  rclcpp::sleep_for(std::chrono::seconds(2));
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
