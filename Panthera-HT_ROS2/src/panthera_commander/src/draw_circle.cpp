#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/executors.hpp>
#include <cmath>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("draw_circle", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt connections...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  // Step 1: Move to pose1 first (starting position)
  RCLCPP_INFO(node->get_logger(), "Step 1: Moving to starting position (pose1)...");
  arm.setStartStateToCurrentState();
  arm.setNamedTarget("pose1");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Moved to starting position successfully!");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan to starting position");
    rclcpp::shutdown();
    return 1;
  }

  // Step 2: Get current pose from named state
  RCLCPP_INFO(node->get_logger(), "Step 2: Getting current pose...");

  const moveit::core::RobotModelConstPtr& robot_model = arm.getRobotModel();
  moveit::core::RobotState robot_state(robot_model);

  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm");
  if (!joint_model_group)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get joint model group 'arm'");
    rclcpp::shutdown();
    return 1;
  }

  if (!robot_state.setToDefaultValues(joint_model_group, "pose1"))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to set robot state to pose1");
    rclcpp::shutdown();
    return 1;
  }

  const std::string& end_effector_link = arm.getEndEffectorLink();
  const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform(end_effector_link);

  geometry_msgs::msg::Pose center_pose;
  center_pose.position.x = end_effector_state.translation().x();
  center_pose.position.y = end_effector_state.translation().y();
  center_pose.position.z = end_effector_state.translation().z();

  Eigen::Quaterniond q(end_effector_state.rotation());
  center_pose.orientation.x = q.x();
  center_pose.orientation.y = q.y();
  center_pose.orientation.z = q.z();
  center_pose.orientation.w = q.w();

  RCLCPP_INFO(node->get_logger(), "Circle center: x=%.3f, y=%.3f, z=%.3f",
              center_pose.position.x, center_pose.position.y, center_pose.position.z);

  // Circle parameters
  const double radius = 0.05;  // 5cm radius
  const int num_points = 36;   // 36 points for smooth circle
  const double angle_step = 2.0 * M_PI / num_points;

  RCLCPP_INFO(node->get_logger(), "Circle parameters: radius=%.3fm, points=%d", radius, num_points);

  // Step 3: Start drawing circles continuously
  int circle_count = 0;
  while (rclcpp::ok())
  {
    circle_count++;
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "========== Drawing Circle #%d ==========", circle_count);

    // Generate circle waypoints in XY plane (parallel to ground)
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (int i = 0; i <= num_points; ++i)  // Include endpoint to close the circle
    {
      double angle = i * angle_step;
      geometry_msgs::msg::Pose waypoint = center_pose;

      // Draw circle in XY plane (Z stays constant)
      waypoint.position.x = center_pose.position.x + radius * cos(angle);
      waypoint.position.y = center_pose.position.y + radius * sin(angle);
      // Z remains the same (parallel to ground)

      waypoints.push_back(waypoint);
    }

    RCLCPP_INFO(node->get_logger(), "Generated %zu waypoints for circle", waypoints.size());

    // Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1cm resolution
    const double jump_threshold = 0.0;  // Disable jump threshold
    const bool avoid_collisions = true;

    arm.setStartStateToCurrentState();
    double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

    RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

    if (fraction > 0.9)
    {
      // Add time stamps to trajectory
      double time_from_start = 0.0;
      const double time_step = 0.1;  // 0.1 seconds between points

      for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
      {
        time_from_start += time_step;
        trajectory.joint_trajectory.points[i].time_from_start =
            rclcpp::Duration::from_seconds(time_from_start);
      }

      RCLCPP_INFO(node->get_logger(), "Executing circle with %zu trajectory points...",
                  trajectory.joint_trajectory.points.size());

      // Execute trajectory
      moveit::planning_interface::MoveGroupInterface::Plan circle_plan;
      circle_plan.trajectory_ = trajectory;

      auto result = arm.execute(circle_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "Circle #%d completed successfully!", circle_count);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Circle execution failed! Error code: %d", result.val);
        break;
      }
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Cartesian path planning failed! fraction = %.3f", fraction);
      RCLCPP_ERROR(node->get_logger(), "Cannot complete circle, stopping...");
      break;
    }

    // Small pause between circles
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(node->get_logger(), "Stopping circle drawing. Total circles drawn: %d", circle_count - 1);

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
