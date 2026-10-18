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

  auto node = std::make_shared<rclcpp::Node>("draw_three_circles", node_options);

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

  // Step 3: Start drawing three circles continuously
  int cycle_count = 0;
  while (rclcpp::ok())
  {
    cycle_count++;
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "========== Cycle #%d ==========", cycle_count);

    // ===== CIRCLE 1: XY plane (Z constant) =====
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "--- Circle 1: XY plane (Z constant) ---");

    std::vector<geometry_msgs::msg::Pose> xy_waypoints;
    for (int i = 0; i <= num_points; ++i)
    {
      double angle = i * angle_step;
      geometry_msgs::msg::Pose waypoint = center_pose;
      waypoint.position.x = center_pose.position.x + radius * cos(angle);
      waypoint.position.y = center_pose.position.y + radius * sin(angle);
      waypoint.position.z = center_pose.position.z;
      xy_waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory xy_trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    const bool avoid_collisions = true;
    arm.setStartStateToCurrentState();
    double xy_fraction = arm.computeCartesianPath(xy_waypoints, eef_step, jump_threshold, xy_trajectory, avoid_collisions);

    RCLCPP_INFO(node->get_logger(), "XY circle path fraction: %.3f", xy_fraction);

    if (xy_fraction > 0.9)
    {
      double time_from_start = 0.0;
      const double time_step = 0.1;
      for (size_t i = 0; i < xy_trajectory.joint_trajectory.points.size(); ++i)
      {
        time_from_start += time_step;
        xy_trajectory.joint_trajectory.points[i].time_from_start =
            rclcpp::Duration::from_seconds(time_from_start);
      }

      moveit::planning_interface::MoveGroupInterface::Plan xy_plan;
      xy_plan.trajectory_ = xy_trajectory;
      auto xy_result = arm.execute(xy_plan);

      if (xy_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "XY circle completed!");
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "XY circle failed! Error: %d", xy_result.val);
        break;
      }
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "XY circle planning failed! fraction = %.3f", xy_fraction);
      break;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // ===== CIRCLE 2: XZ plane (Y constant) =====
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "--- Circle 2: XZ plane (Y constant) ---");

    std::vector<geometry_msgs::msg::Pose> xz_waypoints;
    for (int i = 0; i <= num_points; ++i)
    {
      double angle = i * angle_step;
      geometry_msgs::msg::Pose waypoint = center_pose;
      waypoint.position.x = center_pose.position.x + radius * cos(angle);
      waypoint.position.y = center_pose.position.y;
      waypoint.position.z = center_pose.position.z + radius * sin(angle);
      xz_waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory xz_trajectory;
    arm.setStartStateToCurrentState();
    double xz_fraction = arm.computeCartesianPath(xz_waypoints, eef_step, jump_threshold, xz_trajectory, avoid_collisions);

    RCLCPP_INFO(node->get_logger(), "XZ circle path fraction: %.3f", xz_fraction);

    if (xz_fraction > 0.9)
    {
      double time_from_start = 0.0;
      const double time_step = 0.1;
      for (size_t i = 0; i < xz_trajectory.joint_trajectory.points.size(); ++i)
      {
        time_from_start += time_step;
        xz_trajectory.joint_trajectory.points[i].time_from_start =
            rclcpp::Duration::from_seconds(time_from_start);
      }

      moveit::planning_interface::MoveGroupInterface::Plan xz_plan;
      xz_plan.trajectory_ = xz_trajectory;
      auto xz_result = arm.execute(xz_plan);

      if (xz_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "XZ circle completed!");
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "XZ circle failed! Error: %d", xz_result.val);
        break;
      }
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "XZ circle planning failed! fraction = %.3f", xz_fraction);
      break;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // ===== CIRCLE 3: YZ plane (X constant) =====
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "--- Circle 3: YZ plane (X constant) ---");

    std::vector<geometry_msgs::msg::Pose> yz_waypoints;
    for (int i = 0; i <= num_points; ++i)
    {
      double angle = i * angle_step;
      geometry_msgs::msg::Pose waypoint = center_pose;
      waypoint.position.x = center_pose.position.x;
      waypoint.position.y = center_pose.position.y + radius * cos(angle);
      waypoint.position.z = center_pose.position.z + radius * sin(angle);
      yz_waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory yz_trajectory;
    arm.setStartStateToCurrentState();
    double yz_fraction = arm.computeCartesianPath(yz_waypoints, eef_step, jump_threshold, yz_trajectory, avoid_collisions);

    RCLCPP_INFO(node->get_logger(), "YZ circle path fraction: %.3f", yz_fraction);

    if (yz_fraction > 0.9)
    {
      double time_from_start = 0.0;
      const double time_step = 0.1;
      for (size_t i = 0; i < yz_trajectory.joint_trajectory.points.size(); ++i)
      {
        time_from_start += time_step;
        yz_trajectory.joint_trajectory.points[i].time_from_start =
            rclcpp::Duration::from_seconds(time_from_start);
      }

      moveit::planning_interface::MoveGroupInterface::Plan yz_plan;
      yz_plan.trajectory_ = yz_trajectory;
      auto yz_result = arm.execute(yz_plan);

      if (yz_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "YZ circle completed!");
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "YZ circle failed! Error: %d", yz_result.val);
        break;
      }
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "YZ circle planning failed! fraction = %.3f", yz_fraction);
      break;
    }

    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "Cycle #%d completed! (XY + XZ + YZ circles)", cycle_count);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(node->get_logger(), "Stopping. Total cycles completed: %d", cycle_count - 1);

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}

