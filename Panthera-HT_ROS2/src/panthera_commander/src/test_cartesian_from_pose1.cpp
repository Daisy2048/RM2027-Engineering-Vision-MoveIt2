#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("test_cartesian_from_pose1", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(node->get_logger(), "Waiting for joint states...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  // Step 1: Move to pose1 first (better working position)
  RCLCPP_INFO(node->get_logger(), "Step 1: Moving to pose1...");
  arm.setStartStateToCurrentState();
  arm.setNamedTarget("pose1");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Moved to pose1 successfully!");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan to pose1");
    rclcpp::shutdown();
    return 1;
  }

  // Step 2: Get pose1 position from robot model (avoids clock sync issues)
  RCLCPP_INFO(node->get_logger(), "Step 2: Getting pose from named state 'pose1'...");

  // Get robot model and create a robot state
  const moveit::core::RobotModelConstPtr& robot_model = arm.getRobotModel();
  moveit::core::RobotState robot_state(robot_model);

  // Get the joint model group for "arm"
  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm");
  if (!joint_model_group)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get joint model group 'arm'");
    rclcpp::shutdown();
    return 1;
  }

  // Set to named target "pose1"
  if (!robot_state.setToDefaultValues(joint_model_group, "pose1"))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to set robot state to pose1");
    rclcpp::shutdown();
    return 1;
  }

  // Get the end effector link
  const std::string& end_effector_link = arm.getEndEffectorLink();
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", end_effector_link.c_str());

  // Get pose from robot state at pose1
  const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform(end_effector_link);
  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = end_effector_state.translation().x();
  current_pose.position.y = end_effector_state.translation().y();
  current_pose.position.z = end_effector_state.translation().z();

  Eigen::Quaterniond q(end_effector_state.rotation());
  current_pose.orientation.x = q.x();
  current_pose.orientation.y = q.y();
  current_pose.orientation.z = q.z();
  current_pose.orientation.w = q.w();

  RCLCPP_INFO(node->get_logger(), "Current pose at pose1: x=%.3f, y=%.3f, z=%.3f",
              current_pose.position.x,
              current_pose.position.y,
              current_pose.position.z);
  RCLCPP_INFO(node->get_logger(), "Orientation: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
              current_pose.orientation.x,
              current_pose.orientation.y,
              current_pose.orientation.z,
              current_pose.orientation.w);

  // Step 3: Create small cartesian movements (forward/backward along x-axis)
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Waypoint 1: move forward 7cm from current pose
  auto target_pose = current_pose;
  target_pose.position.x += 0.07;
  waypoints.push_back(target_pose);
  RCLCPP_INFO(node->get_logger(), "Waypoint 1: x=%.3f, y=%.3f, z=%.3f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Waypoint 2: back to original position
  waypoints.push_back(current_pose);
  RCLCPP_INFO(node->get_logger(), "Waypoint 2: x=%.3f, y=%.3f, z=%.3f (back to start)",
              current_pose.position.x, current_pose.position.y, current_pose.position.z);

  // Waypoint 3: move backward 7cm
  target_pose = current_pose;
  target_pose.position.x -= 0.07;
  waypoints.push_back(target_pose);
  RCLCPP_INFO(node->get_logger(), "Waypoint 3: x=%.3f, y=%.3f, z=%.3f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Step 4: Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;  // 10mm resolution
  const double jump_threshold = 0.0;  // Disable jump threshold
  const bool avoid_collisions = true;

  RCLCPP_INFO(node->get_logger(), "Step 3: Computing Cartesian path...");

  // Use robot's actual current state as start state
  arm.setStartStateToCurrentState();

  double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

  RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (fraction > 0.9)
  {
    RCLCPP_INFO(node->get_logger(), "Step 4: Adding time stamps to trajectory...");

    // Manually add time stamps to trajectory points
    double time_from_start = 0.0;
    const double time_step = 0.5;  // 0.5 seconds between waypoints

    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
    {
      time_from_start += time_step;
      trajectory.joint_trajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    }

    RCLCPP_INFO(node->get_logger(), "Step 5: Executing cartesian path with %zu points...",
                trajectory.joint_trajectory.points.size());

    // Wrap trajectory in a Plan object
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    auto result = arm.execute(cartesian_plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Cartesian path executed successfully!");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Cartesian path execution failed! Error code: %d", result.val);
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Cartesian path planning failed! fraction = %.3f", fraction);
    if (fraction == 0.0)
    {
      RCLCPP_ERROR(node->get_logger(), "Complete failure - IK solver could not find any solution");
      RCLCPP_INFO(node->get_logger(), "Possible causes:");
      RCLCPP_INFO(node->get_logger(), "  1. Waypoints are outside reachable workspace");
      RCLCPP_INFO(node->get_logger(), "  2. Required orientation cannot be maintained");
      RCLCPP_INFO(node->get_logger(), "  3. Joint limits would be violated");
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Partial path found - only %.1f%% achievable", fraction * 100);
    }
  }

  rclcpp::sleep_for(std::chrono::seconds(2));
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
