#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_gripper");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
    gripper->setMaxVelocityScalingFactor(1.0);
    gripper->setMaxAccelerationScalingFactor(1.0);

    // Print joint names
    RCLCPP_INFO(node->get_logger(), "=== Gripper joints ===");
    auto joint_names = gripper->getJointNames();
    for (const auto& name : joint_names) {
        RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
    }

    // Wait for robot state
    RCLCPP_INFO(node->get_logger(), "Waiting for robot state...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Test open
    RCLCPP_INFO(node->get_logger(), "=== Testing gripper open ===");
    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("open");
    if (gripper->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Planned trajectory joint names:");
        for (const auto& name : plan.trajectory_.joint_trajectory.joint_names) {
            RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
        }
        RCLCPP_INFO(node->get_logger(), "Last point positions:");
        auto last_point = plan.trajectory_.joint_trajectory.points.back();
        for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "  %s: %f",
                plan.trajectory_.joint_trajectory.joint_names[i].c_str(),
                last_point.positions[i]);
        }
        gripper->execute(plan);
        RCLCPP_INFO(node->get_logger(), "Open execution completed");
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Test half_open
    RCLCPP_INFO(node->get_logger(), "=== Testing gripper half_open ===");
    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("half_open");
    if (gripper->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Last point positions:");
        auto last_point = plan.trajectory_.joint_trajectory.points.back();
        for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "  %s: %f",
                plan.trajectory_.joint_trajectory.joint_names[i].c_str(),
                last_point.positions[i]);
        }
        gripper->execute(plan);
        RCLCPP_INFO(node->get_logger(), "Half_open execution completed");
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Test close
    RCLCPP_INFO(node->get_logger(), "=== Testing gripper close ===");
    gripper->setStartStateToCurrentState();
    gripper->setNamedTarget("close");
    if (gripper->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Last point positions:");
        auto last_point = plan.trajectory_.joint_trajectory.points.back();
        for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "  %s: %f",
                plan.trajectory_.joint_trajectory.joint_names[i].c_str(),
                last_point.positions[i]);
        }
        gripper->execute(plan);
        RCLCPP_INFO(node->get_logger(), "Close execution completed");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
