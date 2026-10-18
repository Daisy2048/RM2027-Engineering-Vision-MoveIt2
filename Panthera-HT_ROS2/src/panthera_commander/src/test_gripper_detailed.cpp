#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_gripper_detailed");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
    gripper->setMaxVelocityScalingFactor(1.0);
    gripper->setMaxAccelerationScalingFactor(1.0);

    // Print joint names
    RCLCPP_INFO(node->get_logger(), "Gripper joints:");
    auto joint_names = gripper->getJointNames();
    for (const auto& name : joint_names) {
        RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
    }

    // Wait for robot state to be available
    RCLCPP_INFO(node->get_logger(), "Waiting for robot state...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Try to get current joint positions
    try {
        auto current_joint_positions = gripper->getCurrentJointValues();
        RCLCPP_INFO(node->get_logger(), "Current joint positions:");
        if (current_joint_positions.size() == joint_names.size()) {
            for (size_t i = 0; i < joint_names.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "  %s: %f", joint_names[i].c_str(), current_joint_positions[i]);
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Joint positions vector size (%zu) doesn't match joint names size (%zu)",
                       current_joint_positions.size(), joint_names.size());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get current joint positions: %s", e.what());
    }

    // Test setting joint values directly
    RCLCPP_INFO(node->get_logger(), "Testing direct joint value setting...");

    // Set both fingers to 0.02 (half open)
    std::map<std::string, double> joint_values;
    joint_values["L_finger_joint"] = 0.02;
    joint_values["R_finger_joint"] = 0.02;
    gripper->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto planning_result = gripper->plan(plan);

    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Plan succeeded!");
        RCLCPP_INFO(node->get_logger(), "Planned trajectory has %zu points", plan.trajectory_.joint_trajectory.points.size());
        RCLCPP_INFO(node->get_logger(), "Trajectory joint names:");
        for (const auto& name : plan.trajectory_.joint_trajectory.joint_names) {
            RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
        }

        // Print planned trajectory for first and last point
        if (!plan.trajectory_.joint_trajectory.points.empty()) {
            auto first_point = plan.trajectory_.joint_trajectory.points[0];
            RCLCPP_INFO(node->get_logger(), "First trajectory point:");
            for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "  %s: %f",
                    plan.trajectory_.joint_trajectory.joint_names[i].c_str(),
                    first_point.positions[i]);
            }

            auto last_point = plan.trajectory_.joint_trajectory.points.back();
            RCLCPP_INFO(node->get_logger(), "Last trajectory point:");
            for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "  %s: %f",
                    plan.trajectory_.joint_trajectory.joint_names[i].c_str(),
                    last_point.positions[i]);
            }
        }

        gripper->execute(plan);
        RCLCPP_INFO(node->get_logger(), "Execution completed!");
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Plan failed!");
    }

    // Check positions after execution
    try {
        auto current_joint_positions = gripper->getCurrentJointValues();
        RCLCPP_INFO(node->get_logger(), "Joint positions after execution:");
        if (current_joint_positions.size() == joint_names.size()) {
            for (size_t i = 0; i < joint_names.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "  %s: %f", joint_names[i].c_str(), current_joint_positions[i]);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get joint positions after execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
