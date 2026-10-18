#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit_by_siyuanshu");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // Wait for connections
    RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt connections...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
    arm->setMaxVelocityScalingFactor(0.5);
    arm->setMaxAccelerationScalingFactor(0.5);

    // Step 1: Move to named target 'pose1' first
    RCLCPP_INFO(node->get_logger(), "Step 1: Moving to named target 'pose1'...");
    arm->setStartStateToCurrentState();
    arm->setNamedTarget("pose1");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    auto result1 = arm->plan(plan1);

    if (result1 == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Planning succeeded! Executing...");
        auto exec_result = arm->execute(plan1);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Moved to pose1 successfully!");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Execution failed! Error code: %d", exec_result.val);
            executor.cancel();
            executor_thread.join();
            rclcpp::shutdown();
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed! Error code: %d", result1.val);
        executor.cancel();
        executor_thread.join();
        rclcpp::shutdown();
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Step 2: Try to move to Cartesian pose using quaternion
    RCLCPP_INFO(node->get_logger(), "Step 2: Moving to Cartesian pose with quaternion...");
    RCLCPP_INFO(node->get_logger(), "Using a reachable target near pose1");

    // Use a more reasonable target close to pose1 position
    // pose1 is at approximately x=0.349, y=0.0, z=0.429
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // No rotation, keep same orientation as pose1
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.4;    // Move forward 5cm from pose1
    target_pose.pose.position.y = 0.1;    // Move 10cm to the side
    target_pose.pose.position.z = 0.45;   // Move up 2cm
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    RCLCPP_INFO(node->get_logger(), "Target: x=%.3f, y=%.3f, z=%.3f, qw=%.3f, qx=%.3f, qy=%.3f, qz=%.3f",
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                target_pose.pose.orientation.w, target_pose.pose.orientation.x,
                target_pose.pose.orientation.y, target_pose.pose.orientation.z);

    arm->setStartStateToCurrentState();
    arm->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    auto result2 = arm->plan(plan2);

    if (result2 == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Planning succeeded! Executing...");
        auto exec_result = arm->execute(plan2);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Moved to target pose successfully!");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Execution failed! Error code: %d", exec_result.val);
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning FAILED! Error code: %d", result2.val);
        RCLCPP_ERROR(node->get_logger(), "");
        RCLCPP_ERROR(node->get_logger(), "REASON: No kinematics solver configured!");
        RCLCPP_ERROR(node->get_logger(), "To use Cartesian pose targets, you need to:");
        RCLCPP_ERROR(node->get_logger(), "  1. Create a kinematics.yaml file");
        RCLCPP_ERROR(node->get_logger(), "  2. Configure an IK solver (e.g., KDL, TRAC-IK)");
        RCLCPP_ERROR(node->get_logger(), "  3. Load it in your MoveIt config");
    }

    // Wait a bit before shutting down
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Clean up
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
