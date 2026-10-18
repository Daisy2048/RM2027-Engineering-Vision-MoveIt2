#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit_by_jointspace");

    auto arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
    arm->setMaxVelocityScalingFactor(1.0);
    arm->setMaxAccelerationScalingFactor(1.0);

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = node->now();

    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.2;

    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(node->get_logger(), "Target pose: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                target_pose.pose.orientation.z, target_pose.pose.orientation.w);

    arm->setStartStateToCurrentState();
    arm->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = arm->plan(plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Planning succeeded, executing...");
        arm->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed, error code: %d", success.val);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
