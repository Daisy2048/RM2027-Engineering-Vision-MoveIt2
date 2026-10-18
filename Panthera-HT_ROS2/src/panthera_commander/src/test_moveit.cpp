#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
    arm->setMaxVelocityScalingFactor(1.0);
    arm->setMaxAccelerationScalingFactor(1.0);


    arm->setStartStateToCurrentState();
    arm->setNamedTarget("pose1");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm->execute(plan);
    }

    arm->setStartStateToCurrentState();
    arm->setNamedTarget("pose2");
    if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm->execute(plan);
    }

    arm->setStartStateToCurrentState();
    arm->setNamedTarget("home");
    if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm->execute(plan);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
