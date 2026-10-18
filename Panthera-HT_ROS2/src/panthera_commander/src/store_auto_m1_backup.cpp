#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

#include <vector>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("store_auto");

    auto arm =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, "arm");

    // M1 的唯一目标：官方 SRDF 中的 pose2
    std::vector<double> target_joint_values = {
        -0.1989,
         0.7072,
         0.6630,
        -0.4862,
         0.5917,
        -0.7597
    };

    // 从当前机械臂状态开始规划
    arm->setStartStateToCurrentState();

    // 设置六关节目标
    arm->setJointValueTarget(target_joint_values);

    // 用来保存 MoveIt 规划出的轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // 请求 MoveIt 进行运动规划
    auto success = arm->plan(plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "M1 planning succeeded");
        arm->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "M1 planning failed");
    }

    rclcpp::shutdown();
    return 0;
}
