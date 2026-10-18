#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <thread>
#include <vector>
#include "moveit_msgs/msg/robot_trajectory.hpp"



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("store_auto");
    rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);

std::thread spinner([&executor]() {
    executor.spin();
});
    auto arm =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, "arm");

    // 等待 MoveIt 获取到有效的当前机器人状态
auto current_state = arm->getCurrentState(5.0);

if (!current_state)
{
    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to get current robot state");

    executor.cancel();

    if (spinner.joinable())
    {
        spinner.join();
    }

    rclcpp::shutdown();
    return 1;
}
// 从当前机械臂状态开始规划
arm->setStartStateToCurrentState();
// 当前机械臂已经停在 STORE
auto store_pose = arm->getCurrentPose().pose;

// RETREAT = APPROACH
// 保持 x、y、orientation 不变，只沿 +Z 上升
auto retreat_pose = store_pose;
retreat_pose.position.z = 0.350;

// Cartesian Path 的目标点
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(retreat_pose);

// 保存 STORE -> RETREAT 的笛卡尔轨迹
moveit_msgs::msg::RobotTrajectory retreat_trajectory;

// 每 5 mm 计算一个中间点
double eef_step = 0.005;

// 暂时关闭 joint jump 检查
double jump_threshold = 0.0;

// 计算 STORE -> RETREAT
double fraction = arm->computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    retreat_trajectory);

// 第一轮只看路径能不能完整生成，不执行
RCLCPP_INFO(
    node->get_logger(),
    "RETREAT Cartesian path fraction: %.1f%%",
    fraction * 100.0);

if (fraction >= 0.99)
{
    RCLCPP_INFO(
        node->get_logger(),
        "M2 STORE -> RETREAT Cartesian path succeeded");

    RCLCPP_INFO(
        node->get_logger(),
        "Executing RETREAT Cartesian path...");

    auto execute_result = arm->execute(retreat_trajectory);

    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "M2 STORE -> RETREAT execution succeeded");
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "M2 STORE -> RETREAT execution failed");
    }
}
else
{
    RCLCPP_ERROR(
        node->get_logger(),
        "M2 STORE -> RETREAT Cartesian path incomplete");
}
    executor.cancel();

if (spinner.joinable())
{
    spinner.join();
}

rclcpp::shutdown();
return 0;
}
