#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <string>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "test_ready_pre_approach");

    auto arm =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface>(
                node, "arm");

    // 明确使用 bat_center 作为末端
    arm->setEndEffectorLink("bat_center");

    // 仿真阶段先慢一点
    arm->setMaxVelocityScalingFactor(0.20);
    arm->setMaxAccelerationScalingFactor(0.20);

    // 单次规划最多 10 秒
    arm->setPlanningTime(10.0);


    // ==================================================
    // 统一姿态
    // ==================================================

    geometry_msgs::msg::Quaternion orientation;

    orientation.x = 0.257;
    orientation.y = -0.048;
    orientation.z = -0.125;
    orientation.w = 0.957;


    // ==================================================
    // READY_WITH_ORE
    // ==================================================

    geometry_msgs::msg::PoseStamped ready_pose;

    ready_pose.header.frame_id = "base_link";
    ready_pose.header.stamp = node->now();

    ready_pose.pose.position.x = 0.280;
    ready_pose.pose.position.y = 0.000;
    ready_pose.pose.position.z = 0.420;

    ready_pose.pose.orientation = orientation;


    // ==================================================
    // PRE_STORE
    // ==================================================

    geometry_msgs::msg::PoseStamped pre_store_pose;

    pre_store_pose.header.frame_id = "base_link";
    pre_store_pose.header.stamp = node->now();

    pre_store_pose.pose.position.x = 0.220;
    pre_store_pose.pose.position.y = 0.200;
    pre_store_pose.pose.position.z = 0.400;

    pre_store_pose.pose.orientation = orientation;


    // ==================================================
    // APPROACH
    // ==================================================

    geometry_msgs::msg::PoseStamped approach_pose;

    approach_pose.header.frame_id = "base_link";
    approach_pose.header.stamp = node->now();

    approach_pose.pose.position.x = 0.173;
    approach_pose.pose.position.y = 0.300;
    approach_pose.pose.position.z = 0.350;

    approach_pose.pose.orientation = orientation;


    // ==================================================
    // 公共运动函数
    // ==================================================

    auto move_to_pose =
        [&](geometry_msgs::msg::PoseStamped target_pose,
            const std::string &name) -> bool
    {
        // 更新时间戳
        target_pose.header.stamp = node->now();

        RCLCPP_INFO(
            node->get_logger(),
            "Planning to %s...",
            name.c_str());

        arm->setStartStateToCurrentState();

        arm->setPoseTarget(target_pose);

        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        auto plan_result =
            arm->plan(plan);

        if (plan_result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Planning to %s failed, error code: %d",
                name.c_str(),
                plan_result.val);

            arm->clearPoseTargets();

            return false;
        }


        RCLCPP_INFO(
            node->get_logger(),
            "Planning to %s succeeded",
            name.c_str());


        RCLCPP_INFO(
            node->get_logger(),
            "Executing %s...",
            name.c_str());

        auto execute_result =
            arm->execute(plan);

        arm->clearPoseTargets();


        if (execute_result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Execution to %s failed, error code: %d",
                name.c_str(),
                execute_result.val);

            return false;
        }


        RCLCPP_INFO(
            node->get_logger(),
            "%s reached",
            name.c_str());

        return true;
    };


    // ==================================================
    // T2
    // ==================================================

    // 当前状态 -> READY_WITH_ORE
    if (!move_to_pose(
            ready_pose,
            "READY_WITH_ORE"))
    {
        rclcpp::shutdown();
        return 1;
    }


    // READY_WITH_ORE -> PRE_STORE
    if (!move_to_pose(
            pre_store_pose,
            "PRE_STORE"))
    {
        rclcpp::shutdown();
        return 1;
    }


    // PRE_STORE -> APPROACH
    if (!move_to_pose(
            approach_pose,
            "APPROACH"))
    {
        rclcpp::shutdown();
        return 1;
    }




    RCLCPP_INFO(
        node->get_logger(),
        "T2 completed successfully");




    rclcpp::shutdown();

    return 0;
}