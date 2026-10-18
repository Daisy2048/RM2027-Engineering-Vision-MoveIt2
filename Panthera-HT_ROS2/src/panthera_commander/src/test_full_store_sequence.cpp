#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include <memory>
#include <string>
#include <vector>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "test_full_store_sequence");

    auto arm =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface>(
                node, "arm");

    arm->setEndEffectorLink("bat_center");
   

    arm->setMaxVelocityScalingFactor(0.20);
    arm->setMaxAccelerationScalingFactor(0.20);
    arm->setPlanningTime(10.0);


    // ==================================================
    // 公共姿态
    //
    // q_front：正前方姿态（沿用旧版已经跑通的值）
    // q_left ：正前方姿态绕 base_link Z 轴 +90°，
    //          用于转向左侧矿仓
    //
    // 注意：
    // q_left 目前只是 RViz/mock 第一版候选姿态，
    // 并不是已经标定完成的真机姿态。
    // ==================================================

    geometry_msgs::msg::Quaternion front_orientation;

    front_orientation.x = 0.257;
    front_orientation.y = -0.048;
    front_orientation.z = -0.125;
    front_orientation.w = 0.957;

    geometry_msgs::msg::Quaternion left_orientation;

    left_orientation.x = 0.216;
    left_orientation.y = 0.148;
    left_orientation.z = 0.588;
    left_orientation.w = 0.765;


    // ==================================================
    // HOME_FOLDED_FRONT
    //
    // 折叠、面向正前方
    // 使用 joint target，不猜 Cartesian HOME 坐标
    // ==================================================

    std::vector<double> home_joint_values =
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    // ==================================================
    // READY_WITH_ORE / PICK_HIGH_FRONT
    //
    // 向正前方展开并升高
    // ==================================================

    geometry_msgs::msg::PoseStamped ready_pose;

    ready_pose.header.frame_id = "base_link";
    ready_pose.header.stamp = node->now();

    ready_pose.pose.position.x = 0.280;
    ready_pose.pose.position.y = 0.000;
    ready_pose.pose.position.z = 0.420;

    ready_pose.pose.orientation = front_orientation;


    // ==================================================
    // LEFT_PRE_STORE
    //
    // 保持较高位置，从正前方转向左侧矿仓
    // x/y 基本沿用旧版 PRE_STORE，z 由 0.400 改为 0.420
    // ==================================================

    geometry_msgs::msg::PoseStamped left_pre_store_pose;

    left_pre_store_pose.header.frame_id = "base_link";
    left_pre_store_pose.header.stamp = node->now();

    left_pre_store_pose.pose.position.x = 0.220;
    left_pre_store_pose.pose.position.y = 0.200;
    left_pre_store_pose.pose.position.z = 0.420;

    left_pre_store_pose.pose.orientation = left_orientation;


    // ==================================================
    // LEFT_APPROACH
    //
    // 左矿仓上方，准备插入
    // ==================================================

    geometry_msgs::msg::PoseStamped left_approach_pose;

    left_approach_pose.header.frame_id = "base_link";
    left_approach_pose.header.stamp = node->now();

    left_approach_pose.pose.position.x = 0.173;
    left_approach_pose.pose.position.y = 0.300;
    left_approach_pose.pose.position.z = 0.350;

    left_approach_pose.pose.orientation = left_orientation;


    // ==================================================
    // LEFT_STORE
    //
    // 与 LEFT_APPROACH 只差 z，x/y 与姿态完全一致
    // ==================================================

    geometry_msgs::msg::Pose left_store_pose =
        left_approach_pose.pose;

    left_store_pose.position.z = 0.180;


    // ==================================================
    // LEFT_RETREAT
    //
    // 严格等于 LEFT_APPROACH
    // ==================================================

    geometry_msgs::msg::Pose left_retreat_pose =
        left_approach_pose.pose;


    // ==================================================
    // 普通 MoveIt 点到点运动
    // READY / LEFT_PRE_STORE / LEFT_APPROACH 使用
    // ==================================================

    auto move_to_pose =
        [&](geometry_msgs::msg::PoseStamped target_pose,
            const std::string &name) -> bool
    {
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
    // joint target 运动
    //
    // 只用于 HOME_FOLDED_FRONT
    // ==================================================

    auto move_to_joint_home =
        [&](const std::string &name) -> bool
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Planning to %s (joint target)...",
            name.c_str());

        arm->setStartStateToCurrentState();
        arm->setJointValueTarget(home_joint_values);

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
    // 0. 当前状态 -> HOME_FOLDED_FRONT
    //
    // 保证流程从“折叠、面向正前方”开始，
    // 这样结束位置与开始位置一致。
    // 如果机械臂本来就已经在 HOME，允许继续。
    // ==================================================

    if (!move_to_joint_home("HOME_FOLDED_FRONT"))
    {
        RCLCPP_WARN(
            node->get_logger(),
            "Could not move to HOME_FOLDED_FRONT at start, "
            "assuming the arm is already folded in front");
    }


    // ==================================================
    // 1. HOME_FOLDED_FRONT -> READY_WITH_ORE
    // ==================================================

    if (!move_to_pose(
            ready_pose,
            "READY_WITH_ORE"))
    {
        rclcpp::shutdown();
        return 1;
    }


    // ==================================================
    // 2. READY_WITH_ORE -> LEFT_PRE_STORE
    // 高位转向左侧矿仓
    // ==================================================

    if (!move_to_pose(
            left_pre_store_pose,
            "LEFT_PRE_STORE"))
    {
        rclcpp::shutdown();
        return 1;
    }


    // ==================================================
    // 3. LEFT_PRE_STORE -> LEFT_APPROACH
    // ==================================================

    if (!move_to_pose(
            left_approach_pose,
            "LEFT_APPROACH"))
    {
        rclcpp::shutdown();
        return 1;
    }


    // ==================================================
    // 4. LEFT_APPROACH -> LEFT_STORE
    // Cartesian 直线下降
    // ==================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Computing LEFT_APPROACH -> LEFT_STORE Cartesian path...");

    arm->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose>
        store_waypoints;

    store_waypoints.push_back(left_store_pose);

    moveit_msgs::msg::RobotTrajectory
        store_trajectory;

    const double eef_step = 0.005;
    const double jump_threshold = 0.0;

    double store_fraction =
        arm->computeCartesianPath(
            store_waypoints,
            eef_step,
            jump_threshold,
            store_trajectory);

    RCLCPP_INFO(
        node->get_logger(),
        "LEFT_APPROACH -> LEFT_STORE Cartesian path fraction: %.1f%%",
        store_fraction * 100.0);

    if (store_fraction < 0.99)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "LEFT_APPROACH -> LEFT_STORE Cartesian path incomplete");

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Executing LEFT_APPROACH -> LEFT_STORE...");

    auto store_result =
        arm->execute(store_trajectory);

    if (store_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "LEFT_APPROACH -> LEFT_STORE execution failed");

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "LEFT_STORE reached");


    // ==================================================
    // 5. LEFT_STORE -> LEFT_RETREAT
    // Cartesian 直线上升
    // ==================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Computing LEFT_STORE -> LEFT_RETREAT Cartesian path...");

    arm->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose>
        retreat_waypoints;

    retreat_waypoints.push_back(left_retreat_pose);

    moveit_msgs::msg::RobotTrajectory
        retreat_trajectory;

    double retreat_fraction =
        arm->computeCartesianPath(
            retreat_waypoints,
            eef_step,
            jump_threshold,
            retreat_trajectory);

    RCLCPP_INFO(
        node->get_logger(),
        "LEFT_STORE -> LEFT_RETREAT Cartesian path fraction: %.1f%%",
        retreat_fraction * 100.0);

    if (retreat_fraction < 0.99)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "LEFT_STORE -> LEFT_RETREAT Cartesian path incomplete");

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Executing LEFT_STORE -> LEFT_RETREAT...");

    auto retreat_result =
        arm->execute(retreat_trajectory);

    if (retreat_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "LEFT_STORE -> LEFT_RETREAT execution failed");

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RETREAT reached");


    // ==================================================
    // 6. LEFT_RETREAT -> HOME_FOLDED_FRONT
    //
    // 回到最开始折叠、面向正前方的位置
    // joint1..joint6 = [0, 0, 0, 0, 0, 0]
    // ==================================================

    if (!move_to_joint_home("HOME_FOLDED_FRONT"))
    {
        rclcpp::shutdown();
        return 1;
    }


    RCLCPP_INFO(
        node->get_logger(),
        "FULL ARM SEQUENCE COMPLETED SUCCESSFULLY");

    RCLCPP_INFO(
        node->get_logger(),
        "HOME_FOLDED_FRONT -> READY_WITH_ORE -> LEFT_PRE_STORE "
        "-> LEFT_APPROACH -> LEFT_STORE -> LEFT_RETREAT "
        "-> HOME_FOLDED_FRONT");


    rclcpp::shutdown();

    return 0;
}
