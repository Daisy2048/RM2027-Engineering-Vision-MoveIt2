/**
 * @file sin_trajectory_moveit.cpp
 * @brief 使用MoveIt在笛卡尔空间中绘制正弦曲线轨迹
 *
 * 末端执行器在XYZ空间中按照正弦函数轨迹运动
 * 沿斜线方向往复绘制正弦曲线
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/executors.hpp>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("sin_trajectory_moveit", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt connections...");
    rclcpp::sleep_for(std::chrono::seconds(3));

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.3);

    // ========== 正弦轨迹参数 ==========
    const double amplitude = 0.03;      // 振幅：3cm
    const int num_cycles = 100;         // 循环次数
    const int num_points = 30;          // 每个周期的点数
    const double wave_length = 0.08;    // 波长：8cm

    // 定义斜线方向（单位向量）
    const double direction_angle = M_PI / 6.0;  // 30度
    double dir_x = std::cos(direction_angle);
    double dir_y = 0.0;
    double dir_z = std::sin(direction_angle);

    // 垂直向量（用于振荡方向）
    double perp_x = -dir_z;
    double perp_y = 0.0;
    double perp_z = dir_x;

    RCLCPP_INFO(node->get_logger(), "========== 笛卡尔空间正弦轨迹 ==========");
    RCLCPP_INFO(node->get_logger(), "振幅: %.3f m", amplitude);
    RCLCPP_INFO(node->get_logger(), "波长: %.3f m", wave_length);
    RCLCPP_INFO(node->get_logger(), "循环次数: %d", num_cycles);
    RCLCPP_INFO(node->get_logger(), "方向: 斜向（角度=%.1f度）", direction_angle * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "模式: 往复运动（画过去再画回来）");
    RCLCPP_INFO(node->get_logger(), "========================================");

    // Step 1: 移动到初始位置
    RCLCPP_INFO(node->get_logger(), "Step 1: 移动到初始位置...");
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pose1");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.execute(plan);
        RCLCPP_INFO(node->get_logger(), "到达初始位置");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to initial position");
        rclcpp::shutdown();
        return 1;
    }

    // Step 2: 获取并保存起始状态
    RCLCPP_INFO(node->get_logger(), "Step 2: 获取当前位姿...");

    const moveit::core::RobotModelConstPtr& robot_model = arm.getRobotModel();
    moveit::core::RobotState robot_state(robot_model);

    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm");
    if (!joint_model_group)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get joint model group 'arm'");
        rclcpp::shutdown();
        return 1;
    }

    if (!robot_state.setToDefaultValues(joint_model_group, "pose1"))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to set robot state to pose1");
        rclcpp::shutdown();
        return 1;
    }

    const std::string& end_effector_link = arm.getEndEffectorLink();
    const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform(end_effector_link);

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = end_effector_state.translation().x();
    start_pose.position.y = end_effector_state.translation().y();
    start_pose.position.z = end_effector_state.translation().z();

    Eigen::Quaterniond q(end_effector_state.rotation());
    start_pose.orientation.x = q.x();
    start_pose.orientation.y = q.y();
    start_pose.orientation.z = q.z();
    start_pose.orientation.w = q.w();

    // 保存起始关节位置
    std::vector<double> start_joint_positions;
    robot_state.copyJointGroupPositions(joint_model_group, start_joint_positions);

    RCLCPP_INFO(node->get_logger(), "起始位置: x=%.3f, y=%.3f, z=%.3f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z);

    // Step 3: 往复绘制正弦曲线
    RCLCPP_INFO(node->get_logger(), "Step 3: 开始往复绘制正弦曲线...");

    int wave_count = 0;
    bool forward = true;

    while (rclcpp::ok() && wave_count < num_cycles)
    {
        wave_count++;
        RCLCPP_INFO(node->get_logger(), "========== 绘制正弦波 #%d (%s) ==========",
                    wave_count, forward ? "正向" : "反向");

        // 每次都从已知的安全位置开始
        if (!forward)
        {
            // 反向运动前，先规划回起点
            RCLCPP_INFO(node->get_logger(), "反向运动：先规划回起点...");
            arm.setStartStateToCurrentState();
            arm.setJointValueTarget(start_joint_positions);

            moveit::planning_interface::MoveGroupInterface::Plan return_plan;
            if (arm.plan(return_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                arm.execute(return_plan);
                RCLCPP_INFO(node->get_logger(), "已返回起点");
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "无法返回起点，跳过此次循环");
                forward = !forward;
                continue;
            }
        }

        // 生成正弦曲线路点
        std::vector<geometry_msgs::msg::Pose> waypoints;

        for (int i = 0; i <= num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;  // 0到1
            double distance = t * wave_length;  // 沿方向移动的距离

            geometry_msgs::msg::Pose waypoint = start_pose;

            // 沿斜线方向移动
            waypoint.position.x = start_pose.position.x + dir_x * distance;
            waypoint.position.y = start_pose.position.y + dir_y * distance;
            waypoint.position.z = start_pose.position.z + dir_z * distance;

            // 添加正弦振荡
            double angle = 2.0 * M_PI * t;  // 0到2π
            double sin_offset = amplitude * std::sin(angle);

            waypoint.position.x += perp_x * sin_offset;
            waypoint.position.y += perp_y * sin_offset;
            waypoint.position.z += perp_z * sin_offset;

            // 姿态保持不变
            waypoint.orientation = start_pose.orientation;

            waypoints.push_back(waypoint);
        }

        RCLCPP_INFO(node->get_logger(), "生成了 %zu 个路点", waypoints.size());

        // 计算笛卡尔路径
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;  // 5mm分辨率
        const double jump_threshold = 0.0;  // Disable jump threshold
        const bool avoid_collisions = true;

        arm.setStartStateToCurrentState();
        double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

        RCLCPP_INFO(node->get_logger(), "笛卡尔路径规划完成度: %.3f", fraction);

        if (fraction > 0.5)
        {
            // 添加时间戳
            double time_from_start = 0.0;
            const double time_step = 0.05;

            for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
            {
                time_from_start += time_step;
                trajectory.joint_trajectory.points[i].time_from_start =
                    rclcpp::Duration::from_seconds(time_from_start);
            }

            RCLCPP_INFO(node->get_logger(), "执行正弦波轨迹（%zu个轨迹点）...",
                        trajectory.joint_trajectory.points.size());

            // 执行轨迹
            moveit::planning_interface::MoveGroupInterface::Plan sin_plan;
            sin_plan.trajectory_ = trajectory;

            auto result = arm.execute(sin_plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(node->get_logger(), "正弦波 #%d 完成!", wave_count);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "正弦波执行失败! 错误代码: %d", result.val);
                break;
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "笛卡尔路径规划失败! 完成度 = %.3f", fraction);
            RCLCPP_ERROR(node->get_logger(), "无法完成正弦波，停止...");
            break;
        }

        // 切换方向
        forward = !forward;

        // 短暂停顿
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(node->get_logger(), "停止绘制正弦曲线。总共绘制了 %d 个波形", wave_count);

    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}
