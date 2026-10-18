#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <array>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 1. 创建 ROS2 节点
    auto node = std::make_shared<rclcpp::Node>(
        "planner_bridge",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
    );

    // 2. 启动执行器，让 ROS2 能持续处理通信
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2
    );

    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

   // ===============================
// 读取规划参数
// ===============================

const std::string planning_group =
    node->get_parameter_or<std::string>(
        "planning_group", "arm");

const std::string tip_link =
    node->get_parameter_or<std::string>(
        "tip_link", "bat_center");

const double planning_time =
    node->get_parameter_or<double>(
        "planning_time", 10.0);

const int64_t planning_attempts =
    node->get_parameter_or<int64_t>(
        "planning_attempts", 10);

const double velocity_scaling =
    node->get_parameter_or<double>(
        "velocity_scaling", 0.1);

const double acceleration_scaling =
    node->get_parameter_or<double>(
        "acceleration_scaling", 0.1);

// 检查参数范围
if (!std::isfinite(planning_time) ||
    planning_time <= 0.0 ||
    planning_attempts < 1 ||
    planning_attempts > 100 ||
    !std::isfinite(velocity_scaling) ||
    velocity_scaling <= 0.0 ||
    velocity_scaling > 1.0 ||
    !std::isfinite(acceleration_scaling) ||
    acceleration_scaling <= 0.0 ||
    acceleration_scaling > 1.0)
{
    RCLCPP_ERROR(
        node->get_logger(),
        "Invalid MoveIt planning parameters"
    );

    executor.cancel();

    if (spinner.joinable())
    {
        spinner.join();
    }

    rclcpp::shutdown();
    return 1;
}

// ===============================
// 创建 MoveIt 规划接口
// ===============================

MoveGroupInterface move_group(node, planning_group);

move_group.setPlanningTime(planning_time);

move_group.setNumPlanningAttempts(
    static_cast<int>(planning_attempts));

move_group.setMaxVelocityScalingFactor(
    velocity_scaling);

move_group.setMaxAccelerationScalingFactor(
    acceleration_scaling);

// 打印当前配置
RCLCPP_INFO(
    node->get_logger(),
    "Planning parameters: time=%.2f, attempts=%ld, velocity=%.2f, acceleration=%.2f",
    planning_time,
    static_cast<long>(planning_attempts),
    velocity_scaling,
    acceleration_scaling
);
    const std::string planning_frame =
        move_group.getPlanningFrame();

    RCLCPP_INFO(
        node->get_logger(),
        "MoveIt planning group: %s",
        planning_group.c_str()
    );

    RCLCPP_INFO(
        node->get_logger(),
        "MoveIt planning frame: %s",
        planning_frame.c_str()
    );

    RCLCPP_INFO(
        node->get_logger(),
        "Target tip link: %s",
        tip_link.c_str()
    );

    // 4. 创建轨迹发布器，用于 RViz 可视化
    auto display_publisher =
        node->create_publisher<
            moveit_msgs::msg::DisplayTrajectory>(
                "/display_planned_path",
                rclcpp::QoS(1).reliable().transient_local()
            );

    // 5. 订阅模拟视觉目标位姿
    auto subscription =
        node->create_subscription<
            geometry_msgs::msg::PoseStamped>(
                "/vision/target_pose_base",
                10,
                [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Received target: x=%.3f y=%.3f z=%.3f",
                        msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z
                    );

                    // 检查坐标系是否匹配
                    if (msg->header.frame_id != planning_frame)
                    {
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "Frame mismatch: received '%s', expected '%s'",
                            msg->header.frame_id.c_str(),
                            planning_frame.c_str()
                        );
                        return;
                    }

                    // 检查数据是否为有限数值
                    const auto &p = msg->pose.position;
                    const auto &q = msg->pose.orientation;

                    const std::array<double, 7> values = {
                        p.x, p.y, p.z,
                        q.x, q.y, q.z, q.w
                    };

                    for (double value : values)
                    {
                        if (!std::isfinite(value))
                        {
                            RCLCPP_ERROR(
                                node->get_logger(),
                                "Invalid target: non-finite value"
                            );
                            return;
                        }
                    }

                    // 四元数必须接近单位长度
                    const double q_norm = std::sqrt(
                        q.x*q.x + q.y*q.y +
                        q.z*q.z + q.w*q.w
                    );

                    if (std::abs(q_norm - 1.0) > 0.01)
                    {
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "Invalid quaternion norm: %.6f",
                            q_norm
                        );
                        return;
                    }

                    // 6. 设置机械臂当前状态为规划起点
                    move_group.setStartStateToCurrentState();

                    // 7. 将目标位姿设置为 MoveIt 规划目标
                    if (!move_group.setPoseTarget(*msg, tip_link))
                    {
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "Failed to set pose target for '%s'",
                            tip_link.c_str()
                        );
                        return;
                    }

                    // 8. 进行 IK / 运动规划
                    MoveGroupInterface::Plan plan;

                    RCLCPP_INFO(
                        node->get_logger(),
                        "Starting motion planning..."
                    );

                    const bool success =
                        static_cast<bool>(move_group.plan(plan));

                    if (success)
                    {
                        RCLCPP_INFO(
                            node->get_logger(),
                            "Planning SUCCESS! Time: %.3f s",
                            plan.planning_time_
                        );
                        // ===============================
// 检查 MoveIt 六关节轨迹
// ===============================

const auto &jt = plan.trajectory_.joint_trajectory;

RCLCPP_INFO(
    node->get_logger(),
    "========== TRAJECTORY CHECK =========="
);

RCLCPP_INFO(
    node->get_logger(),
    "Trajectory points: %zu",
    jt.points.size()
);

if (jt.points.size() >= 2)
{
    const auto &start = jt.points.front().positions;
    const auto &end = jt.points.back().positions;

    if (start.size() != jt.joint_names.size() ||
        end.size() != jt.joint_names.size())
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Invalid trajectory joint dimensions"
        );
    }
    else
    {
        double max_delta = 0.0;

        for (size_t i = 0; i < jt.joint_names.size(); ++i)
        {
            double delta = end[i] - start[i];

            if (std::abs(delta) > max_delta)
            {
                max_delta = std::abs(delta);
            }

            RCLCPP_INFO(
                node->get_logger(),
                "%s: start=%.6f, end=%.6f, delta=%.6f rad",
                jt.joint_names[i].c_str(),
                start[i],
                end[i],
                delta
            );
        }

        const auto &last_time =
            jt.points.back().time_from_start;

        double duration =
            static_cast<double>(last_time.sec) +
            static_cast<double>(last_time.nanosec) * 1e-9;

        RCLCPP_INFO(
            node->get_logger(),
            "Trajectory duration: %.3f s",
            duration
        );

        RCLCPP_INFO(
            node->get_logger(),
            "Max joint displacement: %.6f rad",
            max_delta
        );

        if (max_delta > 0.001)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "NON-ZERO TRAJECTORY VERIFIED!"
            );
        }
        else
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Trajectory has no significant joint movement"
            );
        }
    }
}
else
{
    RCLCPP_WARN(
        node->get_logger(),
        "Trajectory contains fewer than 2 points"
    );
}

RCLCPP_INFO(
    node->get_logger(),
    "======================================"
);
                        // 9. 将规划轨迹发送到 RViz
                        moveit_msgs::msg::DisplayTrajectory display;

                        display.model_id =
                            move_group.getRobotModel()->getName();

                        display.trajectory_start = plan.start_state_;

                        display.trajectory.push_back(
                            plan.trajectory_
                        );

                        // 9. 发布规划轨迹，供 RViz 显示
display_publisher->publish(display);

RCLCPP_INFO(
    node->get_logger(),
    "Trajectory published to RViz"
);

// ===========================================
// 10. 可选：在模拟控制器上执行规划轨迹
// 默认关闭，必须明确开启 simulation_execute
// ===========================================

const bool simulation_execute =
    node->get_parameter_or(
        "simulation_execute", false
    );

if (simulation_execute)
{
    if (jt.points.size() < 2)
    {
        RCLCPP_WARN(
            node->get_logger(),
            "Simulation execution skipped: "
            "trajectory has fewer than 2 points."
        );
    }
    else
    {
        RCLCPP_WARN(
            node->get_logger(),
            "SIMULATION EXECUTION STARTING..."
        );

        // 将规划轨迹交给 MoveIt 执行接口
        // 本次只允许在 FakeSystem Demo 中启用
        const bool execution_success =
            static_cast<bool>(
                move_group.execute(plan)
            );

        if (execution_success)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "EXECUTION SUCCESS!"
            );
        }
        else
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "EXECUTION FAILED!"
            );
        }
    }
}
else
{
    RCLCPP_INFO(
        node->get_logger(),
        "Plan only: simulation execution disabled."
    );
}
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "Planning FAILED. Check IK, reachability and collision."
                        );
                    }

                    move_group.clearPoseTargets();

                    // 仿真执行默认关闭。
                    // 仅在确认使用 FakeSystem 模拟控制器时，
                    // 才允许开启 simulation_execute。
                }
            );

    RCLCPP_INFO(
        node->get_logger(),
        "Planner bridge ready. Waiting for /vision/target_pose_base"
    );

    // 10. 保持程序运行，直到用户按 Ctrl+C
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(200)
        );
    }

    executor.cancel();

    if (spinner.joinable())
    {
        spinner.join();
    }

    return 0;
}