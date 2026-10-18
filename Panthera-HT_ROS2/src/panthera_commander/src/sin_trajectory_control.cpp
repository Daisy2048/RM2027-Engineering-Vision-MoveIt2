/**
 * @file sin_trajectory_control.cpp
 * @brief 正弦轨迹跟踪控制程序
 *
 * 机器人关节沿着正弦函数轨迹运动
 */

#include "panthera/Panthera.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <vector>
#include <cmath>
#include <numeric>

// 全局标志，用于优雅退出
volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        keep_running = 0;
        std::cout << "\n\n程序被中断" << std::endl;
    }
}

int main(int argc, char** argv)
{
    try
    {
        // 注册信号处理器
        signal(SIGINT, signal_handler);

        // 创建机械臂对象
        std::string config_path = "../robot_param/Follower_absolute.yaml";
        if (argc > 1)
        {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        // 检查机械臂初始化是否成功
        int motor_count = robot.getMotorCount();
        if (motor_count <= 0)
        {
            std::cerr << "错误: 机械臂初始化失败！检测到 " << motor_count << " 个电机。" << std::endl;
            std::cerr << "请检查:" << std::endl;
            std::cerr << "  1. 机械臂电源是否已打开" << std::endl;
            std::cerr << "  2. USB连接是否正常 (/dev/ttyACM0-6)" << std::endl;
            std::cerr << "  3. 配置文件路径是否正确" << std::endl;
            std::cerr << "  4. 串口是否被其他程序占用" << std::endl;
            return 1;
        }

        // 先移动到安全的初始位置
        std::cout << "移动到初始位置..." << std::endl;
        std::vector<double> zero_pos(motor_count, 0.0);
        std::vector<double> init_pos = {-0.3, 1.1, 1.1, 0.2, -0.3, 0.0};
        std::vector<double> vel(motor_count, 0.5);
        std::vector<double> max_torque(motor_count, 10.0);

        robot.posVelMaxTorque(zero_pos, vel, max_torque);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        robot.posVelMaxTorque(init_pos, vel, max_torque);
        std::cout << "到达初始位置" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // ========== 正弦轨迹控制参数 ==========
        const double frequency = 0.45;       // Hz，正弦波频率（可调节：0.1-2.0 Hz）
        const double duration = 600.0;       // 运动持续时间（秒）
        const int control_rate = 500;        // 控制频率 Hz
        const double dt = 1.0 / control_rate;

        // 定义各关节角度限制（弧度）
        std::vector<std::pair<double, double>> joint_limits = {
            {-M_PI, M_PI},              // 关节1：±180度
            {0.0, M_PI},                // 关节2：0到180度
            {0.0, M_PI},                // 关节3：0到180度
            {0.0, M_PI / 2.0},          // 关节4：0到90度
            {-M_PI / 2.0, M_PI / 2.0},  // 关节5：±90度
            {-M_PI, M_PI}               // 关节6：±180度
        };

        // 获取初始位置作为中心位置
        std::cout << "获取初始位置..." << std::endl;
        std::vector<double> center_pos = robot.getCurrentPos();
        std::cout << "中心位置: [";
        for (size_t i = 0; i < center_pos.size(); ++i)
        {
            std::cout << std::fixed << std::setprecision(3) << center_pos[i];
            if (i < center_pos.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // 检查初始位置是否在限制范围内
        for (size_t i = 0; i < center_pos.size(); ++i)
        {
            if (center_pos[i] < joint_limits[i].first || center_pos[i] > joint_limits[i].second)
            {
                std::cout << "警告: 关节" << (i + 1) << "初始位置 " << center_pos[i]
                          << " 超出限制范围 [" << joint_limits[i].first
                          << ", " << joint_limits[i].second << "]" << std::endl;
            }
        }

        // 计算安全振幅（避免超限）
        std::vector<double> safe_amplitudes(center_pos.size());
        std::vector<double> preset_amplitudes = {0.4, 0.6, 0.6, 0.5, 0.4, 0.0};
        for (size_t i = 0; i < center_pos.size(); ++i)
        {
            double dist_to_upper = joint_limits[i].second - center_pos[i];
            double dist_to_lower = center_pos[i] - joint_limits[i].first;
            safe_amplitudes[i] = std::min(dist_to_upper, dist_to_lower) * 0.8;
        }

        // 选择较小的振幅
        std::vector<double> amplitudes(center_pos.size());
        for (size_t i = 0; i < center_pos.size(); ++i)
        {
            amplitudes[i] = std::min(safe_amplitudes[i], preset_amplitudes[i]);
        }

        std::cout << "调整后的振幅: [";
        for (size_t i = 0; i < amplitudes.size(); ++i)
        {
            std::cout << std::fixed << std::setprecision(3) << amplitudes[i];
            if (i < amplitudes.size() - 1) std::cout << ", ";
        }
        std::cout << "] rad" << std::endl;

        // 计算并显示最大速度
        std::vector<double> max_velocities(center_pos.size());
        double omega = 2.0 * M_PI * frequency;
        for (size_t i = 0; i < amplitudes.size(); ++i)
        {
            max_velocities[i] = amplitudes[i] * omega;
        }

        std::cout << "各关节最大速度: [";
        for (size_t i = 0; i < max_velocities.size(); ++i)
        {
            std::cout << std::fixed << std::setprecision(3) << max_velocities[i];
            if (i < max_velocities.size() - 1) std::cout << ", ";
        }
        std::cout << "] rad/s" << std::endl;

        // 设置相位偏移（可以让各关节运动不同步）
        std::vector<double> phase_offsets(center_pos.size(), 0.0);  // 零相位偏移

        std::cout << "\n开始正弦轨迹运动..." << std::endl;
        std::cout << "频率: " << frequency << " Hz, 持续时间: " << duration << " 秒" << std::endl;

        auto start_time = std::chrono::steady_clock::now();
        int step = 0;

        // 主控制循环
        while (keep_running)
        {
            auto loop_start = std::chrono::steady_clock::now();
            auto current_chrono = std::chrono::steady_clock::now() - start_time;
            double current_time = std::chrono::duration<double>(current_chrono).count();

            if (current_time >= duration)
            {
                break;
            }

            // 计算正弦轨迹
            omega = 2.0 * M_PI * frequency;

            // 位置：x = x0 + A * sin(ωt + φ)
            // 速度（位置的导数）：v = A * ω * cos(ωt + φ)
            std::vector<double> pos(center_pos.size());
            std::vector<double> vel(center_pos.size());

            for (size_t i = 0; i < center_pos.size(); ++i)
            {
                double sin_val = std::sin(omega * current_time + phase_offsets[i]);
                double cos_val = std::cos(omega * current_time + phase_offsets[i]);

                pos[i] = center_pos[i] + amplitudes[i] * sin_val;
                vel[i] = amplitudes[i] * omega * cos_val;

                // 角度限幅
                if (pos[i] < joint_limits[i].first)
                {
                    pos[i] = joint_limits[i].first;
                    vel[i] = 0.0;
                }
                else if (pos[i] > joint_limits[i].second)
                {
                    pos[i] = joint_limits[i].second;
                    vel[i] = 0.0;
                }
            }

            // 发送控制命令
            robot.posVelMaxTorque(pos, vel, max_torque);

            // 定期打印状态（每50步，约0.5秒）
            if (step % 50 == 0)
            {
                std::cout << "\r时间: " << std::fixed << std::setprecision(2) << current_time << "s | "
                          << "关节1位置: " << std::setprecision(3) << pos[0] << " | "
                          << "关节2位置: " << pos[1] << " | "
                          << "关节3位置: " << pos[2] << std::flush;
            }

            step++;

            // 控制循环频率
            auto loop_end = std::chrono::steady_clock::now();
            auto loop_duration = std::chrono::duration<double>(loop_end - loop_start).count();
            if (loop_duration < dt)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(dt - loop_duration));
            }
        }

        // 返回中心位置
        std::cout << "\n\n返回中心位置..." << std::endl;
        std::vector<double> return_vel(center_pos.size(), 0.5);
        robot.posVelMaxTorque(center_pos, return_vel, max_torque);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 返回零位
        std::cout << "返回零位..." << std::endl;
        robot.posVelMaxTorque(zero_pos, vel, max_torque);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "运动完成" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n所有电机已停止" << std::endl;
    return 0;
}
