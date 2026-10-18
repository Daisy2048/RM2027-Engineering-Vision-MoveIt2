/**
 * @file 1_PD_control.cpp
 * @brief 简单的六关节机器人PD控制程序
 *
 * 直接在代码中修改目标位置数组来控制机器人
 */

#include "panthera/Panthera.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>

// 全局标志，用于优雅退出
volatile sig_atomic_t keep_running = 1;

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = 0;
        std::cout << "\n\n程序被中断" << std::endl;
    }
}

int main(int argc, char** argv)
{
    try {
        // 注册信号处理器
        signal(SIGINT, signal_handler);

        // 创建机械臂对象
        std::string config_path = ament_index_cpp::get_package_share_directory("hightorque_robot") + "/robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        // 定义控制参数
        std::vector<double> zero_pos(robot.getMotorCount(), 0.0);
        std::vector<double> zero_vel(robot.getMotorCount(), 0.0);
        std::vector<double> zero_torque(robot.getMotorCount(), 0.0);

        // 目标位置
        std::vector<double> pos1 = {0.0, 0.7, 0.7, -0.1, 0.0, 0.0};

        // PD 增益
        std::vector<double> kp = {4.0, 10.0, 10.0, 2.0, 2.0, 1.0};
        std::vector<double> kd = {0.5, 0.8, 0.8, 0.2, 0.2, 0.1};

        // 最大力矩
        std::vector<double> max_torque = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0};

        std::cout << "开始 PD 控制，按 Ctrl+C 退出..." << std::endl;

        // 主循环
        while (keep_running) {
            // 发送控制命令
            robot.posVelTorqueKpKd(pos1, zero_vel, zero_torque, kp, kd);

            // 获取当前状态
            auto positions = robot.getCurrentPos();
            auto velocities = robot.getCurrentVel();
            auto torques = robot.getCurrentTorque();

            // 打印关节信息
            std::cout << std::fixed << std::setprecision(3);
            for (int i = 0; i < robot.getMotorCount(); ++i) {
                std::cout << "关节" << (i + 1) << ": "
                          << "位置=" << std::setw(7) << positions[i] << " rad, "
                          << "速度=" << std::setw(7) << velocities[i] << " rad/s, "
                          << "力矩=" << std::setw(7) << torques[i] << std::endl;
            }
            std::cout << std::string(60, '-') << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "\n所有电机已停止" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
