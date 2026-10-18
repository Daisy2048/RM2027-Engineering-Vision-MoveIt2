/**
 * @file 0_robot_get_state.cpp
 * @brief 获取并打印机械臂关节角度信息
 *
 * 实时显示6个关节和夹爪的当前状态
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

/**
 * @brief 打印机器人状态信息
 * @param robot Panthera机械臂对象引用
 */
void printRobotState(panthera::Panthera& robot)
{
    // 获取关节角度
    robot.send_get_motor_state_cmd();
    robot.motor_send_cmd();

    auto positions = robot.getCurrentPos();
    auto velocities = robot.getCurrentVel();
    auto torques = robot.getCurrentTorque();

    // 获取夹爪状态
    double gripper_pos = robot.getCurrentPosGripper();
    double gripper_vel = robot.getCurrentVelGripper();

    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "机械臂状态信息" << std::endl;
    std::cout << std::string(50, '=') << std::endl;

    // 打印关节信息
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < robot.getMotorCount(); ++i) {
        std::cout << "关节" << (i + 1) << ": "
                  << "位置=" << std::setw(7) << positions[i] << " rad, "
                  << "速度=" << std::setw(7) << velocities[i] << " rad/s, "
                  << "力矩=" << std::setw(7) << torques[i] << std::endl;
    }

    // 打印夹爪信息
    std::cout << "夹爪:   "
              << "位置=" << std::setw(7) << gripper_pos << " rad, "
              << "速度=" << std::setw(7) << gripper_vel << " rad/s" << std::endl;
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

        std::cout << "开始获取机器人状态，按 Ctrl+C 退出..." << std::endl;

        // 等待1秒
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 主循环：每0.5秒更新一次
        while (keep_running) {
            printRobotState(robot);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
