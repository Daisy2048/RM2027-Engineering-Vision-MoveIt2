/**
 * @file 1_PosVel_control.cpp
 * @brief 简单的六关节机器人位置速度控制程序
 *
 * 直接在代码中修改目标位置数组来控制机器人
 */

#include "panthera/Panthera.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    try {
        // 创建机械臂对象
        std::string config_path = ament_index_cpp::get_package_share_directory("hightorque_robot") + "/robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        // 定义控制参数
        int motor_count = robot.getMotorCount();
        std::vector<double> zero_pos(motor_count, 0.0);
        std::vector<double> pos1 = {0.0, 0.8, 0.8, 0.3, 0.0, 0.0};
        std::vector<double> pos2 = {0.0, 1.2, 1.2, 0.4, 0.0, 0.0};
        std::vector<double> pos3 = {0.0, 0.0, 0.0, 0.0, 0.0, 2.0};
        std::vector<double> vel(motor_count, 0.5);
        std::vector<double> max_torque = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0};

        std::cout << "开始位置速度控制..." << std::endl;

        // 发送位置控制命令（使用阻塞模式）
        std::cout << "\n移动到零位..." << std::endl;
        bool success = robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
        std::cout << "执行状态: " << (success ? "成功" : "失败") << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n移动到位置2..." << std::endl;
        robot.posVelMaxTorque(pos2, vel, max_torque, true);
        robot.gripperClose();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "\n移动到位置1..." << std::endl;
        robot.posVelMaxTorque(pos1, vel, max_torque, true);
        robot.gripperOpen();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "\n移动到位置2..." << std::endl;
        robot.posVelMaxTorque(pos2, vel, max_torque, true);
        robot.gripperClose();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "\n返回零位..." << std::endl;
        success = robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
        std::cout << "执行状态: " << (success ? "成功" : "失败") << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 保持位置2秒
        std::cout << "\n保持位置2秒..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "\n\n所有电机已停止" << std::endl;
        std::cout << "结束后电机会自动掉电，请注意安全！！" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
