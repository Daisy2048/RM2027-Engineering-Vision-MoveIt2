/**
 * @file 2_joint_impedance_control.cpp
 * @brief 关节空间阻抗控制（带重力补偿）
 *
 * 实现关节空间阻抗控制：τ = K*(q_des - q) + B*(v_des - v) + G(q)
 * 其中：
 *   - K: 刚度系数矩阵（对角阵）
 *   - B: 阻尼系数矩阵（对角阵）
 *   - G(q): 重力补偿力矩
 *   - q_des: 期望关节位置
 *   - v_des: 期望关节速度（通常为0）
 *
 * 注意：本程序需要 Pinocchio 库支持以计算重力补偿力矩
 */

#include "panthera/Panthera.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <cmath>
#include <algorithm>

// Pinocchio 头文件
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

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
 * @brief 计算重力补偿力矩
 * @param model Pinocchio 模型
 * @param data Pinocchio 数据
 * @param q 当前关节位置
 * @return 重力补偿力矩向量
 */
std::vector<double> computeGravityCompensation(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const std::vector<double>& q)
{
    // 将 std::vector 转换为 Eigen::VectorXd
    Eigen::VectorXd q_eigen(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        q_eigen[i] = q[i];
    }

    // 创建零速度和零加速度向量
    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(q.size());
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(q.size());

    // 使用 RNEA 算法计算重力力矩（速度和加速度为零时，只剩重力项）
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q_eigen, v_zero, a_zero);

    // 转换回 std::vector
    std::vector<double> gravity_torque(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        gravity_torque[i] = tau[i];
    }

    return gravity_torque;
}

/**
 * @brief 限制力矩幅值
 * @param torque 输入力矩
 * @param max_torque 最大力矩限制
 * @return 限幅后的力矩
 */
std::vector<double> clipTorque(const std::vector<double>& torque,
                                const std::vector<double>& max_torque)
{
    std::vector<double> clipped(torque.size());
    for (size_t i = 0; i < torque.size(); ++i) {
        clipped[i] = std::max(-max_torque[i], std::min(max_torque[i], torque[i]));
    }
    return clipped;
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

        // 加载 URDF 模型用于重力补偿计算
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("panthera_ht_ros_description") +
            "/urdf/panthera_ht_ros_description.urdf";
        if (argc > 2) {
            urdf_path = argv[2];
        }

        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        std::cout << "URDF 模型加载成功: " << urdf_path << std::endl;
        std::cout << "模型包含 " << model.njoints - 1 << " 个关节（不含base）" << std::endl;

        // ==================== 控制参数设置 ====================

        // 刚度系数（Nm/rad）- 控制位置跟踪的强度
        std::vector<double> K = {4.0, 10.0, 10.0, 2.0, 2.0, 1.0, 0.5};

        // 阻尼系数（Nm·s/rad）- 控制速度阻尼的强度
        std::vector<double> B = {0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.05};

        // 期望目标位置（rad）
        std::vector<double> q_des = {0.0, 0.7, 0.7, -0.1, 0.0, 0.0, 0.0};

        // 期望目标速度（rad/s）- 通常为零
        std::vector<double> v_des(robot.getMotorCount(), 0.0);

        // 力矩限幅（基于电机规格）
        std::vector<double> tau_limit = {10.0, 20.0, 20.0, 10.0, 5.0, 5.0, 2.0};

        // 零位置、零速度、零增益（用于纯力矩控制）
        std::vector<double> zero_pos(robot.getMotorCount(), 0.0);
        std::vector<double> zero_vel(robot.getMotorCount(), 0.0);
        std::vector<double> zero_kp(robot.getMotorCount(), 0.0);
        std::vector<double> zero_kd(robot.getMotorCount(), 0.0);

        // ==================== 提示信息 ====================

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "关节空间阻抗控制（带重力补偿）" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\n控制参数：" << std::endl;
        std::cout << "刚度系数 K: [";
        for (size_t i = 0; i < K.size(); ++i) {
            std::cout << K[i] << (i < K.size() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        std::cout << "阻尼系数 B: [";
        for (size_t i = 0; i < B.size(); ++i) {
            std::cout << B[i] << (i < B.size() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        std::cout << "期望位置 q_des: [";
        for (size_t i = 0; i < q_des.size(); ++i) {
            std::cout << q_des[i] << (i < q_des.size() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        std::cout << "\n提示：" << std::endl;
        std::cout << "- 机械臂将保持在期望位置，可以手动推动感受阻抗效果" << std::endl;
        std::cout << "- 按 Ctrl+C 退出程序" << std::endl;
        std::cout << "- 结束后电机会自动掉电，请注意安全！" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;

        // 等待1秒
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // ==================== 主控制循环 ====================

        auto last_print_time = std::chrono::steady_clock::now();
        const double print_interval = 0.5; // 打印间隔（秒）

        while (keep_running) {
            // 1. 获取当前关节状态
            robot.send_get_motor_state_cmd();
            robot.motor_send_cmd();

            auto q_current = robot.getCurrentPos();
            auto v_current = robot.getCurrentVel();

            // 2. 计算阻抗控制力矩
            std::vector<double> tor_impedance(robot.getMotorCount());
            for (int i = 0; i < robot.getMotorCount(); ++i) {
                tor_impedance[i] = K[i] * (q_des[i] - q_current[i]) +
                                   B[i] * (v_des[i] - v_current[i]);
            }

            // 3. 计算重力补偿力矩
            // 将关节位置扩展到model.nq个（添加缺失的关节，设为0）
            std::vector<double> q_full = q_current;
            while ((int)q_full.size() < model.nq) {
                q_full.push_back(0.0);  // 添加缺失的关节（如joint7, gripper等）
            }

            // 调试：打印向量大小（首次循环）
            static bool first_print = true;
            if (first_print) {
                std::cout << "调试信息: model.nq=" << model.nq
                          << ", q_current.size()=" << q_current.size()
                          << ", q_full.size()=" << q_full.size() << std::endl;
                first_print = false;
            }

            std::vector<double> G = computeGravityCompensation(model, data, q_full);
            // 只使用实际电机数量的重力补偿
            G.resize(robot.getMotorCount());

            // 4. 总力矩 = 阻抗力矩 + 重力补偿力矩
            std::vector<double> tor_total(robot.getMotorCount());
            for (int i = 0; i < robot.getMotorCount(); ++i) {
                tor_total[i] = tor_impedance[i] + G[i];
            }

            // 5. 力矩限幅
            tor_total = clipTorque(tor_total, tau_limit);

            // 6. 发送控制命令（纯力矩控制模式：位置、速度、Kp、Kd 都为零）
            robot.posVelTorqueKpKd(zero_pos, zero_vel, tor_total, zero_kp, zero_kd);

            // 7. 定期打印状态信息
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - last_print_time).count();

            if (elapsed >= print_interval) {
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "\n位置误差: [";
                for (int i = 0; i < robot.getMotorCount(); ++i) {
                    std::cout << std::setw(7) << (q_des[i] - q_current[i])
                              << (i < robot.getMotorCount() - 1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "阻抗力矩: [";
                for (int i = 0; i < robot.getMotorCount(); ++i) {
                    std::cout << std::setw(7) << tor_impedance[i]
                              << (i < robot.getMotorCount() - 1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "重力力矩: [";
                for (int i = 0; i < robot.getMotorCount(); ++i) {
                    std::cout << std::setw(7) << G[i]
                              << (i < robot.getMotorCount() - 1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "总力矩:   [";
                for (int i = 0; i < robot.getMotorCount(); ++i) {
                    std::cout << std::setw(7) << tor_total[i]
                              << (i < robot.getMotorCount() - 1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                last_print_time = current_time;
            }

            // 控制频率：200Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::cout << "\n\n所有电机已停止" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
