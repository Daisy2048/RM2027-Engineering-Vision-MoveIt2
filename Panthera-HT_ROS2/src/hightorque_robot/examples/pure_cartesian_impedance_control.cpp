/**
 * @file pure_cartesian_impedance_control.cpp
 * @brief 纯粹的笛卡尔空间阻抗控制（无关节空间约束）
 *
 * 纯笛卡尔空间阻抗控制的特点：
 *   1. 只对末端位姿施加阻抗控制，不约束关节空间配置
 *   2. 允许零空间运动：关节可以在不影响末端位姿的情况下自由运动
 *   3. 对末端施加外力时，末端会偏离；撤去外力后，末端会回到期望位置
 *   4. 对机械臂侧边（非末端）施加外力时，关节会移动但末端会尽量保持在期望位置
 *
 * 控制律：
 *   F = K*(x_des - x) + B*(v_des - v)
 *   τ = J^T * F + G(q)
 *
 * 其中：
 *   - K: 笛卡尔空间刚度矩阵（6x6对角阵）
 *   - B: 笛卡尔空间阻尼矩阵（6x6对角阵）
 *   - F: 笛卡尔空间力/力矩（6维向量）
 *   - J: 雅可比矩阵
 *   - G(q): 重力补偿力矩
 *   - 注意：没有关节空间的刚度项！
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
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

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
 */
std::vector<double> computeGravityCompensation(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const std::vector<double>& q)
{
    Eigen::VectorXd q_eigen(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        q_eigen[i] = q[i];
    }

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(q.size());
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(q.size());

    Eigen::VectorXd tau = pinocchio::rnea(model, data, q_eigen, v_zero, a_zero);

    std::vector<double> gravity_torque(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        gravity_torque[i] = tau[i];
    }

    return gravity_torque;
}

/**
 * @brief 限制力矩幅值
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

/**
 * @brief 将旋转矩阵转换为欧拉角（ZYX顺序）
 */
Eigen::Vector3d rotationMatrixToEulerZYX(const Eigen::Matrix3d& R)
{
    Eigen::Vector3d euler;
    euler(1) = std::asin(-R(2, 0));  // pitch

    if (std::abs(std::cos(euler(1))) > 1e-6) {
        euler(0) = std::atan2(R(2, 1), R(2, 2));  // roll
        euler(2) = std::atan2(R(1, 0), R(0, 0));  // yaw
    } else {
        euler(0) = 0.0;
        euler(2) = std::atan2(-R(0, 1), R(1, 1));
    }

    return euler;
}

/**
 * @brief 计算姿态误差（使用角度差）
 */
Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& R_des,
                                         const Eigen::Matrix3d& R_current)
{
    // 计算旋转误差矩阵
    Eigen::Matrix3d R_error = R_current.transpose() * R_des;

    // 转换为轴角表示
    Eigen::AngleAxisd angle_axis(R_error);

    // 获取误差向量（在当前末端坐标系中）
    Eigen::Vector3d error_local = angle_axis.angle() * angle_axis.axis();

    // 转换到世界坐标系
    return R_current * error_local;
}

int main(int argc, char** argv)
{
    try {
        // 注册信号处理器
        signal(SIGINT, signal_handler);

        // 创建机械臂对象
        std::string config_path =
            ament_index_cpp::get_package_share_directory("hightorque_robot") +
            "/robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        // 加载 URDF 模型
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
        std::cout << "模型自由度 nq = " << model.nq << ", nv = " << model.nv << std::endl;

        int num_arm_joints = robot.getMotorCount();
        std::cout << "机械臂可控关节数: " << num_arm_joints << std::endl;

        // 查找末端执行器的frame ID
        std::string ee_frame_name = "link6";
        if (!model.existFrame(ee_frame_name)) {
            std::cerr << "错误：未找到末端执行器frame: " << ee_frame_name << std::endl;
            return 1;
        }
        pinocchio::FrameIndex ee_frame_id = model.getFrameId(ee_frame_name);
        std::cout << "末端执行器frame: " << ee_frame_name << " (ID: " << ee_frame_id << ")" << std::endl;

        // ==================== 控制参数设置 ====================

        // 笛卡尔空间刚度系数。位置通道略加强，提高末端回到期望点的能力；
        // 姿态通道保持保守，避免再次激励 5/6 号腕部电机抖动。
        Eigen::Matrix<double, 6, 1> K_cartesian;
        K_cartesian << 140.0, 140.0, 140.0,  // 位置刚度 (N/m)
                       1.4, 1.4, 0.9;         // 姿态刚度 (Nm/rad)

        // 刚度提高后同步提高阻尼，减少回弹过冲。
        Eigen::Matrix<double, 6, 1> B_cartesian;
        B_cartesian << 7.0, 7.0, 7.0,        // 线速度阻尼 (N·s/m)
                       0.5, 0.5, 0.35;       // 角速度阻尼 (Nm·s/rad)

        // 力矩限幅（6个关节）
        std::vector<double> tau_limit = {10.0, 20.0, 20.0, 10.0, 3.0, 2.0};
        const double max_ori_error = 0.5;

        // 零位置、零速度、零增益（纯力矩控制）
        std::vector<double> zero_pos(robot.getMotorCount(), 0.0);
        std::vector<double> zero_vel(robot.getMotorCount(), 0.0);
        std::vector<double> zero_kp(robot.getMotorCount(), 0.0);
        std::vector<double> zero_kd(robot.getMotorCount(), 0.0);

        // ==================== 获取当前末端位置作为期望位置 ====================

        std::cout << "\n正在获取当前末端位置作为期望位置..." << std::endl;
        std::cout << "请确保机械臂处于合适的初始位置！" << std::endl;
        std::cout << "3秒后开始..." << std::endl;

        for (int i = 3; i > 0; --i) {
            std::cout << i << "..." << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "开始！\n" << std::endl;

        // 获取当前关节位置
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        auto q_initial = robot.getCurrentPos();

        // 转换为 Eigen 向量
        Eigen::VectorXd q_eigen = Eigen::VectorXd::Zero(model.nq);
        for (size_t i = 0; i < q_initial.size() && i < model.nq; ++i) {
            q_eigen[i] = q_initial[i];
        }

        // 计算当前末端位姿作为期望位姿
        pinocchio::forwardKinematics(model, data, q_eigen);
        pinocchio::updateFramePlacements(model, data);

        pinocchio::SE3 ee_pose = data.oMf[ee_frame_id];
        Eigen::Vector3d x_des = ee_pose.translation();
        Eigen::Matrix3d R_des = ee_pose.rotation();

        std::cout << "期望末端位置 (m): [" << x_des.transpose() << "]" << std::endl;
        Eigen::Vector3d euler_des = rotationMatrixToEulerZYX(R_des);
        std::cout << "期望末端姿态 (rad): [" << euler_des.transpose() << "]" << std::endl;

        // ==================== 提示信息 ====================

        std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "纯粹的笛卡尔空间阻抗控制" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "\n控制参数：" << std::endl;
        std::cout << "位置刚度 K_pos (N/m): [" << K_cartesian.head<3>().transpose() << "]" << std::endl;
        std::cout << "姿态刚度 K_ori (Nm/rad): [" << K_cartesian.tail<3>().transpose() << "]" << std::endl;
        std::cout << "线速度阻尼 B_vel (N·s/m): [" << B_cartesian.head<3>().transpose() << "]" << std::endl;
        std::cout << "角速度阻尼 B_omega (Nm·s/rad): [" << B_cartesian.tail<3>().transpose() << "]" << std::endl;
        std::cout << "关节力矩限幅 (Nm): [";
        for (size_t i = 0; i < tau_limit.size(); ++i) {
            std::cout << tau_limit[i] << (i + 1 < tau_limit.size() ? " " : "");
        }
        std::cout << "]" << std::endl;

        std::cout << "\n纯笛卡尔空间阻抗控制的特点：" << std::endl;
        std::cout << "1. 对末端施加外力：末端会偏离期望位置，撤去外力后会回到原位" << std::endl;
        std::cout << "2. 对机械臂侧边施加外力：关节会移动，但末端会尽量保持在期望位置" << std::endl;
        std::cout << "3. 轴1不会自动回到初始角度（这是纯笛卡尔控制的正常现象）" << std::endl;
        std::cout << "4. 零空间运动自由：在不影响末端位姿的情况下，关节可以自由配置" << std::endl;
        std::cout << "\n按 Ctrl+C 退出程序" << std::endl;
        std::cout << "结束后电机会自动掉电，请注意安全！" << std::endl;
        std::cout << std::string(70, '=') << "\n" << std::endl;

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

            // 转换为 Eigen 向量
            Eigen::VectorXd q_eigen = Eigen::VectorXd::Zero(model.nq);
            Eigen::VectorXd v_eigen = Eigen::VectorXd::Zero(model.nv);
            for (size_t i = 0; i < q_current.size() && i < model.nq; ++i) {
                q_eigen[i] = q_current[i];
                v_eigen[i] = v_current[i];
            }

            // 2. 计算正运动学和雅可比矩阵
            pinocchio::forwardKinematics(model, data, q_eigen);
            pinocchio::updateFramePlacements(model, data);

            // 计算末端执行器的雅可比矩阵（世界坐标系）
            Eigen::MatrixXd J(6, model.nv);
            J.setZero();
            pinocchio::computeFrameJacobian(model, data, q_eigen, ee_frame_id,
                                           pinocchio::LOCAL_WORLD_ALIGNED, J);

            // 获取当前末端位姿
            pinocchio::SE3 ee_pose_current = data.oMf[ee_frame_id];
            Eigen::Vector3d x_current = ee_pose_current.translation();
            Eigen::Matrix3d R_current = ee_pose_current.rotation();

            // 计算末端速度（线速度和角速度）
            Eigen::Matrix<double, 6, 1> v_ee = J * v_eigen;

            // 3. 计算笛卡尔空间误差（只针对末端位姿）
            Eigen::Vector3d pos_error = x_des - x_current;
            Eigen::Vector3d ori_error = computeOrientationError(R_des, R_current);

            if (ori_error.norm() > max_ori_error) {
                ori_error = ori_error.normalized() * max_ori_error;
            }

            // 组合位置和姿态误差
            Eigen::Matrix<double, 6, 1> x_error;
            x_error << pos_error, ori_error;

            // 4. 计算笛卡尔空间阻抗力/力矩（纯笛卡尔空间控制）
            // F = K * (x_des - x) - B * v_ee
            // 注意：这里没有任何关节空间的刚度项！
            Eigen::Matrix<double, 6, 1> F_cartesian;
            for (int i = 0; i < 6; ++i) {
                F_cartesian(i) = K_cartesian(i) * x_error(i) - B_cartesian(i) * v_ee(i);
            }

            // 5. 转换为关节力矩（使用雅可比转置）
            // τ_impedance = J^T * F
            Eigen::VectorXd tau_impedance = J.transpose() * F_cartesian;

            // 6. 计算重力补偿力矩
            std::vector<double> q_full(model.nq);
            for (int i = 0; i < model.nq; ++i) {
                q_full[i] = q_eigen[i];
            }
            std::vector<double> G = computeGravityCompensation(model, data, q_full);

            // 7. 总力矩 = 笛卡尔阻抗力矩 + 重力补偿
            // 注意：没有关节空间的回复力矩！
            std::vector<double> tor_total(robot.getMotorCount());
            for (int i = 0; i < robot.getMotorCount(); ++i) {
                tor_total[i] = tau_impedance(i) + G[i];
            }

            // 8. 力矩限幅。5/6号腕部电机限幅更低，减少姿态通道激励。
            tor_total = clipTorque(tor_total, tau_limit);

            // 9. 发送控制命令（纯力矩控制模式）
            robot.posVelTorqueKpKd(zero_pos, zero_vel, tor_total, zero_kp, zero_kd);

            // 10. 定期打印状态信息
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - last_print_time).count();

            if (elapsed >= print_interval) {
                std::cout << std::fixed << std::setprecision(4);

                std::cout << "\n当前末端位置 (m): [" << x_current.transpose() << "]" << std::endl;
                std::cout << "位置误差 (m):     [" << pos_error.transpose() << "]" << std::endl;
                std::cout << "位置误差范数:     " << pos_error.norm() << " m" << std::endl;

                Eigen::Vector3d euler_current = rotationMatrixToEulerZYX(R_current);
                std::cout << "当前末端姿态 (rad): [" << euler_current.transpose() << "]" << std::endl;
                std::cout << "姿态误差 (rad):     [" << ori_error.transpose() << "]" << std::endl;
                std::cout << "姿态误差范数:       " << ori_error.norm() << " rad" << std::endl;

                std::cout << std::setprecision(3);
                std::cout << "笛卡尔力/力矩: [";
                for (int i = 0; i < 6; ++i) {
                    std::cout << std::setw(7) << F_cartesian(i)
                              << (i < 5 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "关节位置 (rad): [";
                for (int i = 0; i < robot.getMotorCount(); ++i) {
                    std::cout << std::setw(7) << q_current[i]
                              << (i < robot.getMotorCount() - 1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "关节力矩 (Nm):  [";
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
