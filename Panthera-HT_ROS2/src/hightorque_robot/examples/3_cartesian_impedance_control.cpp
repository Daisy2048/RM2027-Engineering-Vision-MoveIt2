/**
 * @file 3_cartesian_impedance_control.cpp
 * @brief 笛卡尔空间阻抗控制（带重力补偿）
 *
 * 实现笛卡尔空间阻抗控制：
 *   F = K*(x_des - x) + B*(v_des - v)
 *   τ = J^T * F + G(q)
 *
 * 其中：
 *   - K: 笛卡尔空间刚度系数矩阵（6x6对角阵，包括位置和姿态）
 *   - B: 笛卡尔空间阻尼系数矩阵（6x6对角阵）
 *   - F: 笛卡尔空间力/力矩（6维向量）
 *   - J: 雅可比矩阵
 *   - G(q): 重力补偿力矩
 *   - x_des: 期望末端位置和姿态
 *   - v_des: 期望末端速度（通常为0）
 *
 * 注意：本程序需要 Pinocchio 库支持以计算运动学和动力学
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

/**
 * @brief 将旋转矩阵转换为欧拉角（ZYX顺序）
 * @param R 旋转矩阵
 * @return 欧拉角向量 [roll, pitch, yaw]
 */
Eigen::Vector3d rotationMatrixToEulerZYX(const Eigen::Matrix3d& R)
{
    Eigen::Vector3d euler;

    // ZYX 欧拉角提取
    euler(1) = std::asin(-R(2, 0));  // pitch

    if (std::abs(std::cos(euler(1))) > 1e-6) {
        euler(0) = std::atan2(R(2, 1), R(2, 2));  // roll
        euler(2) = std::atan2(R(1, 0), R(0, 0));  // yaw
    } else {
        // 万向锁情况
        euler(0) = 0.0;
        euler(2) = std::atan2(-R(0, 1), R(1, 1));
    }

    return euler;
}

/**
 * @brief 计算姿态误差（使用角度差）
 * @param R_des 期望旋转矩阵
 * @param R_current 当前旋转矩阵
 * @return 姿态误差向量（3维，在世界坐标系中表达）
 */
Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& R_des,
                                         const Eigen::Matrix3d& R_current)
{
    // 计算旋转误差矩阵（在当前坐标系中表达）
    Eigen::Matrix3d R_error = R_current.transpose() * R_des;

    // 将旋转矩阵转换为轴角表示
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
        std::string config_path = ament_index_cpp::get_package_share_directory("hightorque_robot") + "/robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        // 加载 URDF 模型用于运动学和动力学计算
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
        std::cout << "模型自由度 nq = " << model.nq << ", nv = " << model.nv << std::endl;

        // 检查机器人电机数量与模型自由度
        int num_arm_joints = robot.getMotorCount();
        std::cout << "机械臂可控关节数: " << num_arm_joints << std::endl;

        if (model.nq != num_arm_joints) {
            std::cout << "注意：URDF模型包含额外关节（如手指），将使用固定值填充" << std::endl;
        }

        // 查找末端执行器的frame ID（通常是link6）
        std::string ee_frame_name = "link6";
        if (!model.existFrame(ee_frame_name)) {
            std::cerr << "错误：未找到末端执行器frame: " << ee_frame_name << std::endl;
            return 1;
        }
        pinocchio::FrameIndex ee_frame_id = model.getFrameId(ee_frame_name);
        std::cout << "末端执行器frame: " << ee_frame_name << " (ID: " << ee_frame_id << ")" << std::endl;

        // ==================== 控制参数设置 ====================

        // 笛卡尔空间刚度系数（前3个是位置，后3个是姿态）
        // 位置刚度单位：N/m，姿态刚度单位：Nm/rad
        Eigen::Matrix<double, 6, 1> K_cartesian;
        K_cartesian << 200.0, 200.0, 200.0,  // 位置刚度 - 适中的值
                       4.0, 4.0, 4.0;         // 姿态刚度 - 非常小的值

        // 笛卡尔空间阻尼系数（前3个是线速度，后3个是角速度）
        // 线速度阻尼单位：N·s/m，角速度阻尼单位：Nm·s/rad
        Eigen::Matrix<double, 6, 1> B_cartesian;
        B_cartesian << 5.0, 5.0, 5.0,     // 线速度阻尼
                       0.8, 0.8, 0.8;         // 角速度阻尼 - 适中的值

        // 力矩限幅（基于电机规格）
        std::vector<double> tau_limit = {10.0, 20.0, 20.0, 10.0, 5.0, 5.0};

        // 零位置、零速度、零增益（用于纯力矩控制）
        std::vector<double> zero_pos(robot.getMotorCount(), 0.0);
        std::vector<double> zero_vel(robot.getMotorCount(), 0.0);
        std::vector<double> zero_kp(robot.getMotorCount(), 0.0);
        std::vector<double> zero_kd(robot.getMotorCount(), 0.0);

        // ==================== 获取初始位置作为期望位置 ====================

        std::cout << "\n正在计算期望末端位置..." << std::endl;

        // 使用与关节空间阻抗控制相同的目标关节位置
        std::vector<double> q_des_joint = {0.0, 0.7, 0.7, -0.1, 0.0, 0.0};

        // 转换为 Eigen 向量，并扩展到模型的完整自由度
        Eigen::VectorXd q_eigen = Eigen::VectorXd::Zero(model.nq);
        for (size_t i = 0; i < q_des_joint.size() && i < model.nq; ++i) {
            q_eigen[i] = q_des_joint[i];
        }
        // 剩余的关节（如手指）保持为0

        // 计算正运动学
        pinocchio::forwardKinematics(model, data, q_eigen);
        pinocchio::updateFramePlacements(model, data);

        // 获取末端执行器的位姿
        pinocchio::SE3 ee_pose = data.oMf[ee_frame_id];
        Eigen::Vector3d x_des = ee_pose.translation();
        Eigen::Matrix3d R_des = ee_pose.rotation();

        std::cout << "期望末端位置 (m): [" << x_des.transpose() << "]" << std::endl;
        Eigen::Vector3d euler_des = rotationMatrixToEulerZYX(R_des);
        std::cout << "期望末端姿态 (rad): [" << euler_des.transpose() << "]" << std::endl;

        // 检查初始重力补偿
        std::vector<double> q_init(model.nq);
        for (int i = 0; i < model.nq; ++i) {
            q_init[i] = q_eigen[i];
        }
        std::vector<double> G_init = computeGravityCompensation(model, data, q_init);
        std::cout << "\n初始重力补偿力矩 (Nm): [";
        for (int i = 0; i < robot.getMotorCount(); ++i) {
            std::cout << std::setw(7) << std::setprecision(3) << G_init[i]
                      << (i < robot.getMotorCount() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        // ==================== 提示信息 ====================

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "笛卡尔空间阻抗控制（带重力补偿）" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\n控制参数：" << std::endl;
        std::cout << "位置刚度 K_pos (N/m): [" << K_cartesian.head<3>().transpose() << "]" << std::endl;
        std::cout << "姿态刚度 K_ori (Nm/rad): [" << K_cartesian.tail<3>().transpose() << "]" << std::endl;
        std::cout << "线速度阻尼 B_vel (N·s/m): [" << B_cartesian.head<3>().transpose() << "]" << std::endl;
        std::cout << "角速度阻尼 B_omega (Nm·s/rad): [" << B_cartesian.tail<3>().transpose() << "]" << std::endl;

        std::cout << "\n提示：" << std::endl;
        std::cout << "- 机械臂末端将保持在当前位置，可以手动推动感受阻抗效果" << std::endl;
        std::cout << "- 推动后松开，机械臂会回到期望位置" << std::endl;
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

            // 转换为 Eigen 向量，并扩展到模型的完整自由度
            Eigen::VectorXd q_eigen = Eigen::VectorXd::Zero(model.nq);
            Eigen::VectorXd v_eigen = Eigen::VectorXd::Zero(model.nv);
            for (size_t i = 0; i < q_current.size() && i < model.nq; ++i) {
                q_eigen[i] = q_current[i];
                v_eigen[i] = v_current[i];
            }
            // 剩余的关节（如手指）保持为0

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

            // 3. 计算笛卡尔空间误差
            Eigen::Vector3d pos_error = x_des - x_current;
            Eigen::Vector3d ori_error = computeOrientationError(R_des, R_current);

            // 限制姿态误差的最大值，防止过大的控制输出
            const double max_ori_error = 0.5;  // 最大约28度
            if (ori_error.norm() > max_ori_error) {
                ori_error = ori_error.normalized() * max_ori_error;
            }

            // 组合位置和姿态误差
            Eigen::Matrix<double, 6, 1> x_error;
            x_error << pos_error, ori_error;

            // 4. 计算笛卡尔空间阻抗力/力矩
            Eigen::Matrix<double, 6, 1> F_cartesian;
            for (int i = 0; i < 6; ++i) {
                F_cartesian(i) = K_cartesian(i) * x_error(i) - B_cartesian(i) * v_ee(i);
            }

            // 5. 转换为关节力矩（使用雅可比转置）
            Eigen::VectorXd tau_impedance = J.transpose() * F_cartesian;

            // 6. 计算重力补偿力矩（使用完整的关节向量）
            std::vector<double> q_full(model.nq);
            for (int i = 0; i < model.nq; ++i) {
                q_full[i] = q_eigen[i];
            }
            std::vector<double> G = computeGravityCompensation(model, data, q_full);

            // 7. 总力矩 = 阻抗力矩 + 重力补偿力矩（只取前6个关节）
            std::vector<double> tor_total(robot.getMotorCount());
            for (int i = 0; i < robot.getMotorCount(); ++i) {
                tor_total[i] = tau_impedance(i) + G[i];
            }

            // 8. 力矩限幅
            tor_total = clipTorque(tor_total, tau_limit);

            // 9. 发送控制命令（纯力矩控制模式：位置、速度、Kp、Kd 都为零）
            robot.posVelTorqueKpKd(zero_pos, zero_vel, tor_total, zero_kp, zero_kd);

            // 10. 定期打印状态信息
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - last_print_time).count();

            if (elapsed >= print_interval) {
                std::cout << std::fixed << std::setprecision(4);

                std::cout << "\n当前末端位置 (m): [" << x_current.transpose() << "]" << std::endl;
                std::cout << "位置误差 (m):     [" << pos_error.transpose() << "]" << std::endl;

                Eigen::Vector3d euler_current = rotationMatrixToEulerZYX(R_current);
                std::cout << "当前末端姿态 (rad): [" << euler_current.transpose() << "]" << std::endl;
                std::cout << "姿态误差 (rad):     [" << ori_error.transpose() << "]" << std::endl;

                std::cout << std::setprecision(3);
                std::cout << "笛卡尔力/力矩: [";
                for (int i = 0; i < 6; ++i) {
                    std::cout << std::setw(7) << F_cartesian(i)
                              << (i < 5 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                std::cout << "关节力矩:      [";
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
