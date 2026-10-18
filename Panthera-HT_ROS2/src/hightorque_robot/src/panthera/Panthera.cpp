#include "panthera/Panthera.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace panthera
{

// ==================== 构造函数和析构函数 ====================

Panthera::Panthera() : hightorque_robot::robot(), motor_count_(0), gripper_id_(0)
{
    // 使用安装空间中的默认配置文件路径，避免从 ros2 run 启动时依赖工作目录。
    std::string default_config =
        ament_index_cpp::get_package_share_directory("hightorque_robot") +
        "/robot_param/Follower.yaml";
    initialize(default_config);
}

Panthera::Panthera(const std::string& config_path)
    : hightorque_robot::robot(config_path), motor_count_(0), gripper_id_(0)
{
    initialize(config_path);
}

Panthera::~Panthera()
{
}

// ==================== 初始化方法 ====================

void Panthera::initialize(const std::string& config_path)
{
    // 加载配置文件
    loadConfig(config_path);

    // 保存配置文件目录
    size_t last_slash = config_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        config_dir_ = config_path.substr(0, last_slash);
    } else {
        config_dir_ = ".";
    }

    // 注意：父类构造函数已经在构造函数初始化列表中调用，会自动调用 init_robot

    // 获取电机数量（不包含夹爪）
    motor_count_ = Motors.size() - 1;
    gripper_id_ = Motors.size();

    std::cout << "初始化机械臂..." << std::endl;
    std::cout << "发现 " << motor_count_ << " 个电机" << std::endl;

    if (motor_count_ == 0) {
        std::cerr << "未发现电机。请检查您的配置和连接。" << std::endl;
        return;
    }

    // 打印电机信息
    for (size_t i = 0; i < Motors.size(); ++i) {
        std::cout << "Motor " << i << ": "
                  << "ID=" << Motors[i]->get_motor_id() << ", "
                  << "Type=" << static_cast<int>(Motors[i]->get_motor_enum_type()) << ", "
                  << "Name=" << Motors[i]->get_motor_name() << std::endl;
    }
}

void Panthera::loadConfig(const std::string& config_path)
{
    try {
        config_ = YAML::LoadFile(config_path);
        std::cout << "配置文件加载成功: " << config_path << std::endl;

        // 读取关节限位
        if (config_["robot"] && config_["robot"]["joint_limits"]) {
            auto limits = config_["robot"]["joint_limits"];
            joint_limits_lower_ = limits["lower"].as<std::vector<double>>();
            joint_limits_upper_ = limits["upper"].as<std::vector<double>>();

            std::cout << "关节限位加载成功: lower=[";
            for (size_t i = 0; i < joint_limits_lower_.size(); ++i) {
                std::cout << joint_limits_lower_[i];
                if (i < joint_limits_lower_.size() - 1) std::cout << ", ";
            }
            std::cout << "], upper=[";
            for (size_t i = 0; i < joint_limits_upper_.size(); ++i) {
                std::cout << joint_limits_upper_[i];
                if (i < joint_limits_upper_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        } else {
            std::cerr << "警告: 配置文件中未找到joint_limits" << std::endl;
        }

        // 读取关节名称
        if (config_["kinematics"] && config_["kinematics"]["joint_names"]) {
            joint_names_ = config_["kinematics"]["joint_names"].as<std::vector<std::string>>();
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "配置文件加载失败: " << e.what() << std::endl;
        throw;
    }
}

bool Panthera::checkJointLimits(const std::vector<double>& pos)
{
    if (joint_limits_lower_.empty() || joint_limits_upper_.empty()) {
        return true; // 如果没有配置限位，直接通过
    }

    if (pos.size() > joint_limits_lower_.size()) {
        std::cerr << "错误: 位置数组大小超出关节限位配置数" << std::endl;
        return false;
    }

    bool all_in_range = true;
    std::vector<int> out_indices;

    for (size_t i = 0; i < pos.size(); ++i) {
        if (pos[i] < joint_limits_lower_[i] || pos[i] > joint_limits_upper_[i]) {
            all_in_range = false;
            out_indices.push_back(i);
        }
    }

    if (!all_in_range) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "警告：检测到目标位置超出关节限位范围！" << std::endl;
        std::cout << "目标位置: [";
        for (size_t i = 0; i < pos.size(); ++i) {
            std::cout << pos[i];
            if (i < pos.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "限位下限: [";
        for (size_t i = 0; i < joint_limits_lower_.size(); ++i) {
            std::cout << joint_limits_lower_[i];
            if (i < joint_limits_lower_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "限位上限: [";
        for (size_t i = 0; i < joint_limits_upper_.size(); ++i) {
            std::cout << joint_limits_upper_[i];
            if (i < joint_limits_upper_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        for (int idx : out_indices) {
            std::cout << "  关节" << (idx + 1) << ": " << pos[idx]
                      << " 不在 [" << joint_limits_lower_[idx]
                      << ", " << joint_limits_upper_[idx] << "] 范围内" << std::endl;
        }
        std::cout << "控制指令已被拒绝，保护机械臂安全" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
        return false;
    }

    return true;
}

// ==================== 状态获取接口 ====================

std::vector<double> Panthera::getCurrentPos()
{
    std::vector<double> positions(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        positions[i] = state->position;
    }
    return positions;
}

std::vector<double> Panthera::getCurrentVel()
{
    std::vector<double> velocities(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        velocities[i] = state->velocity;
    }
    return velocities;
}

std::vector<double> Panthera::getCurrentTorque()
{
    std::vector<double> torques(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        torques[i] = state->torque;
    }
    return torques;
}

double Panthera::getCurrentPosGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->position;
}

double Panthera::getCurrentVelGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->velocity;
}

double Panthera::getCurrentTorqueGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->torque;
}

// ==================== 控制接口 ====================

bool Panthera::posVelMaxTorque(const std::vector<double>& pos,
                                const std::vector<double>& vel,
                                const std::vector<double>& max_torque,
                                bool is_wait,
                                double tolerance,
                                double timeout)
{
    // 检查参数长度
    if (pos.size() != motor_count_ || vel.size() != motor_count_ ||
        max_torque.size() != motor_count_) {
        std::cerr << "错误: 关节参数长度必须为 " << motor_count_ << std::endl;
        return false;
    }

    // 检查关节限位
    if (!checkJointLimits(pos)) {
        return false;
    }

    // 控制关节（除了夹爪电机）
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->pos_vel_MAXtqe(pos[i], vel[i], max_torque[i]);
    }
    motor_send_cmd();

    if (is_wait) {
        return waitForPosition(pos, tolerance, timeout);
    }

    return true;
}

bool Panthera::posVelTorqueKpKd(const std::vector<double>& pos,
                                 const std::vector<double>& vel,
                                 const std::vector<double>& torque,
                                 const std::vector<double>& kp,
                                 const std::vector<double>& kd)
{
    // 检查参数长度
    if (pos.size() != motor_count_ || vel.size() != motor_count_ ||
        torque.size() != motor_count_ || kp.size() != motor_count_ ||
        kd.size() != motor_count_) {
        std::cerr << "错误: 关节参数长度必须为 " << motor_count_ << std::endl;
        return false;
    }

    // 检查关节限位
    if (!checkJointLimits(pos)) {
        return false;
    }

    // 控制关节（除了夹爪电机）
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->pos_vel_tqe_kp_kd(pos[i], vel[i], torque[i], kp[i], kd[i]);
    }
    motor_send_cmd();

    return true;
}

// ==================== 夹爪控制接口 ====================

bool Panthera::gripperControl(double pos, double vel, double max_torque)
{
    Motors[gripper_id_ - 1]->pos_vel_MAXtqe(pos, vel, max_torque);
    motor_send_cmd();
    return true;
}

bool Panthera::gripperControlMIT(double pos, double vel, double torque,
                                  double kp, double kd)
{
    Motors[gripper_id_ - 1]->pos_vel_tqe_kp_kd(pos, vel, torque, kp, kd);
    motor_send_cmd();
    return true;
}

void Panthera::gripperOpen(double vel, double max_torque)
{
    gripperControl(0.8, vel, max_torque);
}

void Panthera::gripperClose(double pos, double vel, double max_torque)
{
    gripperControl(pos, vel, max_torque);
}

// ==================== 位置检测接口 ====================

bool Panthera::checkPositionReached(const std::vector<double>& target_positions,
                                     double tolerance,
                                     std::vector<double>& position_errors)
{
    bool all_reached = true;
    position_errors.clear();
    position_errors.resize(motor_count_);

    send_get_motor_state_cmd();
    motor_send_cmd();

    // 检查前N个关节（不包含夹爪）
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        double error = std::abs(state->position - target_positions[i]);
        position_errors[i] = error;
        if (error > tolerance) {
            all_reached = false;
        }
    }

    return all_reached;
}

bool Panthera::waitForPosition(const std::vector<double>& target_positions,
                                double tolerance,
                                double timeout)
{
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count() / 1000.0;

        if (elapsed >= timeout) {
            return false;
        }

        std::vector<double> errors;
        if (checkPositionReached(target_positions, tolerance, errors)) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ==================== 工具方法 ====================

void Panthera::getJointLimits(std::vector<double>& lower, std::vector<double>& upper) const
{
    lower = joint_limits_lower_;
    upper = joint_limits_upper_;
}

} // namespace panthera
