#ifndef PANTHERA_HPP
#define PANTHERA_HPP

#include "../hardware/robot.hpp"
#include <vector>
#include <string>
#include <array>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace panthera
{

/**
 * @brief Panthera机械臂高级控制类
 *
 * 继承自hightorque_robot::robot基类，提供机械臂的高级控制接口
 * 包含配置加载、关节状态获取、位置控制、夹爪控制等功能
 * （不包含运动学动力学功能）
 */
class Panthera : public hightorque_robot::robot
{
public:
    /**
     * @brief 默认构造函数
     * 使用默认配置文件路径初始化
     */
    Panthera();

    /**
     * @brief 构造函数
     * @param config_path 配置文件路径（YAML格式）
     */
    explicit Panthera(const std::string& config_path);

    /**
     * @brief 析构函数
     */
    ~Panthera();

    // ==================== 状态获取接口 ====================

    /**
     * @brief 获取当前关节位置
     * @return 关节位置数组（弧度）
     */
    std::vector<double> getCurrentPos();

    /**
     * @brief 获取当前关节速度
     * @return 关节速度数组（弧度/秒）
     */
    std::vector<double> getCurrentVel();

    /**
     * @brief 获取当前关节力矩
     * @return 关节力矩数组（Nm）
     */
    std::vector<double> getCurrentTorque();

    /**
     * @brief 获取夹爪当前位置
     * @return 夹爪位置（弧度）
     */
    double getCurrentPosGripper();

    /**
     * @brief 获取夹爪当前速度
     * @return 夹爪速度（弧度/秒）
     */
    double getCurrentVelGripper();

    /**
     * @brief 获取夹爪当前力矩
     * @return 夹爪力矩（Nm）
     */
    double getCurrentTorqueGripper();

    // ==================== 控制接口 ====================

    /**
     * @brief 位置速度最大力矩控制模式
     * @param pos 目标位置数组（弧度）
     * @param vel 目标速度数组（弧度/秒）
     * @param max_torque 最大力矩数组（Nm）
     * @param is_wait 是否等待到达目标位置
     * @param tolerance 位置到达容差（弧度）
     * @param timeout 超时时间（秒）
     * @return 控制是否成功
     */
    bool posVelMaxTorque(const std::vector<double>& pos,
                         const std::vector<double>& vel,
                         const std::vector<double>& max_torque,
                         bool is_wait = false,
                         double tolerance = 0.1,
                         double timeout = 15.0);

    /**
     * @brief 五参数MIT控制模式
     * @param pos 目标位置数组（弧度）
     * @param vel 目标速度数组（弧度/秒）
     * @param torque 前馈力矩数组（Nm）
     * @param kp Kp增益数组
     * @param kd Kd增益数组
     * @return 控制是否成功
     */
    bool posVelTorqueKpKd(const std::vector<double>& pos,
                          const std::vector<double>& vel,
                          const std::vector<double>& torque,
                          const std::vector<double>& kp,
                          const std::vector<double>& kd);

    // ==================== 夹爪控制接口 ====================

    /**
     * @brief 夹爪控制（位置速度最大力矩模式）
     * @param pos 目标位置（弧度）
     * @param vel 目标速度（弧度/秒）
     * @param max_torque 最大力矩（Nm）
     * @return 控制是否成功
     */
    bool gripperControl(double pos, double vel, double max_torque);

    /**
     * @brief 夹爪控制（5参数MIT模式）
     * @param pos 目标位置（弧度）
     * @param vel 目标速度（弧度/秒）
     * @param torque 前馈力矩（Nm）
     * @param kp Kp增益
     * @param kd Kd增益
     * @return 控制是否成功
     */
    bool gripperControlMIT(double pos, double vel, double torque,
                           double kp, double kd);

    /**
     * @brief 打开夹爪
     * @param vel 速度（弧度/秒），默认0.5
     * @param max_torque 最大力矩（Nm），默认0.5
     */
    void gripperOpen(double vel = 0.5, double max_torque = 0.5);

    /**
     * @brief 关闭夹爪
     * @param pos 目标位置（弧度），默认0.0
     * @param vel 速度（弧度/秒），默认0.5
     * @param max_torque 最大力矩（Nm），默认0.5
     */
    void gripperClose(double pos = 0.0, double vel = 0.5, double max_torque = 0.5);

    // ==================== 位置检测接口 ====================

    /**
     * @brief 检查关节位置是否到达目标
     * @param target_positions 目标位置数组
     * @param tolerance 容差（弧度）
     * @param position_errors 输出各关节位置误差
     * @return 是否全部到达
     */
    bool checkPositionReached(const std::vector<double>& target_positions,
                              double tolerance,
                              std::vector<double>& position_errors);

    /**
     * @brief 等待位置到达
     * @param target_positions 目标位置数组
     * @param tolerance 容差（弧度）
     * @param timeout 超时时间（秒）
     * @return 是否成功到达
     */
    bool waitForPosition(const std::vector<double>& target_positions,
                         double tolerance = 0.01,
                         double timeout = 15.0);

    // ==================== 工具方法 ====================

    /**
     * @brief 获取电机数量（不包含夹爪）
     * @return 电机数量
     */
    int getMotorCount() const { return motor_count_; }

    /**
     * @brief 获取关节限位
     * @param lower 输出下限
     * @param upper 输出上限
     */
    void getJointLimits(std::vector<double>& lower, std::vector<double>& upper) const;

private:
    /**
     * @brief 初始化机械臂
     * @param config_path 配置文件路径
     */
    void initialize(const std::string& config_path);

    /**
     * @brief 加载配置文件
     * @param config_path 配置文件路径
     */
    void loadConfig(const std::string& config_path);

    /**
     * @brief 检查位置是否在关节限位内
     * @param pos 目标位置数组
     * @return 是否在限位内
     */
    bool checkJointLimits(const std::vector<double>& pos);

    // ==================== 成员变量 ====================

    YAML::Node config_;                          // 配置文件内容
    std::string config_dir_;                     // 配置文件目录
    int motor_count_;                            // 电机数量（不包含夹爪）
    int gripper_id_;                             // 夹爪电机ID
    std::vector<double> joint_limits_lower_;     // 关节下限
    std::vector<double> joint_limits_upper_;     // 关节上限
    std::vector<std::string> joint_names_;       // 关节名称
};

} // namespace panthera

#endif // PANTHERA_HPP
