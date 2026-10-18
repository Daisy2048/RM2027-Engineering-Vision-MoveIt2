/**
 * @file arm_control_node.cpp
 * @brief Panthera HT arm control node - direct SDK driver
 *
 * Provides ROS2 topics and services to control the arm via the Panthera SDK.
 * Includes KDL-based IK/FK for Cartesian space control.
 * No MoveIt dependency - directly drives motors.
 */

#include "rclcpp/rclcpp.hpp"
#include "panthera/Panthera.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "panthera_interfaces/msg/arm_pose.hpp"
#include "panthera_interfaces/msg/arm_status.hpp"
#include "panthera_interfaces/msg/end_pose_euler.hpp"
#include "panthera_interfaces/msg/pos_cmd.hpp"
#include "panthera_interfaces/srv/move_to_joint.hpp"
#include "panthera_interfaces/srv/move_to_pose.hpp"
#include "panthera_interfaces/srv/gripper_control.hpp"
#include "panthera_interfaces/srv/enable.hpp"
#include "panthera_interfaces/srv/go_zero.hpp"
#include "panthera_interfaces/srv/gripper_srv.hpp"

// KDL headers
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>

#include <atomic>
#include <mutex>
#include <algorithm>
#include <vector>
#include <cmath>

using ArmJointMsg = example_interfaces::msg::Float64MultiArray;
using ArmPoseMsg = panthera_interfaces::msg::ArmPose;
using BoolMsg = example_interfaces::msg::Bool;
using JointStateMsg = sensor_msgs::msg::JointState;
using ArmStatusMsg = panthera_interfaces::msg::ArmStatus;
using PosCmdMsg = panthera_interfaces::msg::PosCmd;
using EndPoseEulerMsg = panthera_interfaces::msg::EndPoseEuler;
using MoveToJointSrv = panthera_interfaces::srv::MoveToJoint;
using MoveToPoseSrv = panthera_interfaces::srv::MoveToPose;
using GripperControlSrv = panthera_interfaces::srv::GripperControl;
using EnableSrv = panthera_interfaces::srv::Enable;
using GoZeroSrv = panthera_interfaces::srv::GoZero;
using GripperSrvType = panthera_interfaces::srv::GripperSrv;
using TriggerSrv = std_srvs::srv::Trigger;

class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode()
        : Node("arm_control_node")
    {
        // Parameters
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<double>("status_publish_rate", 20.0);
        this->declare_parameter<double>("max_velocity", 0.5);
        this->declare_parameter<std::vector<double>>("max_torque",
            {21.0, 36.0, 36.0, 21.0, 10.0, 10.0});
        this->declare_parameter<std::string>("urdf_file", "");
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("tip_link", "link6");

        std::string config_file = this->get_parameter("config_file").as_string();
        if (config_file.empty()) {
            config_file = ament_index_cpp::get_package_share_directory("hightorque_robot")
                          + "/robot_param/Follower.yaml";
        }

        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_torque_ = this->get_parameter("max_torque").as_double_array();

        // Initialize robot SDK
        RCLCPP_INFO(this->get_logger(), "Initializing Panthera SDK with config: %s", config_file.c_str());
        robot_ = std::make_unique<panthera::Panthera>(config_file);
        motor_count_ = robot_->getMotorCount();
        RCLCPP_INFO(this->get_logger(), "Robot initialized, motor count: %d", motor_count_);

        // Joint names
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // Initialize KDL kinematics
        initKDL();

        // ==================== Publishers ====================

        joint_states_pub_ = this->create_publisher<JointStateMsg>("joint_states_single", 10);
        arm_status_pub_ = this->create_publisher<ArmStatusMsg>("arm_status", 10);
        end_pose_pub_ = this->create_publisher<EndPoseEulerMsg>("end_pose_euler", 10);

        // ==================== Subscribers ====================

        arm_joint_sub_ = this->create_subscription<ArmJointMsg>(
            "arm_joint_cmd", 10,
            std::bind(&ArmControlNode::armJointCallback, this, std::placeholders::_1));

        gripper_sub_ = this->create_subscription<BoolMsg>(
            "gripper_cmd", 10,
            std::bind(&ArmControlNode::gripperCallback, this, std::placeholders::_1));

        enable_flag_sub_ = this->create_subscription<BoolMsg>(
            "enable_flag", 10,
            std::bind(&ArmControlNode::enableFlagCallback, this, std::placeholders::_1));

        pos_cmd_sub_ = this->create_subscription<PosCmdMsg>(
            "pos_cmd", 10,
            std::bind(&ArmControlNode::posCmdCallback, this, std::placeholders::_1));

        // ==================== Services ====================

        move_to_joint_srv_ = this->create_service<MoveToJointSrv>(
            "move_to_joint",
            std::bind(&ArmControlNode::moveToJointService, this,
                       std::placeholders::_1, std::placeholders::_2));

        move_to_pose_srv_ = this->create_service<MoveToPoseSrv>(
            "move_to_pose",
            std::bind(&ArmControlNode::moveToPoseService, this,
                       std::placeholders::_1, std::placeholders::_2));

        gripper_control_srv_ = this->create_service<GripperControlSrv>(
            "gripper_control",
            std::bind(&ArmControlNode::gripperControlService, this,
                       std::placeholders::_1, std::placeholders::_2));

        enable_srv_ = this->create_service<EnableSrv>(
            "enable_srv",
            std::bind(&ArmControlNode::enableService, this,
                       std::placeholders::_1, std::placeholders::_2));

        go_zero_srv_ = this->create_service<GoZeroSrv>(
            "go_zero_srv",
            std::bind(&ArmControlNode::goZeroService, this,
                       std::placeholders::_1, std::placeholders::_2));

        stop_srv_ = this->create_service<TriggerSrv>(
            "stop_srv",
            std::bind(&ArmControlNode::stopService, this,
                       std::placeholders::_1, std::placeholders::_2));

        reset_srv_ = this->create_service<TriggerSrv>(
            "reset_srv",
            std::bind(&ArmControlNode::resetService, this,
                       std::placeholders::_1, std::placeholders::_2));

        gripper_detail_srv_ = this->create_service<GripperSrvType>(
            "gripper_srv",
            std::bind(&ArmControlNode::gripperDetailService, this,
                       std::placeholders::_1, std::placeholders::_2));

        // ==================== Timer ====================

        double publish_rate = this->get_parameter("status_publish_rate").as_double();
        int period_ms = static_cast<int>(1000.0 / publish_rate);
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ArmControlNode::publishStatus, this));

        RCLCPP_INFO(this->get_logger(), "=== Panthera Arm Control Node Started ===");
        RCLCPP_INFO(this->get_logger(), "Publishers: joint_states_single, arm_status, end_pose_euler");
        RCLCPP_INFO(this->get_logger(), "Subscribers: arm_joint_cmd, gripper_cmd, enable_flag, pos_cmd");
        RCLCPP_INFO(this->get_logger(), "Services: move_to_joint, move_to_pose, gripper_control, enable_srv, go_zero_srv, stop_srv, reset_srv, gripper_srv");
    }

private:
    // ==================== KDL Initialization ====================

    void initKDL()
    {
        std::string urdf_file = this->get_parameter("urdf_file").as_string();
        if (urdf_file.empty()) {
            urdf_file = ament_index_cpp::get_package_share_directory("panthera_ht_ros_description")
                        + "/urdf/panthera_ht_ros_description.urdf";
        }

        std::string base_link = this->get_parameter("base_link").as_string();
        std::string tip_link = this->get_parameter("tip_link").as_string();

        RCLCPP_INFO(this->get_logger(), "Loading URDF for KDL: %s", urdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Kinematic chain: %s -> %s", base_link.c_str(), tip_link.c_str());

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromFile(urdf_file, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file: %s", urdf_file.c_str());
            kdl_ready_ = false;
            return;
        }

        if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from %s to %s",
                         base_link.c_str(), tip_link.c_str());
            kdl_ready_ = false;
            return;
        }

        unsigned int nj = kdl_chain_.getNrOfJoints();
        RCLCPP_INFO(this->get_logger(), "KDL chain has %u joints", nj);

        if (static_cast<int>(nj) != motor_count_) {
            RCLCPP_WARN(this->get_logger(),
                        "KDL joint count (%u) != motor count (%d), IK may not work correctly",
                        nj, motor_count_);
        }

        // Store joint limits for post-IK validation
        robot_->getJointLimits(joint_limits_lower_, joint_limits_upper_);

        // Create solvers
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

        // LMA solver: weight translation (1,1,1) more than rotation (0.01,0.01,0.01)
        Eigen::Matrix<double, 6, 1> L;
        L << 1.0, 1.0, 1.0, 0.01, 0.01, 0.01;
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(
            kdl_chain_, L, 1e-5, 500);

        kdl_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "KDL IK/FK solvers initialized successfully");
    }

    // ==================== FK / IK ====================

    bool computeFK(const std::vector<double> & joint_pos,
                   double & x, double & y, double & z,
                   double & roll, double & pitch, double & yaw)
    {
        if (!kdl_ready_) return false;

        unsigned int nj = kdl_chain_.getNrOfJoints();
        KDL::JntArray q(nj);
        for (unsigned int i = 0; i < nj && i < joint_pos.size(); ++i) {
            q(i) = joint_pos[i];
        }

        KDL::Frame frame;
        int ret = fk_solver_->JntToCart(q, frame);
        if (ret < 0) return false;

        x = frame.p.x();
        y = frame.p.y();
        z = frame.p.z();
        frame.M.GetRPY(roll, pitch, yaw);
        return true;
    }

    bool solveIK(double x, double y, double z,
                 double roll, double pitch, double yaw,
                 const std::vector<double> & seed_joints,
                 std::vector<double> & result_joints,
                 std::string & error_msg)
    {
        if (!kdl_ready_) {
            error_msg = "KDL not initialized";
            return false;
        }

        unsigned int nj = kdl_chain_.getNrOfJoints();

        // Build target frame
        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(x, y, z);
        target_frame.M = KDL::Rotation::RPY(roll, pitch, yaw);

        // Seed from current joint positions
        KDL::JntArray q_seed(nj);
        for (unsigned int i = 0; i < nj && i < seed_joints.size(); ++i) {
            q_seed(i) = seed_joints[i];
        }

        KDL::JntArray q_result(nj);
        int ret = ik_solver_->CartToJnt(q_seed, target_frame, q_result);
        if (ret < 0) {
            error_msg = "IK solver failed (code " + std::to_string(ret) +
                        "). Target may be unreachable.";
            return false;
        }

        // Check joint limits
        result_joints.resize(nj);
        for (unsigned int i = 0; i < nj; ++i) {
            result_joints[i] = q_result(i);
            if (i < joint_limits_lower_.size() && i < joint_limits_upper_.size()) {
                if (result_joints[i] < joint_limits_lower_[i] ||
                    result_joints[i] > joint_limits_upper_[i]) {
                    error_msg = "IK result joint" + std::to_string(i + 1) +
                                "=" + std::to_string(result_joints[i]) +
                                " out of limits [" + std::to_string(joint_limits_lower_[i]) +
                                ", " + std::to_string(joint_limits_upper_[i]) + "]";
                    return false;
                }
            }
        }
        return true;
    }

    // ==================== Cartesian Path (linear interpolation) ====================

    bool executeCartesianPath(double target_x, double target_y, double target_z,
                              double target_roll, double target_pitch, double target_yaw,
                              double vel, std::string & error_msg)
    {
        // Get current end-effector pose via FK
        std::vector<double> current_joints;
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            current_joints = robot_->getCurrentPos();
        }

        double cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw;
        if (!computeFK(current_joints, cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw)) {
            error_msg = "FK failed for current joint positions";
            return false;
        }

        // Compute distance for step count
        double dx = target_x - cur_x;
        double dy = target_y - cur_y;
        double dz = target_z - cur_z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 1cm per step
        int num_steps = std::max(2, static_cast<int>(dist / 0.01));

        for (int step = 1; step <= num_steps; ++step) {
            double t = static_cast<double>(step) / num_steps;

            double ix = cur_x + t * dx;
            double iy = cur_y + t * dy;
            double iz = cur_z + t * dz;
            double ir = cur_roll + t * (target_roll - cur_roll);
            double ip = cur_pitch + t * (target_pitch - cur_pitch);
            double iy_aw = cur_yaw + t * (target_yaw - cur_yaw);

            // Use previous result as seed for next IK
            std::vector<double> ik_result;
            if (!solveIK(ix, iy, iz, ir, ip, iy_aw, current_joints, ik_result, error_msg)) {
                error_msg = "Cartesian path IK failed at step " + std::to_string(step) +
                            "/" + std::to_string(num_steps) + ": " + error_msg;
                return false;
            }

            if (!executeJointTarget(ik_result, vel, error_msg)) {
                return false;
            }

            current_joints = ik_result;
        }

        return true;
    }

    // ==================== Core Motion ====================

    bool executeJointTarget(const std::vector<double> & joint_values,
                            double vel, std::string & error_msg)
    {
        if (!arm_enabled_) {
            error_msg = "Arm is disabled. Enable it first.";
            return false;
        }

        if (static_cast<int>(joint_values.size()) != motor_count_) {
            error_msg = "Expected " + std::to_string(motor_count_) +
                        " joint values, got " + std::to_string(joint_values.size());
            return false;
        }

        std::vector<double> velocities(motor_count_, vel);

        std::lock_guard<std::mutex> lock(robot_mutex_);
        is_moving_ = true;
        bool success = robot_->posVelMaxTorque(
            joint_values, velocities, max_torque_, true, 0.05, 15.0);
        is_moving_ = false;

        if (!success) {
            error_msg = "Joint target execution failed or timed out";
            setError(error_msg);
            return false;
        }

        clearError();
        return true;
    }

    // ==================== Error Tracking ====================

    void setError(const std::string & msg)
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_message_ = msg;
    }

    void clearError()
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_message_.clear();
    }

    // ==================== Topic Callbacks ====================

    void armJointCallback(const ArmJointMsg::SharedPtr msg)
    {
        if (static_cast<int>(msg->data.size()) != motor_count_) {
            RCLCPP_ERROR(this->get_logger(), "arm_joint_cmd: expected %d values, got %zu",
                         motor_count_, msg->data.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "arm_joint_cmd: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    msg->data[0], msg->data[1], msg->data[2],
                    msg->data[3], msg->data[4], msg->data[5]);

        std::string error_msg;
        std::vector<double> joints(msg->data.begin(), msg->data.end());
        if (executeJointTarget(joints, max_velocity_, error_msg)) {
            RCLCPP_INFO(this->get_logger(), "Joint target reached");
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
        }
    }

    void gripperCallback(const BoolMsg::SharedPtr msg)
    {
        if (!arm_enabled_) {
            RCLCPP_WARN(this->get_logger(), "Arm disabled, ignoring gripper command");
            return;
        }

        std::lock_guard<std::mutex> lock(robot_mutex_);
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Gripper: open");
            robot_->gripperOpen();
        } else {
            RCLCPP_INFO(this->get_logger(), "Gripper: close");
            robot_->gripperClose();
        }
    }

    void enableFlagCallback(const BoolMsg::SharedPtr msg)
    {
        arm_enabled_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Arm %s via enable_flag",
                    arm_enabled_.load() ? "ENABLED" : "DISABLED");
        if (!arm_enabled_) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->set_stop();
            robot_->motor_send_cmd();
        }
    }

    void posCmdCallback(const PosCmdMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "pos_cmd: xyz=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f] gripper=%.2f mode=%d",
                    msg->x, msg->y, msg->z,
                    msg->roll, msg->pitch, msg->yaw,
                    msg->gripper, msg->mode1);

        std::string error_msg;

        if (msg->mode1 == 1) {
            // Cartesian path mode
            if (!executeCartesianPath(msg->x, msg->y, msg->z,
                                      msg->roll, msg->pitch, msg->yaw,
                                      max_velocity_, error_msg)) {
                RCLCPP_ERROR(this->get_logger(), "pos_cmd cartesian: %s", error_msg.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "pos_cmd cartesian: target reached");
            }
        } else {
            // Joint-space mode: IK then move
            std::vector<double> seed_joints;
            {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                seed_joints = robot_->getCurrentPos();
            }

            std::vector<double> ik_result;
            if (!solveIK(msg->x, msg->y, msg->z,
                         msg->roll, msg->pitch, msg->yaw,
                         seed_joints, ik_result, error_msg)) {
                RCLCPP_ERROR(this->get_logger(), "pos_cmd IK: %s", error_msg.c_str());
            } else if (!executeJointTarget(ik_result, max_velocity_, error_msg)) {
                RCLCPP_ERROR(this->get_logger(), "pos_cmd move: %s", error_msg.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "pos_cmd: target reached");
            }
        }

        // Handle gripper
        if (msg->gripper >= 0.0) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            if (msg->gripper > 0.5) {
                robot_->gripperOpen();
            } else {
                robot_->gripperClose();
            }
        }
    }

    // ==================== Service Callbacks ====================

    void moveToJointService(const MoveToJointSrv::Request::SharedPtr request,
                            MoveToJointSrv::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(),
                    "move_to_joint: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    request->joint_angles[0], request->joint_angles[1],
                    request->joint_angles[2], request->joint_angles[3],
                    request->joint_angles[4], request->joint_angles[5]);

        std::vector<double> joints(request->joint_angles.begin(),
                                   request->joint_angles.end());

        double vel = request->velocity_scaling * max_velocity_;
        std::string error_msg;
        response->success = executeJointTarget(joints, vel, error_msg);
        response->message = response->success ? "Joint target reached" : error_msg;
    }

    void moveToPoseService(const MoveToPoseSrv::Request::SharedPtr request,
                           MoveToPoseSrv::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(),
                    "move_to_pose: xyz=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f] cartesian=%d",
                    request->x, request->y, request->z,
                    request->roll, request->pitch, request->yaw,
                    request->cartesian_path);

        double vel = request->velocity_scaling * max_velocity_;
        std::string error_msg;

        if (request->cartesian_path) {
            response->success = executeCartesianPath(
                request->x, request->y, request->z,
                request->roll, request->pitch, request->yaw,
                vel, error_msg);
        } else {
            std::vector<double> seed_joints;
            {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                seed_joints = robot_->getCurrentPos();
            }

            std::vector<double> ik_result;
            if (!solveIK(request->x, request->y, request->z,
                         request->roll, request->pitch, request->yaw,
                         seed_joints, ik_result, error_msg)) {
                response->success = false;
                response->message = error_msg;
                return;
            }

            RCLCPP_INFO(this->get_logger(),
                        "IK result: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        ik_result[0], ik_result[1], ik_result[2],
                        ik_result[3], ik_result[4], ik_result[5]);

            response->success = executeJointTarget(ik_result, vel, error_msg);
        }

        response->message = response->success ? "Pose target reached" : error_msg;
    }

    void gripperControlService(const GripperControlSrv::Request::SharedPtr request,
                               GripperControlSrv::Response::SharedPtr response)
    {
        if (!arm_enabled_) {
            response->success = false;
            response->message = "Arm is disabled";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "gripper_control: %s", request->action.c_str());

        std::lock_guard<std::mutex> lock(robot_mutex_);
        if (request->action == "open") {
            robot_->gripperOpen();
        } else if (request->action == "close") {
            robot_->gripperClose();
        } else if (request->action == "half_open") {
            robot_->gripperClose(0.5);  // half position
        } else {
            response->success = false;
            response->message = "Unknown action: " + request->action + ". Use open/close/half_open";
            return;
        }

        response->success = true;
        response->message = "Gripper " + request->action + " done";
    }

    void enableService(const EnableSrv::Request::SharedPtr request,
                       EnableSrv::Response::SharedPtr response)
    {
        arm_enabled_ = request->enable_request;
        if (!arm_enabled_) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->set_stop();
            robot_->motor_send_cmd();
        }
        response->enable_response = arm_enabled_.load();
        RCLCPP_INFO(this->get_logger(), "enable_srv: arm %s",
                    arm_enabled_.load() ? "ENABLED" : "DISABLED");
    }

    void goZeroService(const GoZeroSrv::Request::SharedPtr /*request*/,
                       GoZeroSrv::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "go_zero_srv: moving to home position");

        if (!arm_enabled_) {
            response->code = -1;
            response->status = false;
            return;
        }

        std::vector<double> zero_pos(motor_count_, 0.0);
        std::string error_msg;
        if (executeJointTarget(zero_pos, max_velocity_, error_msg)) {
            response->code = 0;
            response->status = true;
            RCLCPP_INFO(this->get_logger(), "go_zero_srv: completed");
        } else {
            response->code = -2;
            response->status = false;
            RCLCPP_ERROR(this->get_logger(), "go_zero_srv: %s", error_msg.c_str());
        }
    }

    void stopService(const TriggerSrv::Request::SharedPtr /*request*/,
                     TriggerSrv::Response::SharedPtr response)
    {
        RCLCPP_WARN(this->get_logger(), "stop_srv: emergency stop");
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->set_stop();
            robot_->motor_send_cmd();
        }
        is_moving_ = false;
        response->success = true;
        response->message = "Motion stopped";
    }

    void resetService(const TriggerSrv::Request::SharedPtr /*request*/,
                      TriggerSrv::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "reset_srv: resetting arm");
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->set_reset();
            robot_->motor_send_cmd();
        }
        clearError();
        arm_enabled_ = true;
        is_moving_ = false;
        response->success = true;
        response->message = "Arm reset and re-enabled";
    }

    void gripperDetailService(const GripperSrvType::Request::SharedPtr request,
                              GripperSrvType::Response::SharedPtr response)
    {
        if (!arm_enabled_) {
            response->code = -1;
            response->status = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "gripper_srv: angle=%.3f effort=%.3f",
                    request->gripper_angle, request->gripper_effort);

        if (request->set_zero) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->set_reset_zero({robot_->getMotorCount() + 1});
            robot_->motor_send_cmd();
            response->code = 0;
            response->status = true;
            return;
        }

        double effort = std::clamp(request->gripper_effort, 0.1, 2.0);

        std::lock_guard<std::mutex> lock(robot_mutex_);
        bool success = robot_->gripperControl(request->gripper_angle, 0.5, effort);
        robot_->motor_send_cmd();

        response->status = success;
        response->code = success ? 0 : -2;
    }

    // ==================== Periodic Status Publisher ====================

    void publishStatus()
    {
        auto now = this->get_clock()->now();

        // Query motor states
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->send_get_motor_state_cmd();
            robot_->motor_send_cmd();
        }

        // 1. Publish joint_states_single
        JointStateMsg joint_msg;
        joint_msg.header.stamp = now;
        joint_msg.header.frame_id = "base_link";
        joint_msg.name = joint_names_;

        std::vector<double> positions, velocities, torques;
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            positions = robot_->getCurrentPos();
            velocities = robot_->getCurrentVel();
            torques = robot_->getCurrentTorque();
        }

        joint_msg.position.assign(positions.begin(), positions.end());
        joint_msg.velocity.assign(velocities.begin(), velocities.end());
        joint_msg.effort.assign(torques.begin(), torques.end());

        // Add gripper
        joint_msg.name.push_back("L_finger_joint");
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            joint_msg.position.push_back(robot_->getCurrentPosGripper());
            joint_msg.velocity.push_back(robot_->getCurrentVelGripper());
            joint_msg.effort.push_back(robot_->getCurrentTorqueGripper());
        }

        joint_states_pub_->publish(joint_msg);

        // 2. Publish arm_status
        ArmStatusMsg status_msg;
        status_msg.header.stamp = now;
        status_msg.header.frame_id = "base_link";
        status_msg.arm_enabled = arm_enabled_.load();

        if (is_moving_) {
            status_msg.motion_status = 1;  // moving
        } else {
            std::lock_guard<std::mutex> lock(error_mutex_);
            status_msg.motion_status = last_error_message_.empty() ? 0 : 2;
            status_msg.error_message = last_error_message_;
        }

        // Motor fault info from SDK
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            for (int i = 0; i < 6 && i < static_cast<int>(robot_->Motors.size()); ++i) {
                auto * state = robot_->Motors[i]->get_current_motor_state();
                status_msg.motor_modes[i] = state->mode;
                status_msg.motor_faults[i] = state->fault;
                status_msg.joint_at_limit[i] = (robot_->Motors[i]->pos_limit_flag != 0);
            }

            status_msg.gripper_position = robot_->getCurrentPosGripper();
            int gripper_idx = robot_->getMotorCount();
            if (gripper_idx < static_cast<int>(robot_->Motors.size())) {
                auto * gs = robot_->Motors[gripper_idx]->get_current_motor_state();
                status_msg.gripper_fault = gs->fault;
            }
        }

        arm_status_pub_->publish(status_msg);

        // 3. Publish end_pose_euler (FK)
        if (kdl_ready_) {
            EndPoseEulerMsg pose_msg;
            pose_msg.header.stamp = now;
            pose_msg.header.frame_id = "base_link";

            if (computeFK(positions, pose_msg.x, pose_msg.y, pose_msg.z,
                          pose_msg.roll, pose_msg.pitch, pose_msg.yaw)) {
                end_pose_pub_->publish(pose_msg);
            }
        }
    }

    // ==================== Members ====================

    std::unique_ptr<panthera::Panthera> robot_;
    std::mutex robot_mutex_;
    int motor_count_;
    double max_velocity_;
    std::vector<double> max_torque_;
    std::vector<std::string> joint_names_;

    std::atomic<bool> arm_enabled_{true};
    std::atomic<bool> is_moving_{false};
    std::string last_error_message_;
    std::mutex error_mutex_;

    // KDL
    bool kdl_ready_{false};
    KDL::Chain kdl_chain_;
    std::vector<double> joint_limits_lower_;
    std::vector<double> joint_limits_upper_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

    // Publishers
    rclcpp::Publisher<JointStateMsg>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<ArmStatusMsg>::SharedPtr arm_status_pub_;
    rclcpp::Publisher<EndPoseEulerMsg>::SharedPtr end_pose_pub_;

    // Subscribers
    rclcpp::Subscription<ArmJointMsg>::SharedPtr arm_joint_sub_;
    rclcpp::Subscription<BoolMsg>::SharedPtr gripper_sub_;
    rclcpp::Subscription<BoolMsg>::SharedPtr enable_flag_sub_;
    rclcpp::Subscription<PosCmdMsg>::SharedPtr pos_cmd_sub_;

    // Services
    rclcpp::Service<MoveToJointSrv>::SharedPtr move_to_joint_srv_;
    rclcpp::Service<MoveToPoseSrv>::SharedPtr move_to_pose_srv_;
    rclcpp::Service<GripperControlSrv>::SharedPtr gripper_control_srv_;
    rclcpp::Service<EnableSrv>::SharedPtr enable_srv_;
    rclcpp::Service<GoZeroSrv>::SharedPtr go_zero_srv_;
    rclcpp::Service<TriggerSrv>::SharedPtr stop_srv_;
    rclcpp::Service<TriggerSrv>::SharedPtr reset_srv_;
    rclcpp::Service<GripperSrvType>::SharedPtr gripper_detail_srv_;

    // Timer
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
