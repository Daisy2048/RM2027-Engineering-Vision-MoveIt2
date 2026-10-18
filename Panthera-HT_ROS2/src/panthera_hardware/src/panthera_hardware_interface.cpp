#include "panthera_hardware/panthera_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "panthera/Panthera.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "yaml-cpp/yaml.h"

namespace panthera_hardware
{

bool parse_gain_vector(const char * value, std::vector<double> & gains)
{
  if (value == nullptr) {
    return false;
  }

  try {
    std::stringstream stream(value);
    std::string item;
    gains.clear();
    while (std::getline(stream, item, ',')) {
      gains.push_back(std::stod(item));
    }
  } catch (const std::exception &) {
    gains.clear();
    return false;
  }

  return gains.size() == 6;
}

class GravityModel
{
public:
  bool load(const std::string & config_file)
  {
    try {
      const YAML::Node config = YAML::LoadFile(config_file);
      if (!config["urdf"] || !config["urdf"]["file_path"]) {
        return false;
      }

      const std::size_t separator = config_file.find_last_of("/\\");
      const std::string config_dir =
        separator == std::string::npos ? "." : config_file.substr(0, separator);
      std::string urdf_path =
        config_dir + "/" + config["urdf"]["file_path"].as<std::string>();

      if (urdf_path.size() >= 6 &&
          urdf_path.substr(urdf_path.size() - 6) == ".xacro") {
        const std::string urdf_candidate =
          urdf_path.substr(0, urdf_path.size() - 6) + ".urdf";
        std::ifstream urdf_file(urdf_candidate);
        if (urdf_file.good()) {
          urdf_path = urdf_candidate;
        }
      }

      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);

      if (!config["kinematics"] || !config["kinematics"]["joint_names"]) {
        return false;
      }

      joint_ids_.clear();
      const auto joint_names =
        config["kinematics"]["joint_names"].as<std::vector<std::string>>();
      for (const auto & name : joint_names) {
        if (!model_.existJointName(name)) {
          return false;
        }
        joint_ids_.push_back(model_.getJointId(name));
      }

      return joint_ids_.size() == 6;
    } catch (const std::exception &) {
      return false;
    }
  }

  std::vector<double> gravity(const std::vector<double> & positions) const
  {
    if (joint_ids_.size() != 6 || positions.size() != 6) {
      return {};
    }

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
      const auto joint_id = joint_ids_[i];
      q[model_.joints[joint_id].idx_q()] = positions[i];
    }

    const Eigen::VectorXd generalized_gravity =
      pinocchio::computeGeneralizedGravity(model_, data_, q);

    std::vector<double> result(6, 0.0);
    for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
      result[i] = generalized_gravity[model_.joints[joint_ids_[i]].idx_v()];
    }
    return result;
  }

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  std::vector<pinocchio::JointIndex> joint_ids_;
};

hardware_interface::CallbackReturn PantheraHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get configuration file path
  if (info_.hardware_parameters.find("config_file") != info_.hardware_parameters.end())
  {
    config_file_ = info_.hardware_parameters["config_file"];
    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Config file: %s", config_file_.c_str());
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Parameter 'config_file' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get control mode (default: position_velocity)
  if (info_.hardware_parameters.find("control_mode") != info_.hardware_parameters.end())
  {
    control_mode_ = info_.hardware_parameters["control_mode"];
  }
  else
  {
    control_mode_ = "position_velocity";
  }
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
              "Control mode: %s", control_mode_.c_str());

  // Get gripper rad to meter conversion factor
  if (info_.hardware_parameters.find("gripper_rad_to_m") != info_.hardware_parameters.end())
  {
    gripper_rad_to_m_ = std::stod(info_.hardware_parameters["gripper_rad_to_m"]);
  }
  else
  {
    gripper_rad_to_m_ = 0.01;  // Default: 1 radian = 0.01 meter
  }
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
              "Gripper rad to meter conversion: %.4f", gripper_rad_to_m_);

  // Check if velocity and effort commands are enabled
  use_velocity_commands_ = (
    control_mode_ == "full_control" ||
    control_mode_ == "position_velocity" ||
    control_mode_ == "mit_gravity_compensation");
  use_effort_commands_ = (control_mode_ == "full_control");

  // Initialize state and command storage
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_efforts_.resize(info_.joints.size(), 0.0);

  // Initialize control parameters
  max_torques_.resize(info_.joints.size(), 0.0);
  max_velocities_.resize(info_.joints.size(), 0.5);
  kp_gains_.resize(info_.joints.size(), 0.0);
  kd_gains_.resize(info_.joints.size(), 0.0);

  // Load joint-specific parameters
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Max torque
    if (info_.joints[i].parameters.find("max_torque") != info_.joints[i].parameters.end())
    {
      max_torques_[i] = std::stod(info_.joints[i].parameters.at("max_torque"));
    }
    else
    {
      max_torques_[i] = 10.0;  // Default value
    }

    // Max velocity
    if (info_.joints[i].parameters.find("max_velocity") != info_.joints[i].parameters.end())
    {
      max_velocities_[i] = std::stod(info_.joints[i].parameters.at("max_velocity"));
    }
    else
    {
      max_velocities_[i] = 0.5;  // Default value
    }

    // PD gains
    if (info_.joints[i].parameters.find("kp") != info_.joints[i].parameters.end())
    {
      kp_gains_[i] = std::stod(info_.joints[i].parameters.at("kp"));
    }
    else
    {
      kp_gains_[i] = 4.0;  // Default value
    }

    if (info_.joints[i].parameters.find("kd") != info_.joints[i].parameters.end())
    {
      kd_gains_[i] = std::stod(info_.joints[i].parameters.at("kd"));
    }
    else
    {
      kd_gains_[i] = 0.5;  // Default value
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Joint %s: max_torque=%.2f, max_velocity=%.2f, kp=%.2f, kd=%.2f",
                info_.joints[i].name.c_str(), max_torques_[i], max_velocities_[i],
                kp_gains_[i], kd_gains_[i]);
  }

  if (control_mode_ == "mit_gravity_compensation") {
    // Use a stiffer default MIT gain set when no launch overrides are set.
    for (size_t i = 0; i < 6 && i < kp_gains_.size(); ++i) {
      kp_gains_[i] *= 3.0;
      kd_gains_[i] *= 3.0;
    }

    std::vector<double> mit_kp;
    std::vector<double> mit_kd;
    const bool has_kp_override = parse_gain_vector(
      std::getenv("PANTHERA_MIT_KP"), mit_kp);
    const bool has_kd_override = parse_gain_vector(
      std::getenv("PANTHERA_MIT_KD"), mit_kd);

    if (has_kp_override) {
      std::copy(mit_kp.begin(), mit_kp.end(), kp_gains_.begin());
    } else if (std::getenv("PANTHERA_MIT_KP") != nullptr) {
      RCLCPP_WARN(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "Invalid PANTHERA_MIT_KP; using default MIT Kp values");
    }

    if (has_kd_override) {
      std::copy(mit_kd.begin(), mit_kd.end(), kd_gains_.begin());
    } else if (std::getenv("PANTHERA_MIT_KD") != nullptr) {
      RCLCPP_WARN(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "Invalid PANTHERA_MIT_KD; using default MIT Kd values");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("PantheraHardwareInterface"),
      "MIT gains: Kp=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], "
      "Kd=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      kp_gains_[0], kp_gains_[1], kp_gains_[2],
      kp_gains_[3], kp_gains_[4], kp_gains_[5],
      kd_gains_[0], kd_gains_[1], kd_gains_[2],
      kd_gains_[3], kd_gains_[4], kd_gains_[5]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Configuring...");

  // Initialize Panthera robot
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Initializing Panthera robot with config: %s", config_file_.c_str());

    robot_ = std::make_unique<panthera::Panthera>(config_file_);

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Panthera robot initialized successfully");

    if (control_mode_ == "mit_gravity_compensation") {
      gravity_model_ = std::make_shared<GravityModel>();
      if (!gravity_model_->load(config_file_)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("PantheraHardwareInterface"),
          "Failed to load the URDF/dynamics model for MIT gravity compensation");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "MIT gravity compensation model loaded successfully");
    }
  }
  catch (const std::bad_alloc & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Memory allocation failed during Panthera initialization: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to initialize Panthera robot: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Unknown exception during Panthera initialization");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint states
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read 6 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();

    for (size_t i = 0; i < 6; i++)
    {
      hw_positions_[i] = positions[i];
      hw_velocities_[i] = velocities[i];
      hw_efforts_[i] = torques[i];
      hw_commands_positions_[i] = positions[i];  // Initialize commands to current position
    }

    // Read gripper state (7th joint, index 6 = L_finger_joint) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_positions_[6] = gripper_rad * gripper_rad_to_m_;
      hw_velocities_[6] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[6] = robot_->getCurrentTorqueGripper();
      hw_commands_positions_[6] = hw_positions_[6];  // Initialize commands to current position
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_positions_[7] = -hw_positions_[6];  // Mimic L_finger_joint position (negated, opposite direction)
      hw_velocities_[7] = -hw_velocities_[6];  // Mimic L_finger_joint velocity (negated)
      hw_efforts_[7] = 0.0;  // Passive joint, no actuator
      hw_commands_positions_[7] = hw_positions_[7];
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Initial joint states read successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to read initial joint states: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PantheraHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PantheraHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Skip command interfaces for mimic joints (R_finger_joint)
    if (info_.joints[i].name == "R_finger_joint")
    {
      continue;
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));

    if (use_velocity_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }

    if (use_effort_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Activating...");

  // Read current state and set as command
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Read 6 arm joint positions
    auto positions = robot_->getCurrentPos();
    for (size_t i = 0; i < 6; i++)
    {
      hw_commands_positions_[i] = positions[i];
    }

    // Read gripper position (7th joint, index 6 = L_finger_joint) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_commands_positions_[6] = gripper_rad * gripper_rad_to_m_;
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_commands_positions_[7] = -hw_commands_positions_[6];  // Mimic L_finger_joint position (negated)
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Hardware activated successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to activate hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Deactivating...");

  // Optionally send stop command or hold position
  // For safety, we'll just log the deactivation

  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PantheraHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read joint states from hardware
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();

    // Read 6 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();

    for (size_t i = 0; i < 6; i++)
    {
      hw_positions_[i] = positions[i];
      hw_velocities_[i] = velocities[i];
      hw_efforts_[i] = torques[i];
    }

    // Read gripper state (7th joint, index 6 = L_finger_joint)
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_positions_[6] = gripper_rad * gripper_rad_to_m_;
      hw_velocities_[6] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[6] = robot_->getCurrentTorqueGripper();
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_positions_[7] = -hw_positions_[6];  // Mimic L_finger_joint position (negated, opposite direction)
      hw_velocities_[7] = -hw_velocities_[6];  // Mimic L_finger_joint velocity (negated)
      hw_efforts_[7] = 0.0;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "Failed to read joint states: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PantheraHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commands to hardware
  try
  {
    // Extract first 6 joints for arm control
    std::vector<double> arm_positions(hw_commands_positions_.begin(),
                                       hw_commands_positions_.begin() + 6);
    std::vector<double> arm_velocities(hw_commands_velocities_.begin(),
                                        hw_commands_velocities_.begin() + 6);
    std::vector<double> arm_efforts(hw_commands_efforts_.begin(),
                                     hw_commands_efforts_.begin() + 6);
    std::vector<double> arm_max_torques(max_torques_.begin(), max_torques_.begin() + 6);
    std::vector<double> arm_max_velocities(max_velocities_.begin(), max_velocities_.begin() + 6);
    std::vector<double> arm_kp(kp_gains_.begin(), kp_gains_.begin() + 6);
    std::vector<double> arm_kd(kd_gains_.begin(), kd_gains_.begin() + 6);

    // Control 6 arm joints
    if (control_mode_ == "full_control")
    {
      // Full control mode: use position, velocity, and effort commands
      robot_->posVelTorqueKpKd(arm_positions, arm_velocities,
                               arm_efforts, arm_kp, arm_kd);
    }
    else if (control_mode_ == "mit_gravity_compensation")
    {
      // MIT mode: MoveIt supplies the position/velocity trajectory while the
      // hardware interface adds model-based gravity compensation as feedforward.
      std::vector<double> current_positions(hw_positions_.begin(),
                                            hw_positions_.begin() + 6);
      std::vector<double> gravity_torque =
        gravity_model_ ? gravity_model_->gravity(current_positions) : std::vector<double>();
      if (gravity_torque.size() != 6) {
        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("PantheraHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Gravity compensation torque calculation failed");
        return hardware_interface::return_type::ERROR;
      }
      for (size_t i = 0; i < gravity_torque.size(); ++i) {
        gravity_torque[i] = std::max(
          -arm_max_torques[i], std::min(arm_max_torques[i], gravity_torque[i]));
      }
      robot_->posVelTorqueKpKd(arm_positions, arm_velocities,
                               gravity_torque, arm_kp, arm_kd);
    }
    else if (control_mode_ == "position_velocity")
    {
      // Position-Velocity-MaxTorque control mode
      std::vector<double> velocities = use_velocity_commands_ ? arm_velocities : arm_max_velocities;
      robot_->posVelMaxTorque(arm_positions, velocities, arm_max_torques, false);
    }
    else if (control_mode_ == "pd_control")
    {
      // PD control mode (MIT mode with zero velocity and torque)
      std::vector<double> zero_vel(6, 0.0);
      std::vector<double> zero_torque(6, 0.0);
      robot_->posVelTorqueKpKd(arm_positions, zero_vel, zero_torque, arm_kp, arm_kd);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                            *rclcpp::Clock::make_shared(), 1000,
                            "Unknown control mode: %s", control_mode_.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Control gripper (7th joint, index 6) if present
    // Convert from meters to radians for hardware
    if (info_.joints.size() > 6)
    {
      double gripper_pos_m = hw_commands_positions_[6];
      double gripper_pos_rad = gripper_pos_m / gripper_rad_to_m_;
      double gripper_vel_m = use_velocity_commands_ ? hw_commands_velocities_[6] : max_velocities_[6];
      double gripper_vel_rad = gripper_vel_m / gripper_rad_to_m_;
      double gripper_max_torque = max_torques_[6];

      if (control_mode_ == "full_control")
      {
        // Use MIT mode for gripper
        double gripper_torque = hw_commands_efforts_[6];
        double gripper_kp = kp_gains_[6];
        double gripper_kd = kd_gains_[6];
        robot_->gripperControlMIT(gripper_pos_rad, gripper_vel_rad, gripper_torque,
                                   gripper_kp, gripper_kd);
      }
      else
      {
        // Use posVelMaxTorque mode for gripper
        robot_->gripperControl(gripper_pos_rad, gripper_vel_rad, gripper_max_torque);
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "Failed to write commands: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace panthera_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panthera_hardware::PantheraHardwareInterface, hardware_interface::SystemInterface)
