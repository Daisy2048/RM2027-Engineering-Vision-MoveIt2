# ArmStatus.msg - Arm status feedback (adapted from Piper's PiperStatusMsg)
std_msgs/Header header

# Overall arm state
bool arm_enabled              # Whether the arm is enabled and accepting commands
uint8 motion_status           # 0=idle, 1=moving, 2=error
string error_message          # Human-readable error description

# Per-motor mode and fault
uint8[6] motor_modes          # Control mode per motor
uint8[6] motor_faults         # Fault code per motor (0 = no fault)

# Joint limit flags: true means the joint has hit its position limit
bool[6] joint_at_limit

# Gripper state
float64 gripper_position      # Gripper position
uint8 gripper_fault           # Gripper motor fault code
