# PosCmd.msg - Combined end-effector pose + gripper command
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper              # Gripper value (0.0=close, 1.0=fully open)
uint8 mode1                  # 0=joint-space planning, 1=cartesian path
uint8 mode2                  # Reserved for future use
