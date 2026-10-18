# 正弦轨迹控制程序

## 描述

这个程序实现了机械臂关节的正弦轨迹跟踪控制，所有关节沿着正弦函数轨迹运动。

## 文件位置

`/home/tk/aaa_ws/src/gripper_robot_commander/src/sin_trajectory_control.cpp`

## 编译

```bash
cd /home/tk/aaa_ws
colcon build --packages-select gripper_robot_commander
source install/setup.bash
```

## 运行

```bash
# 使用默认配置文件
ros2 run gripper_robot_commander sin_trajectory_control

# 或指定自定义配置文件
ros2 run gripper_robot_commander sin_trajectory_control /path/to/config.yaml
```

## 控制参数

代码中的可调节参数（位于`sin_trajectory_control.cpp`）：

```cpp
const double frequency = 0.45;       // 正弦波频率 (Hz)，范围: 0.1-2.0
const double duration = 600.0;       // 运动持续时间 (秒)
const int control_rate = 500;        // 控制频率 (Hz)
```

### 频率调节
- **0.1 Hz**: 缓慢、平滑的运动
- **0.45 Hz**: 默认值，平衡的速度
- **1.0 Hz**: 较快的运动
- **2.0 Hz**: 快速运动

## 初始位置

程序会先将机械臂移动到以下初始位置：

```cpp
{-0.3, 1.1, 1.1, 0.2, -0.3, 0.0}  // 各关节角度(弧度)
```

## 关节限位

程序为每个关节设置了安全的角度限位：

| 关节 | 下限 | 上限 | 说明 |
|------|------|------|------|
| 关节1 | -π | π | ±180° |
| 关节2 | 0 | π | 0到180° |
| 关节3 | 0 | π | 0到180° |
| 关节4 | 0 | π/2 | 0到90° |
| 关节5 | -π/2 | π/2 | ±90° |
| 关节6 | -π | π | ±180° |

## 振幅自动调整

程序会自动计算每个关节的安全振幅，考虑以下因素：
1. 初始位置到上下限的距离
2. 预设的默认振幅：`[0.4, 0.6, 0.6, 0.5, 0.4, 0.0]`
3. 最终振幅取两者中的较小值，并乘以0.8的安全系数

## 正弦轨迹公式

```
位置: x(t) = x₀ + A·sin(ωt + φ)
速度: v(t) = A·ω·cos(ωt + φ)
```

其中：
- `x₀`: 中心位置（初始位置）
- `A`: 振幅
- `ω = 2πf`: 角频率
- `φ`: 相位偏移（默认为0）

## 退出程序

按 `Ctrl+C` 可随时中断程序，机械臂将：
1. 返回中心位置
2. 返回零位
3. 停止所有电机

## 输出示例

```
移动到初始位置...
到达初始位置
获取初始位置...
中心位置: [-0.300, 1.100, 1.100, 0.200, -0.300, 0.000]
调整后的振幅: [0.320, 0.480, 0.480, 0.240, 0.320, 0.000] rad
各关节最大速度: [0.904, 1.357, 1.357, 0.679, 0.904, 0.000] rad/s

开始正弦轨迹运动...
频率: 0.45 Hz, 持续时间: 600 秒
时间: 10.50s | 关节1位置: 0.015 | 关节2位置: 1.550 | 关节3位置: 1.550
```

## 注意事项

1. **确保工作空间安全**: 运行前确保机械臂周围有足够的空间
2. **观察首次运行**: 首次运行时建议降低频率和持续时间
3. **紧急停止**: 如需立即停止，按 `Ctrl+C`
4. **配置文件**: 默认使用 `Follower_absolute.yaml`，可自定义

## 故障排除

### 编译错误

如果遇到找不到 `panthera/Panthera.hpp` 的错误：

```bash
# 重新编译 hightorque_robot 包
colcon build --packages-select hightorque_robot

# 然后重新编译 gripper_robot_commander
colcon build --packages-select gripper_robot_commander
```

### 运行时错误

如果遇到库加载错误：

```bash
source /home/tk/aaa_ws/install/setup.bash
ros2 run gripper_robot_commander sin_trajectory_control
```

## 与Python版本的对比

| 特性 | Python版本 | C++版本 |
|------|-----------|---------|
| 文件位置 | `/home/tk/下载/...` | `/home/tk/aaa_ws/...` |
| 性能 | 较低 | 更高 |
| 控制频率 | 500 Hz | 500 Hz |
| 功能 | 完全相同 | 完全相同 |
