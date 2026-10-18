# Panthera C++ SDK 迁移文档

## 概述

本次迁移将 `panthera_python` 中的 Panthera 机械臂 Python SDK 迁移到 C++ 中，实现了除运动学动力学之外的核心功能。

## 文件结构

### 新增文件

```
panthera_cpp/
├── include/panthera/
│   └── Panthera.hpp                 # Panthera 类头文件
├── src/panthera/
│   └── Panthera.cpp                 # Panthera 类实现
└── examples/
    ├── 0_robot_get_state.cpp        # 获取机器人状态示例
    ├── 0_robot_set_zero.cpp         # 设置零位示例
    ├── 1_PD_control.cpp             # PD 控制示例
    └── 1_PosVel_control.cpp         # 位置速度控制示例
```

## Panthera 类功能

### 已实现功能（不含运动学动力学）

#### 1. 配置管理
- ✅ 配置文件加载（YAML 格式）
- ✅ 关节限位读取和检查
- ✅ 关节名称配置

#### 2. 状态获取
- ✅ `getCurrentPos()` - 获取当前关节位置
- ✅ `getCurrentVel()` - 获取当前关节速度
- ✅ `getCurrentTorque()` - 获取当前关节力矩
- ✅ `getCurrentPosGripper()` - 获取夹爪位置
- ✅ `getCurrentVelGripper()` - 获取夹爪速度
- ✅ `getCurrentTorqueGripper()` - 获取夹爪力矩

#### 3. 关节控制
- ✅ `posVelMaxTorque()` - 位置速度最大力矩控制模式
  - 支持阻塞等待
  - 自动关节限位检查
- ✅ `posVelTorqueKpKd()` - 五参数 MIT 控制模式
  - 自动关节限位检查

#### 4. 夹爪控制
- ✅ `gripperControl()` - 夹爪位置速度最大力矩控制
- ✅ `gripperControlMIT()` - 夹爪 5 参数 MIT 控制
- ✅ `gripperOpen()` - 打开夹爪
- ✅ `gripperClose()` - 关闭夹爪

#### 5. 位置检测
- ✅ `checkPositionReached()` - 检查位置是否到达
- ✅ `waitForPosition()` - 等待位置到达（阻塞）

#### 6. 工具方法
- ✅ `getMotorCount()` - 获取电机数量
- ✅ `getJointLimits()` - 获取关节限位

### 未实现功能（运动学动力学）

以下功能将在后续迁移中实现：

- ❌ `forward_kinematics()` - 正运动学
- ❌ `inverse_kinematics()` - 逆运动学
- ❌ `get_Gravity()` - 重力补偿
- ❌ `get_Coriolis()` - 科氏力
- ❌ `get_Mass_Matrix()` - 质量矩阵
- ❌ `get_Inertia_Terms()` - 惯性力矩
- ❌ `get_Dynamics()` - 完整动力学
- ❌ `get_friction_compensation()` - 摩擦力补偿
- ❌ `quintic_interpolation()` - 五次插值
- ❌ `septic_interpolation()` - 七次插值
- ❌ `septic_interpolation_with_velocity()` - 带速度边界的七次插值

## 示例程序

### 1. 0_robot_get_state.cpp
获取并实时显示机器人状态（位置、速度、力矩）

**对应 Python 版本**: `panthera_python/scripts/0_robot_get_state.py`

**功能**:
- 实时获取 6 个关节和夹爪的状态
- 每 0.5 秒更新一次
- 支持 Ctrl+C 优雅退出

### 2. 0_robot_set_zero.cpp
设置机器人零位并显示状态

**对应 Python 版本**: `panthera_python/scripts/0_robot_set_zero.py`

**功能**:
- 将当前位置设置为零位
- 实时显示关节状态

### 3. 1_PD_control.cpp
简单的 PD 控制示例

**对应 Python 版本**: `panthera_python/scripts/1_PD_control.py`

**功能**:
- 使用 MIT 5 参数控制模式
- 目标位置: [0.0, 0.7, 0.7, -0.1, 0.0, 0.0]
- 可调节的 Kp 和 Kd 增益
- 实时打印关节状态

### 4. 1_PosVel_control.cpp
位置速度控制示例

**对应 Python 版本**: `panthera_python/scripts/1_PosVel_control.py`

**功能**:
- 使用位置速度最大力矩控制模式
- 阻塞等待位置到达
- 夹爪开合控制
- 多点轨迹运动

## 编译与使用

### 编译

```bash
cd panthera_cpp
mkdir -p build && cd build
cmake ..
make
```

### 运行示例

```bash
# 在 build 目录下运行
./0_robot_get_state
./0_robot_set_zero
./1_PD_control
./1_PosVel_control
```

## 与 Python SDK 的差异

### 接口命名
- Python: 使用蛇形命名法 (snake_case)
  - `get_current_pos()`, `pos_vel_MAXtqe()`
- C++: 使用驼峰命名法 (camelCase)
  - `getCurrentPos()`, `posVelMaxTorque()`

### 数据类型
- Python: 使用 `list` 和 `numpy.ndarray`
- C++: 使用 `std::vector<double>`

### 配置文件
- 两者都使用相同的 YAML 配置文件格式
- C++ 版本使用 `yaml-cpp` 库解析

### 继承关系
- Python: `Panthera` 继承自 `htr.Robot`
- C++: `panthera::Panthera` 继承自 `hightorque_robot::robot`

## 依赖项

- **yaml-cpp**: YAML 配置文件解析
- **libserialport**: 串口通信
- **lcm**: 轻量级通信与编组
- **pthread**: 多线程支持

## CMake 配置修改

在 `CMakeLists.txt` 中进行了以下修改：

1. **添加源文件**:
   - `src/panthera/Panthera.cpp`

2. **添加头文件目录**:
   - `include/panthera`

3. **添加示例程序**:
   - `0_robot_get_state`
   - `0_robot_set_zero`
   - `1_PD_control`
   - `1_PosVel_control`

4. **更新 PUBLIC_HEADER**:
   - `include/panthera/Panthera.hpp`

## 后续工作

1. **运动学动力学功能迁移**
   - 集成 Pinocchio 或 KDL 库
   - 实现正逆运动学
   - 实现动力学计算
   - 实现轨迹插值

2. **更多示例程序**
   - 重力补偿控制
   - 阻抗控制
   - 轨迹记录与回放
   - 遥操作控制

3. **优化与增强**
   - 添加更多错误检查
   - 性能优化
   - 添加单元测试
   - 完善文档

## 注意事项

1. **线程安全**: 当前实现未考虑线程安全，多线程使用时需要额外的同步机制

2. **配置文件路径**:
   - 默认配置文件路径: `../robot_param/Follower.yaml`
   - 可通过命令行参数指定自定义配置文件

3. **关节限位**:
   - 控制命令会自动检查关节限位
   - 超出限位的命令会被拒绝并打印警告信息

4. **电机掉电**:
   - 程序结束后电机会自动掉电
   - 使用时请注意安全

## 参考

- Python SDK: `panthera_python/scripts/Panthera_lib/Panthera.py`
- 配置文件: `panthera_python/robot_param/Follower.yaml`
- Python 示例: `panthera_python/scripts/`
