# RoboMaster 2027 工程机器人 MoveIt2 规划接口验收报告

## 1. 项目基本信息

**项目名称：** RoboMaster 2027 工程机器人视觉辅助机械臂算法

**模块名称：** engineer_vision_moveit_bridge

**验收日期：** 2026-10-15

**开发环境：**
- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- C++17
- HighTorque Panthera 机械臂模型
- OMPL / RRTConnect

**本次验收范围：**

ROS2 目标位姿接收、输入合法性检查、MoveIt2 运动规划、
六轴关节轨迹生成、规划结果发布和异常处理。

**不包含：**
- 真实相机数据接入
- 相机与机械臂外参标定
- 真实机械臂运动执行
- DM-MC02 下位机运动控制

---

## 2. 项目目标

本模块用于连接视觉感知系统与机械臂运动规划系统。

系统接收目标物体在机械臂基座坐标系下的位姿，
通过 MoveIt2 生成 Panthera 六轴机械臂的运动轨迹。

第一阶段目标：

1. 建立独立的 ROS2 C++ 桥接节点。
2. 接收标准 PoseStamped 目标位姿。
3. 实现基本输入数据合法性检查。
4. 调用 MoveIt2 进行机械臂运动规划。
5. 生成六关节非零运动轨迹。
6. 发布轨迹用于 RViz 可视化。
7. 验证异常目标的处理能力。
8. 保证当前阶段不执行真实机械臂运动。

---

## 3. 系统架构

```text
          模拟视觉位姿
                |
                v
   /vision/target_pose_base
                |
                v
         planner_bridge
                |
                v
        输入数据合法性检查
                |
                v
       MoveGroupInterface
                |
                v
            MoveIt2
                |
         IK / 运动规划
                |
                v
         六关节运动轨迹
                |
                v
      /display_planned_path
                |
                v
         RViz 轨迹预览
```

当前使用模拟视觉目标。

未来真实视觉数据需要经过坐标转换与标定后接入。

---

## 4. 项目结构

```text
engineer_vision_moveit_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── src/
│   └── planner_bridge.cpp
│
├── config/
│   └── bridge.yaml
│
├── launch/
│   └── bridge.launch.py
│
└── docs/
    └── ACCEPTANCE.md
```

主要文件说明：

| 文件 | 作用 |
|---|---|
| planner_bridge.cpp | ROS2 位姿接收与 MoveIt2 规划 |
| bridge.yaml | 规划参数配置 |
| bridge.launch.py | 桥接节点启动与模型配置加载 |
| CMakeLists.txt | C++ 编译与安装 |
| package.xml | ROS2 软件包依赖 |
| README.md | 项目说明及使用方法 |
| ACCEPTANCE.md | 实际测试与验收记录 |

---

## 5. ROS2 接口定义

### 5.1 输入接口

话题名称：

`/vision/target_pose_base`

消息类型：

`geometry_msgs/msg/PoseStamped`

输入要求：

- frame_id：base_link
- 位置单位：m
- 姿态：标准单位四元数
- 四元数顺序：x, y, z, w
- 不允许 NaN 或无穷大
- 不允许零长度四元数

### 5.2 输出接口

话题名称：

`/display_planned_path`

消息类型：

`moveit_msgs/msg/DisplayTrajectory`

作用：

发布 MoveIt 计算得到的轨迹，供 RViz 等工具使用。

本阶段不向真实机械臂控制器发送执行命令。

---

## 6. MoveIt2 配置

| 配置项 | 当前值 |
|---|---|
| Planning Group | arm |
| Planning Frame | base_link |
| Target Link | bat_center |
| 关节数量 | 6 |
| 规划框架 | MoveIt2 |
| 规划算法 | OMPL / RRTConnect |

六个关节名称：

```text
joint1
joint2
joint3
joint4
joint5
joint6
```

### 6.1 规划参数

参数文件：

`config/bridge.yaml`

当前配置：

| 参数 | 数值 |
|---|---:|
| planning_time | 10.0 s |
| planning_attempts | 10 |
| velocity_scaling | 0.1 |
| acceleration_scaling | 0.1 |

上述参数已通过 ROS2 参数查询验证。

---

## 7. 编译验收

### 编译命令

```bash
cd ~/rm2027/engineer_moveit_ws

source /opt/ros/humble/setup.bash

source ~/Panthera-HT_ROS2/install/setup.bash

colcon build --symlink-install \
  --packages-select engineer_vision_moveit_bridge
```

### 测试结果

```text
Starting >>> engineer_vision_moveit_bridge

Finished <<< engineer_vision_moveit_bridge

Summary: 1 package finished
```

**验收结果：通过。**

C++ 软件包能够成功编译。

---

## 8. Launch 启动验收

### 启动命令

先启动 Panthera MoveIt Demo。

随后启动桥接节点：

```bash
cd ~/rm2027/engineer_moveit_ws

source /opt/ros/humble/setup.bash

source ~/Panthera-HT_ROS2/install/setup.bash

source install/setup.bash

ros2 launch engineer_vision_moveit_bridge bridge.launch.py
```

### 实际日志

```text
Loaded robot model

Ready to take commands for planning group arm.

Planning parameters:
time=10.00
attempts=10
velocity=0.10
acceleration=0.10

MoveIt planning group: arm

MoveIt planning frame: base_link

Target tip link: bat_center

Planner bridge ready.
Waiting for /vision/target_pose_base
```

**验收结果：通过。**

桥接节点可以通过 Launch 启动，并加载机器人模型及规划参数。

本 Launch 不负责启动 MoveIt Demo。

---

## 9. 正常目标规划测试

### 9.1 测试目标

通过 ROS2 发布模拟末端位姿：

```yaml
header:
  frame_id: base_link

pose:
  position:
    x: 0.184
    y: 0.072
    z: 0.289

  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### 9.2 实际规划结果

2026-10-15 使用 Launch 启动桥接节点后进行测试。

实际日志：

```text
Planning SUCCESS! Time: 0.111 s

Trajectory points: 5

Trajectory duration: 0.316 s

Max joint displacement: 0.025031 rad

NON-ZERO TRAJECTORY VERIFIED!

Trajectory published to RViz
```

### 9.3 六关节轨迹数据

| 关节 | 起始角度(rad) | 目标角度(rad) | 变化量(rad) |
|---|---:|---:|---:|
| joint1 | 0.000000 | 0.006795 | +0.006795 |
| joint2 | 0.000000 | 0.008684 | +0.008684 |
| joint3 | 0.000000 | 0.025031 | +0.025031 |
| joint4 | 0.000000 | -0.008110 | -0.008110 |
| joint5 | 0.000000 | -0.015860 | -0.015860 |
| joint6 | 0.000000 | 0.010629 | +0.010629 |

### 9.4 验收结论

通过。

系统成功：

1. 接收模拟目标位姿。
2. 调用 MoveIt2。
3. 完成运动规划。
4. 生成 5 个轨迹点。
5. 生成六关节非零运动轨迹。
6. 发布 DisplayTrajectory 消息。

该测试证明规划接口软件链路有效。

不代表真实机械臂末端定位误差或实机控制精度。

---

## 10. 异常输入测试

### T01：错误坐标系

输入：

```text
frame_id = camera
```

预期：

拒绝不属于 base_link 的目标。

实际日志：

```text
Frame mismatch:
received 'camera', expected 'base_link'
```

结果：

**PASS**

错误坐标系被识别，未进入运动规划流程。

---

### T02：非法四元数

输入：

```text
orientation = (0, 0, 0, 0)
```

预期：

拒绝无效四元数。

实际日志：

```text
Invalid quaternion norm: 0.000000
```

结果：

**PASS**

非法四元数被正确拒绝。

---

### T03：非法浮点数

输入：

```text
position.x = NaN
```

预期：

拒绝非有限数值。

实际日志：

```text
Received target: x=-nan

Invalid target: non-finite value
```

结果：

**PASS**

系统识别并拒绝了 NaN 输入。

---

### T04：不可达目标

输入：

```text
X = 10.0 m
Y = 0.072 m
Z = 0.289 m
```

预期：

MoveIt 无法找到有效轨迹，并返回规划失败。

实际日志：

```text
Received target: x=10.000 y=0.072 z=0.289

Starting motion planning...

Planning request accepted

Planning request aborted

MoveGroupInterface::plan() failed or timeout reached

Planning FAILED.
Check IK, reachability and collision.
```

结果：

**PASS**

MoveIt 未返回规划成功结果，
桥接节点正确记录规划失败。

此次测试从发起请求到失败返回约 20 秒。

当前 planning_time 配置为 10 秒，
需要后续分析完整请求耗时与规划时间的关系。

---

### T05：规划失败后恢复

测试过程：

1. 发送不可达目标，等待规划失败。
2. 不重新启动桥接节点。
3. 再次发送已经验证过的合法目标。
4. 检查系统能否重新生成运动轨迹。

实际日志：

```text
Planning SUCCESS! Time: 0.097 s

Trajectory points: 5

Trajectory duration: 0.313 s

Max joint displacement: 0.024492 rad

NON-ZERO TRAJECTORY VERIFIED!

Trajectory published to RViz
```

结果：

**PASS**

系统在一次规划失败后，
仍然能够成功处理新的合法目标。

---

## 11. 验收结果汇总

| 编号 | 测试项目 | 结果 |
|---|---|---|
| B01 | ROS2 软件包编译 | PASS |
| B02 | Launch 启动 | PASS |
| B03 | 机器人模型加载 | PASS |
| B04 | 规划参数加载 | PASS |
| F01 | 合法目标规划 | PASS |
| F02 | 六关节非零轨迹生成 | PASS |
| F03 | 规划轨迹消息发布 | PASS |
| T01 | 错误坐标系 | PASS |
| T02 | 非法四元数 | PASS |
| T03 | NaN 非法数值 | PASS |
| T04 | 不可达目标 | PASS |
| T05 | 规划失败后恢复 | PASS |

### 尚未验证的项目

| 项目 | 状态 |
|---|---|
| RViz 轨迹动画实际播放 | 待确认 |
| 真实视觉位姿接入 | 未开始 |
| 相机与机械臂外参标定 | 未开始 |
| 实机轨迹执行 | 未开始 |
| DM-MC02 轨迹执行接口 | 未开始 |

---

## 12. 已知问题

### 12.1 KDL 根连杆惯性警告

运行时出现：

```text
The root link base_link has an inertia specified
in the URDF, but KDL does not support
a root link with an inertia.
```

当前不影响已完成的规划测试。

尚未修改 Panthera 原始 URDF。

### 12.2 规划请求耗时

不可达目标测试中，
规划失败返回约耗时 20 秒。

与 planning_time=10.0 s 不完全一致。

需要进一步调查 MoveIt 规划流程和 ROS2 Action 通信。

### 12.3 Action 响应日志

早期调试阶段曾出现：

```text
unknown goal response, ignoring...

unknown result response, ignoring...
```

规划功能未因此中断。

但尚未确认异常产生原因，
需在后续稳定性测试中继续观察。

### 12.4 工作空间边界

当前主要依赖 MoveIt 判断目标可达性。

尚未实现针对真实比赛机器人的工作空间安全边界。

### 12.5 RViz 显示验证

已确认桥接节点发布规划轨迹消息。

尚需通过实际 RViz 界面进一步确认轨迹动画播放效果。

---

## 13. 安全说明

本阶段仅开发运动规划接口。

程序不执行规划轨迹。

不调用 MoveGroupInterface::execute()。

不调用 MoveGroupInterface::move()。

不向 DM-MC02 发送运动指令。

不连接真实机械臂电机控制路径。

当前测试结果不构成实机运动安全认证。

真实部署前必须进一步完成：

- 机械臂模型与实机一致性验证
- 关节限位验证
- 碰撞模型验证
- 工作空间安全约束
- 真实控制器通信验证
- 急停与异常保护
- 分阶段低速实机测试

---

## 14. 第一阶段验收结论

本次完成了 RoboMaster 2027 工程机器人
ROS2 → MoveIt2 规划接口的第一阶段研发。

已经验证：

- 独立 C++ 桥接节点能够运行。
- ROS2 目标位姿能够正确接收。
- MoveIt2 能够生成六关节非零运动轨迹。
- Launch 启动和 YAML 参数配置正常。
- 基础异常输入能够正确处理。
- 不可达目标规划失败后，节点能够恢复正常规划。

**结论：**

第一阶段核心软件规划功能验收通过。

当前成果可以作为后续视觉位姿接入的规划模块基础。

本结论不包含真实视觉定位精度、
RViz 动画显示效果及实机运动执行验收。

---

## 15. 后续研发方向

下一阶段计划开发视觉与规划之间的位姿转换接口。

目标：

```text
BIT 视觉系统
     |
     v
YOLO-Pose
     |
     v
PnP 位姿计算
     |
     v
相机坐标系 → base_link
     |
     v
/vision/target_pose_base
     |
     v
engineer_vision_moveit_bridge
     |
     v
MoveIt2 运动规划
```

需要进一步完成：

1. 视觉输出位姿格式检查。
2. 坐标单位转换。
3. 相机与机械臂 TF 坐标变换。
4. 外参标定。
5. 目标抓取位姿与末端工具偏移。
6. 视觉目标有效性和安全边界判断。
7. 视觉目标输入与 MoveIt 规划联调。

真实机械臂轨迹执行与 DM-MC02 通信属于后续独立开发阶段。