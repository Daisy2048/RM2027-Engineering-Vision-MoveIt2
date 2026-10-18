# RoboMaster 2027 工程机器人视觉辅助 MoveIt2 规划接口

## 1. 项目介绍

本项目面向 RoboMaster 2027 工程机器人机械臂算法开发。

目前实现了一个基于 ROS2 Humble 和 MoveIt2 的独立 C++ 规划桥接节点。

该节点负责接收视觉系统提供的目标位姿，完成输入数据检查，并调用 MoveIt2 为 Panthera 六轴机械臂生成运动轨迹。

当前版本只负责运动规划，不负责真实机械臂运动执行。

## 2. 软件环境

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- C++17
- Panthera-HT_ROS2
- OMPL / RRTConnect

## 3. 系统架构

```text
模拟视觉目标 / 未来 BIT 视觉输出
                |
                v
      /vision/target_pose_base
                |
                v
          planner_bridge
                |
                v
          输入合法性检查
                |
                v
        MoveGroupInterface
                |
                v
          MoveIt2 规划
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

注意：未来 BIT 视觉数据必须经过坐标变换与姿态格式转换后才能接入。

## 4. ROS2 接口

### 输入话题

名称：

`/vision/target_pose_base`

消息类型：

`geometry_msgs/msg/PoseStamped`

要求：

- 坐标系：base_link
- 位置单位：m
- 姿态格式：标准四元数 xyzw
- 所有数值必须为有限数值
- 四元数必须接近单位长度

### 输出话题

名称：

`/display_planned_path`

消息类型：

`moveit_msgs/msg/DisplayTrajectory`

该话题用于发布规划结果，供 RViz 等工具进行轨迹可视化。

## 5. MoveIt 配置

规划组：

`arm`

末端 Link：

`bat_center`

六轴关节：

`joint1` ~ `joint6`

规划坐标系：

`base_link`

支持通过 config/bridge.yaml 配置：

- planning_group
- tip_link
- planning_time
- planning_attempts
- velocity_scaling
- acceleration_scaling

## 6. 编译方法

```bash
cd ~/rm2027/engineer_moveit_ws

source /opt/ros/humble/setup.bash

source ~/Panthera-HT_ROS2/install/setup.bash

colcon build --symlink-install \
  --packages-select engineer_vision_moveit_bridge
```

## 7. 启动方法

### 终端 1：启动 Panthera MoveIt Demo

```bash
cd ~/Panthera-HT_ROS2

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch panthera_ht_config demo.launch.py
```

### 终端 2：启动规划桥接节点

```bash
cd ~/rm2027/engineer_moveit_ws

source /opt/ros/humble/setup.bash

source ~/Panthera-HT_ROS2/install/setup.bash

source install/setup.bash

ros2 launch engineer_vision_moveit_bridge bridge.launch.py
```

## 8. 模拟目标测试

打开第三个终端，加载 ROS2 环境后执行：

```bash
ros2 topic pub --once \
  /vision/target_pose_base \
  geometry_msgs/msg/PoseStamped \
  '{
    header: {
      frame_id: "base_link"
    },
    pose: {
      position: {
        x: 0.184,
        y: 0.072,
        z: 0.289
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }'
```

注意：

该目标仅为当前 Panthera 演示环境中验证过的测试位姿，不保证适用于其他机器人初始状态或真实机械臂。

## 9. 当前测试结果

2026-10-15 已完成模拟目标规划测试。

一次失败恢复测试后的规划结果：

- 规划成功
- 规划耗时：0.097 s
- 轨迹点数量：5
- 轨迹时长：0.313 s
- 最大关节变化：0.024492 rad
- 六关节均存在非零角度变化

已验证错误坐标系、非法四元数、NaN 数值、不可达目标的处理，以及规划失败后的恢复能力。

上述数据来自一次具体测试，不代表平均规划性能。

## 10. 安全说明

当前版本只调用 MoveGroupInterface::plan()。

不执行运动轨迹。

不向 DM-MC02 发送关节运动指令。

不连接真实机械臂驱动。

不允许将本项目的规划成功结果直接视为实机运动安全验证。

## 11. 已知限制

1. 目前主要使用模拟目标位姿进行测试。
2. 尚未完成真实相机与机械臂的外参标定。
3. 尚未验证真实环境下的目标位姿精度。
4. 尚未实现轨迹到 DM-MC02 的执行接口。
5. 尚未完成针对真实机器人工作空间的安全边界设置。
6. RViz 轨迹实际播放效果仍需单独确认。
7. 尚需进一步检查 ROS2 Action 偶发响应异常与规划超时行为。

## 12. 后续开发

下一阶段计划接入 BIT 视觉算法。

完成相机坐标系到机械臂基座坐标系的变换、目标位姿有效性检查，以及视觉目标与 MoveIt 规划模块之间的集成测试。

真实机械臂执行接口将在独立阶段开发和验收。