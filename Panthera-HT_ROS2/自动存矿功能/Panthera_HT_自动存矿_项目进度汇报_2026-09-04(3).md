# Panthera-HT 工程机器人自动存矿项目进度汇报

> 项目方向：工程机器人机械臂自动存矿  
> 当前技术路线：ROS2 Humble + MoveIt2 + ros2_control  
> 当前阶段：M0、M1 已完成，准备进入 M2  
> 日期：2026-09-04

---

## 一、当前目标

目前自动存矿功能的目标是：

> 在机器人已经完成取矿、机械臂已经夹持矿石的前提下，使机械臂能够自动完成从持矿姿态到矿仓附近、插入矿仓、释放矿石、退出并回到安全位的完整流程。

当前规划的动作流程为：

```text
READY_WITH_ORE
↓
PRE_STORE
↓
APPROACH
↓
STORE
↓
RELEASE
↓
DETACH
↓
RETREAT
↓
SAFE_HOME
```

目前优先完成仿真闭环，后续再迁移到真机。

---

## 二、当前完成进度

### 1. M0：ROS2 + MoveIt 官方环境部署完成

已完成 Panthera-HT_ROS2 项目的完整部署，并成功跑通官方 MoveIt Demo。

当前环境：

```text
Ubuntu 22.04
ROS2 Humble
MoveIt2
ros2_control
Panthera-HT_ROS2
```

已完成：

- ROS2 Humble 安装
- MoveIt2 安装
- Panthera-HT_ROS2 工程克隆
- 相关依赖补全
- 全工程 `colcon build` 编译成功
- 官方 `demo.launch.py` 启动成功
- RViz 中机械臂模型正常显示
- 官方 MoveIt Plan & Execute 可正常工作

M0 当前状态：

```text
M0 ✅ 已完成
```

---

### 2. M1：自定义单目标 MoveIt 控制闭环完成

已完成自定义 ROS2 C++ 程序：

```text
store_auto.cpp
```

该程序目前能够：

```text
获取当前机械臂状态
↓
设置六关节目标
↓
调用 MoveIt 规划
↓
生成轨迹
↓
调用 execute()
↓
通过 panthera_arm_controller 执行
↓
RViz 中机械臂运动到目标并停下
```

M1 当前使用官方 SRDF 中的 `pose2` 作为已知安全目标：

```text
joint1 = -0.1989
joint2 =  0.7072
joint3 =  0.6630
joint4 = -0.4862
joint5 =  0.5917
joint6 = -0.7597
```

当前已验证：

- `MoveGroupInterface("arm")` 可以正常创建
- `setJointValueTarget()` 正常
- `plan()` 正常
- `execute()` 正常
- `panthera_arm_controller` 可正常执行六轴轨迹
- 机械臂运动后能够正常停止

M1 当前状态：

```text
M1 ✅ 已完成
```

---

## 三、M1 过程中遇到的主要问题

### 1. `robot_description_semantic` 缺失

最开始直接运行：

```bash
ros2 run panthera_commander store_auto
```

出现：

```text
Could not find parameter robot_description_semantic
Unable to construct robot model
```

原因是：

> `store_auto` 裸启动时没有获得完整的 MoveIt Robot Model / SRDF 参数。

解决方式：

基于官方 `commander.launch.py` 新建：

```text
store_auto.launch.py
```

通过：

```python
moveit_config.to_dict()
```

把 MoveIt 配置传给 `store_auto`。

之后改为：

```bash
ros2 launch panthera_commander store_auto.launch.py
```

问题解决。

---

### 2. `plan()` 成功，但 `execute()` 被 aborted

当时日志表现为：

```text
Planning request complete!
M1 planning succeeded
Execute request accepted
Execute request aborted
```

检查：

```bash
ros2 control list_controllers
```

发现只有：

```text
joint_state_broadcaster active
```

而负责六轴轨迹执行的：

```text
panthera_arm_controller
```

没有启动。

随后手动启动：

```bash
ros2 run controller_manager spawner panthera_arm_controller --controller-manager /controller_manager
```

再确认：

```text
panthera_arm_controller ... active
```

之后 `execute()` 正常执行。

---

## 四、当前已经打通的完整链路

目前已经验证通过的控制链为：

```text
store_auto.cpp
↓
ROS2 Node
↓
MoveGroupInterface
↓
/move_group
↓
MoveIt Planning
↓
execute()
↓
ros2_control
↓
panthera_arm_controller
↓
joint1 ~ joint6
↓
RViz 仿真机械臂运动
```

这意味着：

> 当前已经解决“自定义程序能否通过 MoveIt 控制 Panthera 机械臂”的基础问题。

后续工作可以正式进入自动存矿动作本体。

---

## 五、当前项目总体进度

当前阶段划分如下：

```text
M0  官方 ROS2 + MoveIt 基线
M1  自定义单 Waypoint MoveIt 最小闭环
M2  自动存矿 Waypoint 建立
M3  最小完整运动链
M4  Planning Scene + 矿仓 + 矿石 + Attach/Detach
M5  状态机 + ERROR
M6  完整仿真验收
M7  真机迁移
```

当前完成情况：

```text
M0 ✅
M1 ✅
M2 ⏳
M3 ⏳
M4 ⏳
M5 ⏳
M6 ⏳
M7 ⏳
```

目前可以认为：

> 基础环境和最小运动闭环已经完成，项目正式进入自动存矿功能开发阶段。

---

## 六、下一阶段计划

### M2：建立自动存矿关键 Waypoint

计划建立：

```text
READY_WITH_ORE
PRE_STORE
APPROACH
STORE
RETREAT
SAFE_HOME
```

主要任务：

- 确定各个目标姿态
- 明确哪些点使用 Joint Target
- 明确哪些点使用 Pose Target
- 每个 Waypoint 单独 Plan
- 每个 Waypoint 单独 Execute
- 检查姿态是否合理
- 检查路径是否存在明显碰撞或奇异姿态

M2 仍然采用：

```text
一个点
↓
单独验证
↓
再加入下一个点
```

---

### M3：建立完整动作链

计划：

```text
READY
↓
PRE_STORE
↓
APPROACH
↓
STORE
↓
RETREAT
↓
HOME
```

初步规划方式：

```text
READY → PRE_STORE
普通 MoveIt Planning

PRE_STORE → APPROACH
普通 MoveIt Planning

APPROACH → STORE
Cartesian Path

STORE → RETREAT
Cartesian Path

RETREAT → SAFE_HOME
普通 MoveIt Planning
```

其中插入矿仓和退出矿仓部分，优先使用笛卡尔直线路径。

---

### M4：加入矿仓和矿石模型

后续将加入：

```text
PlanningSceneInterface
Storage CollisionObject
Ore CollisionObject
Attach
Detach
```

目标是让 MoveIt 真正考虑：

- 机械臂与矿仓碰撞
- 矿石与环境碰撞
- 机械臂持矿时矿石随末端移动
- 释放后矿石重新成为环境物体

---

### M5：状态机与异常处理

计划状态机：

```text
IDLE
↓
PREPARE
↓
MOVE_PRE_STORE
↓
MOVE_APPROACH
↓
INSERT
↓
RELEASE
↓
DETACH
↓
RETREAT
↓
RETURN_HOME
↓
WAIT_NEXT_TASK
```

异常统一进入：

```text
ERROR
```

包括：

- 规划失败
- 执行失败
- Cartesian Path 完成率不足
- 目标不可达
- Attach / Detach 异常

---

### M6：完整仿真验收

目标：

```text
持矿
↓
自动接近矿仓
↓
插入
↓
释放
↓
退出
↓
回安全位
```

计划至少连续重复 3 次，验证：

- 规划稳定性
- 执行稳定性
- 状态机稳定性
- 碰撞场景合理性

---

### M7：真机迁移

真机阶段暂未开始。

后续将按：

```text
只读关节状态
↓
极小幅度运动
↓
单 Waypoint
↓
双 Waypoint
↓
空载完整流程
↓
夹爪
↓
低速持矿
↓
完整自动存矿
```

逐步验证。

真机阶段会严格限制：

```text
速度
加速度
运动范围
人员安全距离
硬件急停
```

---

## 七、目前需要继续解决的问题

当前主要待解决内容：

1. 自动存矿真实 Waypoint 尚未确定
2. 矿仓几何位置还未加入 Planning Scene
3. 矿石 CollisionObject 尚未建立
4. Attach / Detach 尚未实现
5. Cartesian Path 尚未接入正式流程
6. 自动存矿状态机尚未实现
7. Controller 当前仍需要手动启动，后续可以考虑整合进统一 launch
8. 还没有进行真机验证

---

## 八、当前阶段结论

目前项目已经从：

```text
只能运行官方 Panthera MoveIt Demo
```

推进到：

```text
能够编写自己的 ROS2 C++ 节点
↓
调用 MoveIt
↓
完成六轴运动规划
↓
通过 ros2_control
↓
驱动 Panthera 仿真机械臂执行目标运动
```

当前最大的基础风险已经排除：

> 自定义程序控制 Panthera MoveIt 机械臂的最小闭环已经打通。

下一阶段工作重点将从：

```text
“能不能控制机械臂”
```

转向：

```text
“自动存矿应该怎么走、怎么安全插入、怎么带矿、怎么释放、怎么异常停止”
```

下一步进入：

# M2 —— 自动存矿关键 Waypoint 建立与逐点验证
