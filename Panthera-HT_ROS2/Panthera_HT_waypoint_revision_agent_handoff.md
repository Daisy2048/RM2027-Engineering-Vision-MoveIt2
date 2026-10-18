# Panthera-HT Waypoint 微调方案（交给 Agent 执行）

## 0. 修改目标

本次**不要重做程序架构**，只在现有 `test_full_store_sequence.cpp` 基础上微调 waypoint / 朝向，并把最后返回位置改成真正的初始折叠位置。

当前不做夹爪开合，不加入 RELEASE，不改 gripper，不进入 Gazebo。

主要修改文件：

```text
~/Panthera-HT_ROS2/src/panthera_commander/src/test_full_store_sequence.cpp
```

除非编译需要，否则不要改：

```text
CMakeLists.txt
launch 文件
URDF/Xacro
SRDF
controller 配置
gripper 相关文件
```

---

# 1. 用户期望的真实动作逻辑

机械臂工作空间约定：

```text
正前方：取矿区
左侧：左矿仓
右侧：右矿仓
```

本次先做**左侧矿仓**。

动作顺序应为：

```text
HOME_FOLDED_FRONT
机械臂折叠，面向正前方
        ↓

READY_WITH_ORE / PICK_HIGH_FRONT
机械臂向正前方打开并升高
在正前方完成取矿准备/视为已经拿到矿石
        ↓

LEFT_PRE_STORE
保持较高位置，整体向左侧矿仓转过去
        ↓

LEFT_APPROACH
到左矿仓上方，准备插入
        ↓

LEFT_STORE
沿直线下降进入矿仓
        ↓

LEFT_RETREAT
沿原路线直线上升退出
        ↓

HOME_FOLDED_FRONT
恢复到最开始折叠、面向正前方的位置
```

重点：

```text
APPROACH -> STORE
```

以及：

```text
STORE -> RETREAT
```

必须继续使用 Cartesian Path，保持“直线进入、直线退出”。

其他阶段继续用普通 MoveIt `plan() + execute()`。

---

# 2. 本次修改原则

这不是重新设计 waypoint。

要求：

```text
1. 坐标数值尽量沿用之前已经跑通过的值。
2. 主要变化是机械臂的朝向和流程意义。
3. 正前方阶段使用原来的正前方姿态。
4. 转去左矿仓以后，末端姿态跟着向左旋转。
5. STORE 与 RETREAT 不改变 x/y 和姿态，只改变 z。
6. 最后不再回 PRE_STORE，而是恢复初始折叠位置。
7. 不加入任何夹爪动作。
```

---

# 3. 旧版已验证坐标

旧版使用：

```text
READY_WITH_ORE
x = 0.280
y = 0.000
z = 0.420

PRE_STORE
x = 0.220
y = 0.200
z = 0.400

APPROACH
x = 0.173
y = 0.300
z = 0.350

STORE
x = 0.173
y = 0.300
z = 0.180

RETREAT
x = 0.173
y = 0.300
z = 0.350
```

旧版所有点共用的末端姿态：

```text
q_front = (x, y, z, w)

(0.257, -0.048, -0.125, 0.957)
```

对应曾经实际测得的 APPROACH 姿态约为：

```text
RPY:
roll  ≈ 30.3°
pitch ≈ -1.6°
yaw   ≈ -15.3°
```

---

# 4. 新版第一版 waypoint 建议

## 4.1 HOME_FOLDED_FRONT

意义：

```text
机械臂折叠
面向正前方
作为整个流程的开始和结束
```

在当前 mock 环境里，重新启动 demo 后机械臂的默认关节状态为：

```text
joint1 = 0
joint2 = 0
joint3 = 0
joint4 = 0
joint5 = 0
joint6 = 0
```

因此第一版建议：

```text
HOME_FOLDED_FRONT 使用 joint target：

[0, 0, 0, 0, 0, 0]
```

不要人为猜一个 Cartesian HOME 坐标。

这样可以保证：

```text
开始时的折叠位置
=
结束时恢复的位置
```

注意：

这个 HOME 目前只是 RViz/mock 候选状态，真机前仍需重新确认。

---

## 4.2 READY_WITH_ORE / PICK_HIGH_FRONT

意义：

```text
机械臂从折叠状态向正前方展开并升高
准备取矿 / 当前测试中视为已经拿到矿石
```

第一版保持之前已经跑通过的坐标：

```text
Position:

x = 0.280
y = 0.000
z = 0.420
```

姿态继续使用：

```text
q_front:

x = 0.257
y = -0.048
z = -0.125
w = 0.957
```

因此这一点先不要大改。

---

## 4.3 LEFT_PRE_STORE

意义：

```text
拿到矿石后
保持较高位置
从正前方转向左侧矿仓
```

位置只做轻微调整，继续靠近旧版 PRE_STORE：

```text
Position:

x = 0.220
y = 0.200
z = 0.420
```

说明：

旧版是：

```text
(0.220, 0.200, 0.400)
```

本次建议只把：

```text
z: 0.400 -> 0.420
```

让“转向左矿仓”的过程稍微保持高一点。

如果实际 RViz 中 0.400 更自然，可恢复 0.400；不要为了这 2 cm 重构程序。

---

# 5. 左侧姿态

旧版的问题之一是所有 waypoint 都用了同一个 quaternion。

现在要求：

```text
正前方时使用 q_front
转向左矿仓后使用 q_left
```

假设 ROS 常用坐标约定成立：

```text
base_link +X = 正前方
base_link +Y = 左侧
base_link +Z = 上方
```

并且当前左矿仓坐标一直使用正 y：

```text
y = +0.200
y = +0.300
```

因此第一版将正前方姿态绕 `base_link` 的 Z 轴再旋转：

```text
+90°
```

得到候选左侧姿态：

```text
q_left ≈

x = 0.216
y = 0.148
z = 0.588
w = 0.765
```

它保持原来的 roll / pitch，主要把 yaw 从约：

```text
-15.3°
```

改为约：

```text
+74.7°
```

也就是相对于原姿态向左转约 90°。

重要：

```text
这是第一版 RViz 候选姿态。
```

Agent 不要把它描述成已经标定完成的真机姿态。

如果 RViz 实际显示成向右转，说明本模型的方向约定与预期相反，应反转 yaw 方向再测试，而不是乱改 x/y。

---

# 6. LEFT_APPROACH

位置基本不变：

```text
x = 0.173
y = 0.300
z = 0.350
```

但姿态改为：

```text
q_left = (0.216, 0.148, 0.588, 0.765)
```

所以：

```text
LEFT_APPROACH:

Position:
(0.173, 0.300, 0.350)

Orientation:
(0.216, 0.148, 0.588, 0.765)
```

这一步仍然使用普通 MoveIt 点到点规划。

---

# 7. LEFT_STORE

继续沿用旧坐标：

```text
x = 0.173
y = 0.300
z = 0.180
```

姿态保持：

```text
q_left
```

完整目标：

```text
LEFT_STORE:

Position:
(0.173, 0.300, 0.180)

Orientation:
(0.216, 0.148, 0.588, 0.765)
```

要求：

```text
LEFT_APPROACH -> LEFT_STORE
```

继续使用：

```cpp
computeCartesianPath()
```

不要改成普通 `plan()`。

---

# 8. LEFT_RETREAT

RETREAT 继续严格等于 APPROACH：

```text
LEFT_RETREAT = LEFT_APPROACH
```

即：

```text
Position:
(0.173, 0.300, 0.350)

Orientation:
(0.216, 0.148, 0.588, 0.765)
```

要求：

```text
LEFT_STORE -> LEFT_RETREAT
```

继续使用 Cartesian Path。

应该保持：

```text
x 不变
y 不变
姿态不变
只改变 z
```

从：

```text
z = 0.180
```

回到：

```text
z = 0.350
```

---

# 9. 最终返回 HOME

旧程序当前大致是：

```text
SAFE_HOME = PRE_STORE
```

这个逻辑本次需要修改。

新的要求是：

```text
LEFT_RETREAT
    ↓
HOME_FOLDED_FRONT
```

最终恢复：

```text
joint1..joint6 = [0, 0, 0, 0, 0, 0]
```

推荐 Agent 单独增加一个非常简单的 joint-target helper，例如逻辑上：

```cpp
arm->setStartStateToCurrentState();
arm->setJointValueTarget(home_joint_values);
arm->plan(plan);
arm->execute(plan);
```

不要为了 HOME 改写整个 MoveIt 控制结构。

如果 mock 中 `[0,0,0,0,0,0]` 的实际视觉姿态不符合“折叠面向正前方”，先停止并让用户在 RViz 中确认新的 joint home，不要擅自猜新的六轴角度。

---

# 10. 修改后的完整流程

第一版最终应为：

```text
HOME_FOLDED_FRONT
joint = [0,0,0,0,0,0]

        ↓ 普通 MoveIt

READY_WITH_ORE / PICK_HIGH_FRONT
Position = (0.280, 0.000, 0.420)
Orientation = q_front

        ↓ 普通 MoveIt

LEFT_PRE_STORE
Position = (0.220, 0.200, 0.420)
Orientation = q_left

        ↓ 普通 MoveIt

LEFT_APPROACH
Position = (0.173, 0.300, 0.350)
Orientation = q_left

        ↓ Cartesian 直线下降

LEFT_STORE
Position = (0.173, 0.300, 0.180)
Orientation = q_left

        ↓ Cartesian 直线上升

LEFT_RETREAT
Position = (0.173, 0.300, 0.350)
Orientation = q_left

        ↓ 普通 MoveIt / joint target

HOME_FOLDED_FRONT
joint = [0,0,0,0,0,0]
```

其中：

```text
q_front =
(0.257, -0.048, -0.125, 0.957)

q_left ≈
(0.216, 0.148, 0.588, 0.765)
```

---

# 11. 对现有代码的最小修改要求

Agent 应优先复用现有：

```cpp
move_to_pose(...)
```

不要重新写一套节点。

建议只做以下级别的修改：

```text
1. 将单一 orientation 拆为：
   front_orientation
   left_orientation

2. READY 使用 front_orientation。

3. PRE_STORE 改成 LEFT_PRE_STORE：
   x/y 基本保持
   z 第一版改为 0.420
   orientation 改为 left_orientation。

4. APPROACH / STORE / RETREAT：
   坐标基本不动
   orientation 全部改为 left_orientation。

5. STORE 和 RETREAT 的 Cartesian Path 逻辑完全保留。

6. SAFE_HOME 不再等于 PRE_STORE。

7. 最后改为回 joint home：
   [0,0,0,0,0,0]

8. 不添加任何 gripper / RELEASE 代码。
```

---

# 12. 当前已知的 full-sequence 卡住问题

在本次 waypoint 修改之前，存在一个需要 Agent 注意的已知现象：

```text
test_ready_pre_approach
```

当前能够正常跑通。

但：

```text
test_full_store_sequence
```

曾卡在：

```text
Planning to READY_WITH_ORE...
MoveGroup action client/server ready
```

已确认当时：

```text
arm_controller active
joint_state_broadcaster active
/joint_states 正常
/move_action server = /move_group
/test_full_store_sequence action client 存在
```

两个源码 diff 中，第一次 plan 之前最明显的行为差异是 full sequence 多了：

```cpp
arm->setPoseReferenceFrame("base_link");
```

而已经验证可以运行的 `test_ready_pre_approach.cpp` 没有这一行。

在交给 Agent 修改时，建议：

```text
先让 test_full_store_sequence 的 READY/PRE_STORE/APPROACH 部分
尽量与已经能运行的 test_ready_pre_approach.cpp 保持一致。
```

尤其可以先移除：

```cpp
arm->setPoseReferenceFrame("base_link");
```

因为每个 `PoseStamped` 已经明确：

```cpp
header.frame_id = "base_link";
```

不要为了修这个问题重构线程/executor。

---

# 13. 当前测试约束

保持：

```cpp
arm->setMaxVelocityScalingFactor(0.20);
arm->setMaxAccelerationScalingFactor(0.20);
arm->setPlanningTime(10.0);
```

Cartesian Path 暂时继续使用：

```text
eef_step = 0.005
jump_threshold = 0.0
```

这些仍属于 RViz/mock 仿真测试参数。

不要直接当成真机最终参数。

---

# 14. Agent 修改完成后的验收顺序

启动：

```bash
cd ~/Panthera-HT_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch panthera_ht_config demo.launch.py
```

再开第二个终端：

```bash
cd ~/Panthera-HT_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch panthera_commander test_full_store_sequence.launch.py
```

视觉上应验证：

```text
1. 初始机械臂折叠且面向前方。

2. 机械臂向正前方展开并抬高。

3. 拿矿后的高位动作从正前方转向左侧矿仓。

4. 左侧 APPROACH 位于矿仓上方。

5. APPROACH -> STORE 明显直线下降。

6. STORE -> RETREAT 明显沿原路直线上升。

7. RETREAT 后机械臂收回。

8. 最后恢复到初始折叠、面向前方的 HOME。
```

如果 `q_left` 导致机械臂实际向右侧旋转：

```text
停止继续测试，
保留所有位置数值，
只修正左转姿态的 yaw 符号。
```

---

# 15. 不允许 Agent 顺手做的事情

本次任务范围只到 waypoint 微调。

不要顺手：

```text
修改 gripper
重新加入 RELEASE
修改 URDF
修改 ros2_control
重做 launch 架构
改 PlanningScene
加入 Gazebo
修改真机驱动
自动大规模重构 store_auto.cpp
```

也不要直接把测试代码合入正式 `store_auto.cpp`。

先让：

```text
test_full_store_sequence.cpp
```

在 RViz/mock 中稳定跑通，再进行下一阶段。

---

# 16. 本次第一版参数汇总

```text
HOME_FOLDED_FRONT
joint = [0, 0, 0, 0, 0, 0]

READY_WITH_ORE / PICK_HIGH_FRONT
xyz = (0.280, 0.000, 0.420)
q   = (0.257, -0.048, -0.125, 0.957)

LEFT_PRE_STORE
xyz = (0.220, 0.200, 0.420)
q   = (0.216, 0.148, 0.588, 0.765)

LEFT_APPROACH
xyz = (0.173, 0.300, 0.350)
q   = (0.216, 0.148, 0.588, 0.765)

LEFT_STORE
xyz = (0.173, 0.300, 0.180)
q   = (0.216, 0.148, 0.588, 0.765)

LEFT_RETREAT
xyz = (0.173, 0.300, 0.350)
q   = (0.216, 0.148, 0.588, 0.765)

END
回 HOME_FOLDED_FRONT
joint = [0, 0, 0, 0, 0, 0]
```

这是第一版微调候选，目标是最大程度复用之前已经验证过的坐标，只改变真实动作逻辑和左右方位。先在 RViz/mock 中验证，之后再继续微调。
