# Panthera-HT 第一个 Waypoint Demo：下次开机启动与验收流程

> 适用环境：当前已经替换为真实机械臂 URDF 的 `~/Panthera-HT_ROS2` 工作空间  
> 当前控制器名称：`arm_controller`  
> 当前测试程序：`store_auto`（M1 单 Waypoint Demo）  
> 目标：下次重新开机后，从零启动 MoveIt 环境，并再次跑通第一个固定 Waypoint 测试  
> 当前已知状态：新 URDF 下 M1 已回归通过

---

## 1. 这次 Demo 在验证什么

当前第一个 Waypoint Demo 的完整链路是：

```text
store_auto.cpp
↓
MoveGroupInterface("arm")
↓
/move_group
↓
MoveIt plan()
↓
execute()
↓
arm_controller
↓
joint1 ~ joint6
↓
RViz 中机械臂运动到目标并停下
```

当前 `store_auto` 使用的是一个固定六关节目标，用于验证 MoveIt 最小闭环。

因此这次启动的目的不是做完整自动存矿，而是确认：

```text
ROS2 环境正常
MoveIt 正常
新 URDF 正常
arm_controller 正常
store_auto 正常
Plan 正常
Execute 正常
```

---

## 2. 下次开机后需要两个终端

建议使用两个终端：

```text
Terminal 1
→ 启动整个 MoveIt Demo / RViz 后台

Terminal 2
→ 检查系统状态并运行 store_auto
```

Terminal 1 启动以后必须保持运行，不要关闭。

---

## 3. Terminal 1：启动 MoveIt Demo

进入工作空间：

```bash
cd ~/Panthera-HT_ROS2
```

加载 ROS2：

```bash
source /opt/ros/humble/setup.bash
```

加载 Panthera 工作空间：

```bash
source install/setup.bash
```

启动 MoveIt Demo：

```bash
ros2 launch panthera_ht_config demo.launch.py
```

这条命令不是单纯“打开 RViz”，而是在搭建整个运行环境：

```text
加载 Robot Description / 新 URDF
↓
加载 SRDF
↓
启动 move_group
↓
启动 ros2_control / controller_manager
↓
自动启动 joint_state_broadcaster
↓
自动启动 arm_controller
↓
启动 RViz MotionPlanning
```

当前新版本已经修改为：

```text
arm_controller 自动 spawn
```

因此正常情况下，不再需要手动执行旧命令：

```bash
ros2 run controller_manager spawner panthera_arm_controller --controller-manager /controller_manager
```

旧的 `panthera_arm_controller` 已经统一改为：

```text
arm_controller
```

---

## 4. 等待 RViz 完全启动

看到 RViz 中机械臂模型正常显示后，再进入 Terminal 2。

此时先检查：

```text
模型没有缺失
模型比例正常
机械臂没有飞散
```

如果 Robot Model 本身就异常，不要继续执行 Waypoint。

---

## 5. Terminal 2：加载环境

打开第二个终端：

```bash
cd ~/Panthera-HT_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 6. Terminal 2：确认 MoveIt 后台已经启动

执行：

```bash
ros2 node list | grep move_group
```

正常应至少看到：

```text
/move_group
```

意义：

```text
MoveGroupInterface
需要向 /move_group 发送规划和执行请求
```

如果没有 `/move_group`：

```text
不要运行 store_auto
```

先回 Terminal 1 检查：

```bash
ros2 launch panthera_ht_config demo.launch.py
```

是否正常运行。

---

## 7. Terminal 2：确认 Controller 状态

执行：

```bash
ros2 control list_controllers
```

当前正确结果应类似：

```text
joint_state_broadcaster ... active
arm_controller           ... active
```

重点确认：

```text
arm_controller active
```

因为：

```text
joint_state_broadcaster
= 发布关节当前状态

arm_controller
= 真正接收轨迹并控制 joint1~joint6 执行
```

---

## 8. 如果 `arm_controller` 没有 active

正常情况下新版 `demo.launch.py` 会自动启动 `arm_controller`。

如果没有，先不要直接跑 `store_auto`。

先再次确认：

```bash
ros2 control list_controllers
```

如确实需要手动尝试，可使用当前控制器名称：

```bash
ros2 run controller_manager spawner arm_controller --controller-manager /controller_manager
```

然后再次执行：

```bash
ros2 control list_controllers
```

必须确认：

```text
arm_controller ... active
```

再继续。

---

## 9. Terminal 2：运行第一个 Waypoint Demo

确认：

```text
/move_group 存在       ✅
arm_controller active ✅
```

然后执行：

```bash
ros2 launch panthera_commander store_auto.launch.py
```

---

## 10. 运行后应该发生什么

程序内部流程：

```text
store_auto.launch.py
↓
给 store_auto 注入 MoveIt 参数
↓
启动 store_auto executable
↓
创建 ROS2 Node
↓
创建 MoveGroupInterface("arm")
↓
设置当前状态为起点
↓
设置固定六关节目标
↓
plan()
↓
规划成功
↓
execute()
↓
arm_controller 接收轨迹
↓
六个关节运动
↓
RViz 中模型运动到目标
↓
停止
```

---

## 11. 终端验收标准

理想日志应包含类似：

```text
Ready to take commands for planning group arm
Planning request accepted
Planning request complete!
M1 planning succeeded
Execute request success!
```

核心判断：

```text
Plan 成功          ✅
Execute 成功       ✅
程序正常退出       ✅
```

---

## 12. RViz 验收标准

同时观察 RViz：

```text
机械臂从当前姿态开始运动
↓
到达固定 Waypoint
↓
停在那里
```

不要只看“有动画”。

需要区分：

```text
Planned Path 动画
≠ 真实 Execute
```

之前已经关闭：

```text
Loop Animation
```

否则 RViz 可能不停重复播放规划动画，让人误以为机械臂还在执行。

---

## 13. 如果 RViz 又出现“重影”

如果看到两套机械臂：

```text
MotionPlanning
→ Planning Request
→ Query Goal State
```

检查是否被勾选。

如果只是想看当前真实状态，可以关闭：

```text
Query Goal State
```

---

## 14. 如果出现 `robot_description_semantic` 报错

典型错误：

```text
Could not find parameter robot_description_semantic
Unable to construct robot model
```

通常说明你用了：

```bash
ros2 run panthera_commander store_auto
```

而不是：

```bash
ros2 launch panthera_commander store_auto.launch.py
```

当前正确方式是：

```bash
ros2 launch panthera_commander store_auto.launch.py
```

因为 `store_auto.launch.py` 会把 MoveIt Robot Model / SRDF 等配置传给 `store_auto`。

---

## 15. 如果 Plan 成功但 Execute 失败

如果看到：

```text
Planning request complete!
M1 planning succeeded
Execute request aborted
```

先检查：

```bash
ros2 control list_controllers
```

确认：

```text
arm_controller ... active
```

如果 Controller 不 active，MoveIt 即使能规划出轨迹，也没人真正执行。

---

## 16. 下次开机最简版操作

### Terminal 1

```bash
cd ~/Panthera-HT_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panthera_ht_config demo.launch.py
```

保持运行。

### Terminal 2

```bash
cd ~/Panthera-HT_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查 MoveIt：

```bash
ros2 node list | grep move_group
```

检查 Controller：

```bash
ros2 control list_controllers
```

理想：

```text
joint_state_broadcaster active
arm_controller active
```

最后运行：

```bash
ros2 launch panthera_commander store_auto.launch.py
```

---

## 17. 什么情况下需要重新 `colcon build`

普通重新开机不需要重新编译。

只有修改了这些内容才需要重新 build：

```text
store_auto.cpp
CMakeLists.txt
URDF / Xacro
SRDF
controller YAML
launch 文件
其他 ROS2 源码
```

例如修改 `panthera_commander` 后：

```bash
colcon build --packages-select panthera_commander --event-handlers console_direct+
```

修改 description/config 后，可按实际修改 package 重新编译：

```bash
colcon build --packages-select panthera_ht_ros_description panthera_ht_config --event-handlers console_direct+
```

然后必须：

```bash
source install/setup.bash
```

再运行。

---

## 18. 当前新 URDF 下仍需注意的事项

虽然 M1 已经重新跑通，但以下项目仍未最终确认：

```text
真实 joint limit             UNVERIFIED
真实 bat_center / TCP        UNVERIFIED
base_link 相对真机的方向     UNVERIFIED
pose1 / pose2 真机安全性     UNVERIFIED
真机链路                     未测试
```

因此：

> 当前 Waypoint Demo 只作为仿真 / MoveIt 功能验证，不等于可以直接在真机大幅执行相同动作。

---

## 19. 下一次正式研发从哪里继续

下次 M1 Demo 重新跑通以后，不再继续修改 M1。

直接进入：

```text
M2-0
↓
重新确认新 URDF 下的 base_link
↓
确认 +X / +Y / +Z
↓
确认真实 TCP / bat_center
↓
确认一侧紫色矿仓相对 base_link 的位置
↓
开始建立 PRE_STORE / APPROACH / STORE
```

当前自动存矿 V1：

> 先只做一侧紫色矿仓。

---

## 20. 一句话记忆

下次开机记住：

```text
Terminal 1：
启动整个 MoveIt 舞台

Terminal 2：
确认 move_group + arm_controller
然后运行自己的 store_auto
```

完整最短链：

```text
demo.launch.py
↓
move_group + arm_controller
↓
store_auto.launch.py
↓
plan
↓
execute
↓
Waypoint Demo 成功
```
