# RM2027 MoveIt 2---STM32 C板通信开发记录

-   日期：2026-11-06
-   工程目录：`/home/daisy/rm2027`
-   通信工作区：`/home/daisy/rm2027/cboard_comm_ws`
-   环境：Ubuntu 22.04 / ROS 2 Humble
-   阶段：MoveIt 2 到 STM32 C板通信链路联调

## 1. 开发目标

本阶段目标是验证 MoveIt 2 规划轨迹经 ROS 2 桥接后，能够通过 USB CDC
串口发送到 STM32 C板，并检查上位机能否接收
C板返回的数据。本阶段不驱动机械臂电机，不以机械臂实际运动作为验收条件。

## 2. 通信架构

``` text
RViz Plan / Execute
        |
        v
MoveIt 2 (move_group)
        |
        | FollowJointTrajectory Action
        v
trajectory_bridge_node
        |
        | /cboard/command (std_msgs/msg/String)
        v
cboard_comm_node
        |
        | JSON解析、协议编码、串口发送
        v
USB CDC（115200 baud）
        |
        v
STM32 C板

STM32 C板反馈 -> USB CDC -> cboard_comm_node
                              |-> /cboard/feedback
                              |-> /cboard/status
```

## 3. 核心模块

### 3.1 MoveIt 控制器配置

配置文件：`Panthera-HT_ROS2/src/panthera_ht_config/config/moveit_controllers_cboard.yaml`

-   控制器：`cboard_controller`
-   Action：`/cboard_controller/follow_joint_trajectory`
-   关节：`joint1` 至 `joint6`

运行时曾确认 MoveIt 参数为：

``` text
String values are: ['cboard_controller']
```

### 3.2 轨迹桥接节点

源码：`cboard_comm_ws/src/rm2027_cboard_comm/rm2027_cboard_comm/trajectory_bridge_node.py`

节点：`trajectory_bridge_node`

功能： 1. 提供 `FollowJointTrajectory` Action Server。 2. 接收 MoveIt
六轴轨迹。 3. 检查关节名称、数量、轨迹点和数值。 4. 转换为标准关节顺序。
5. 按 `time_from_start` 调度轨迹点。 6. 将六轴位置及夹爪字段组织为
JSON。 7. 发布到 `/cboard/command`。

控制消息示例：

``` json
{
  "joints_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "gripper_field": 0.0
}
```

`joints_rad` 单位为 rad。当前测试的 `gripper_field` 为
`0.0`；实际含义和安全值仍需与固件、电控人员确认。

### 3.3 C板通信节点

源码：`cboard_comm_ws/src/rm2027_cboard_comm/rm2027_cboard_comm/cboard_comm_node.py`

节点：`cboard_comm_node`

功能： - 订阅 `/cboard/command`。 - 解析 JSON
并检查六轴数据及夹爪字段。 - 调用协议模块生成目标数据帧。 - 根据
`enable_tx` 决定是否真实写入串口。 - 读取串口返回数据并进行流式解析。 -
发布 `/cboard/feedback` 和 `/cboard/status`。

`enable_tx:=false` 为 DRY RUN；`enable_tx:=true` 允许真实串口发送。

### 3.4 协议模块

源码：`cboard_comm_ws/src/rm2027_cboard_comm/rm2027_cboard_comm/protocol.py`

根据当前代码检查，目标帧总长度为 35 字节：

  字段                              长度
  --------------------------- ----------
  帧头                            1 byte
  类型                            1 byte
  Payload长度                     1 byte
  序号                           2 bytes
  六轴及夹爪（7 × float32）     28 bytes
  CRC16                          2 bytes
  总计                          35 bytes

代码采用小端浮点数据编码、uint16 序号和 CRC16/CCITT-FALSE。最终以 STM32
固件协议定义为准，仍需交叉核对字段、字节序及 CRC 覆盖范围。

## 4. 验证记录

### 4.1 MoveIt 轨迹与桥接验证

RViz 执行 Plan 和 Execute 后，桥接节点曾接收到并转发一条轨迹：

-   轨迹点数量：17
-   轨迹总时长：约 1.5253 s
-   成功发布：17 个轨迹点
-   节点输出：`轨迹转发完成`

该次日志证明 MoveIt Action
请求到达桥接节点，桥接节点能够解析、调度并发布轨迹点。该次输出当时处于
DRY RUN，不能单独作为真实串口发送证据。

### 4.2 ROS 2 话题验证

控制话题：`/cboard/command`\
类型：`std_msgs/msg/String`

节点接口检查显示： - `trajectory_bridge_node` 发布 `/cboard/command`。 -
`cboard_comm_node` 订阅 `/cboard/command`。

反馈话题： - `/cboard/feedback` - `/cboard/status`

通信节点启动后，上述话题可以被发现和读取。

### 4.3 DRY RUN 验证

曾观察到：

  指标                     结果
  ---------------------- ------
  `commands_received`        18
  `commands_validated`       18
  `tx_frames`                 0
  `tx_bytes`                  0
  `last_tx_error`          null

说明控制消息能够完成解析和协议编码预览，但当时没有真实串口发送。

### 4.4 真实串口发送验证

通信节点使用 USB CDC 设备启动：

``` bash
PORT=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_206238494D4B-if00

ros2 run rm2027_cboard_comm cboard_comm \
  --ros-args \
  -p port:="$PORT" \
  -p baudrate:=115200 \
  -p enable_tx:=true
```

节点输出确认 USB CDC 连接成功，真实发送使能为 True。

之后读取 `/cboard/status`，得到一次状态采样：

  指标                         结果
  ---------------------- ----------
  `serial_open`                true
  `read_only`                 false
  `tx_enabled`                 true
  `tx_mode`                hardware
  `commands_received`            74
  `commands_validated`           74
  `tx_frames`                    74
  `tx_bytes`                   2590
  `valid_frames`              80706
  `frame_rate_hz`            100.01
  `rx_count_low16`               73
  `tx_sequence_next`             74
  `last_tx_error`              null
  `serial_error`               null

数据长度核算：`74 × 35 = 2590 bytes`。

最后一帧预览中的序号字节为 `49 00`，按小端 uint16 解读为
73；同一状态采样中的 `rx_count_low16` 也是
73。这是数值一致的证据，但在确认 STM32 固件中 `rx_count_low16`
的定义之前，不能将其直接认定为逐帧 ACK 或严格的一一对应确认。

状态中的 `commands_received`、`tx_frames`
等为节点运行期间累计计数，不能据此认定这 74 帧全部来自某一次 MoveIt
Execute。

## 5. 阶段结论

  验证项                           结果
  -------------------------------- ----------------
  MoveIt 生成规划轨迹              已验证
  MoveIt Action 到达桥接节点       已验证
  六轴轨迹解析与时间调度           已验证
  桥接节点发布 `/cboard/command`   已验证
  通信节点订阅控制话题             已验证
  JSON 解析及协议编码              已验证
  USB CDC 串口打开                 已验证
  真实串口发送计数增长             已验证
  串口反馈数据接收与帧解析         已观察到有效帧
  反馈字段的严格语义               待核实
  `link_alive` 持续在线状态        尚未通过
  机械臂电机实际运动               本阶段未测试

**阶段性结论：** MoveIt 轨迹经过 ROS 2 桥接和通信节点后，已经取得真实
USB CDC
串口发送的运行证据；上位机也能够接收并解析串口反馈数据。因此，MoveIt 到
C板的数据发送主链路取得阶段性验证成果。反馈确认机制和链路健康状态仍需继续验证。

## 6. 后续工作

1.  对照 STM32 固件确认 `rx_count_low16` 的定义。
2.  核对帧头、消息类型、Payload 长度、序号、字节序及 CRC 覆盖范围。
3.  在单次 MoveIt Execute 期间同步记录 Action
    轨迹点数、控制话题消息数、`tx_frames` 和
    C板接收计数，建立可复现的对应关系。
4.  排查 `link_alive: false` 的判定条件。
5.  进行连续发送和长时间运行测试，记录丢帧、错误帧、超时及串口异常。
6.  与电控确认 `gripper_field` 的定义和安全值。
7.  完成协议与通信安全核验后，再单独规划电机动作测试。

------------------------------------------------------------------------

**记录边界：**
本文只记录终端日志和代码检查能够支持的结论；串口发送成功不等于电机动作成功，尚未确认的反馈字段也不视为
ACK。
