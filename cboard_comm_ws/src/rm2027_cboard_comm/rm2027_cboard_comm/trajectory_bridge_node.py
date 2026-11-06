#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RM2027 MoveIt -> C板轨迹桥接节点

当前阶段：D1 离散轨迹点转发

功能：
1. 接收 MoveIt 的 FollowJointTrajectory Action。
2. 验证六轴关节轨迹。
3. 检查关节名称、位置、速度、加速度和时间。
4. 将关节顺序统一为 joint1 ~ joint6。
5. 按照轨迹时间逐点调度。
6. 将轨迹点转换成 JSON。
7. 发布到 /cboard/command。
8. 由 CBoard Comm 节点完成协议编码。

安全模式：
DRY RUN

不直接操作串口，不发送真实硬件控制指令。
"""

import json
import math
import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String


# =========================================================
# 1. 机械臂关节定义
# =========================================================

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


# =========================================================
# 2. 轨迹桥接节点
# =========================================================

class CBoardTrajectoryBridge(Node):

    def __init__(self):

        super().__init__("trajectory_bridge_node")

        # -------------------------------------------------
        # 2.1 夹爪协议字段
        # -------------------------------------------------

        # 必须由启动参数显式设置。
        # NaN表示尚未配置，不允许发送控制消息。

        self.declare_parameter(
            "gripper_field",
            float("nan")
        )

        # -------------------------------------------------
        # 2.2 创建C板控制指令Publisher
        # -------------------------------------------------

        self.command_pub = self.create_publisher(
            String,
            "/cboard/command",
            10
        )

        # -------------------------------------------------
        # 2.3 创建MoveIt轨迹Action Server
        # -------------------------------------------------

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/cboard_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # -------------------------------------------------
        # 2.4 启动日志
        # -------------------------------------------------

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "RM2027轨迹桥接节点启动成功"
        )

        self.get_logger().info(
            "Action接口："
            "/cboard_controller/follow_joint_trajectory"
        )

        self.get_logger().info(
            "Publisher接口：/cboard/command"
        )

        self.get_logger().info(
            "当前阶段：D1离散轨迹点转发"
        )

        self.get_logger().info(
            "当前模式：DRY RUN"
        )

        self.get_logger().info(
            "========================================"
        )

    # =====================================================
    # 3. 时间转换
    # =====================================================

    @staticmethod
    def duration_to_seconds(duration):

        return (
            duration.sec
            + duration.nanosec * 1e-9
        )

    # =====================================================
    # 4. 获取夹爪字段
    # =====================================================

    def get_gripper_field(self):

        value = float(
            self.get_parameter("gripper_field").value
        )

        if not math.isfinite(value):

            raise ValueError(
                "gripper_field尚未配置，禁止发送控制指令"
            )

        return value

    # =====================================================
    # 5. 轨迹验证
    # =====================================================

    def validate_trajectory(self, trajectory):

        names = list(trajectory.joint_names)

        # -------------------------------------------------
        # 5.1 检查关节数量
        # -------------------------------------------------

        if len(names) != 6:

            raise ValueError(
                f"关节数量错误：{len(names)}，要求6个"
            )

        # -------------------------------------------------
        # 5.2 检查重复关节
        # -------------------------------------------------

        if len(set(names)) != 6:

            raise ValueError(
                "关节名称存在重复"
            )

        # -------------------------------------------------
        # 5.3 检查关节名称
        # -------------------------------------------------

        if set(names) != set(JOINT_NAMES):

            raise ValueError(
                f"关节名称不匹配：{names}"
            )

        # -------------------------------------------------
        # 5.4 计算关节顺序映射
        # -------------------------------------------------

        indices = [

            names.index(name)

            for name in JOINT_NAMES

        ]

        
        # -------------------------------------------------
        # 5.6 检查轨迹点
        # -------------------------------------------------

        points = trajectory.points

        if not points:

            raise ValueError(
                "轨迹没有任何轨迹点"
            )

        previous_time = -1.0

        for index, point in enumerate(points):

            # =============================================
            # 检查位置
            # =============================================

            if len(point.positions) != 6:

                raise ValueError(
                    f"第{index}个轨迹点位置数据长度错误"
                )

            # =============================================
            # 检查速度
            # =============================================

            if point.velocities:

                if len(point.velocities) != 6:

                    raise ValueError(
                        f"第{index}个轨迹点速度数据长度错误"
                    )

            # =============================================
            # 检查加速度
            # =============================================

            if point.accelerations:

                if len(point.accelerations) != 6:

                    raise ValueError(
                        f"第{index}个轨迹点加速度数据长度错误"
                    )

            # =============================================
            # 检查effort
            # =============================================

            if point.effort:

                if len(point.effort) != 6:

                    raise ValueError(
                        f"第{index}个轨迹点effort数据长度错误"
                    )

            # =============================================
            # 检查数值是否合法
            # =============================================

            fields = [

                ("positions", point.positions),

                ("velocities", point.velocities),

                ("accelerations", point.accelerations),

                ("effort", point.effort),

            ]

            for field_name, values in fields:

                for value in values:

                    if not math.isfinite(value):

                        raise ValueError(
                            f"第{index}个轨迹点的"
                            f"{field_name}包含NaN或Inf"
                        )

            # =============================================
            # 检查时间
            # =============================================

            current_time = self.duration_to_seconds(
                point.time_from_start
            )

            if current_time < 0:

                raise ValueError(
                    "轨迹时间不能为负数"
                )

            if current_time <= previous_time:

                raise ValueError(
                    f"第{index}个轨迹点时间没有严格递增"
                )

            previous_time = current_time

        # -------------------------------------------------
        # 5.7 返回验证结果
        # -------------------------------------------------

        return {

            "joint_names": names,

            "indices": indices,

            "points": points,

            "point_count": len(points),

            "duration": previous_time,

        }

    # =====================================================
    # 6. 发布JSON控制指令
    # =====================================================

    def publish_trajectory_point(
        self,
        positions,
        gripper_field
    ):

        if len(positions) != 6:

            raise ValueError(
                "控制指令必须包含六轴位置"
            )

        # -------------------------------------------------
        # 构造JSON
        # -------------------------------------------------

        command = {

            "joints_rad": [

                float(value)

                for value in positions

            ],

            "gripper_field": float(gripper_field),

        }

        # -------------------------------------------------
        # 检查数值
        # -------------------------------------------------

        for value in command["joints_rad"]:

            if not math.isfinite(value):

                raise ValueError(
                    "关节角度包含非法数值"
                )

        if not math.isfinite(command["gripper_field"]):

            raise ValueError(
                "夹爪字段包含非法数值"
            )

        # -------------------------------------------------
        # JSON序列化
        # -------------------------------------------------

        json_data = json.dumps(
            command,
            ensure_ascii=False,
            allow_nan=False
        )

        # -------------------------------------------------
        # 发布ROS话题
        # -------------------------------------------------

        msg = String()

        msg.data = json_data

        self.command_pub.publish(msg)

        return json_data

    # =====================================================
    # 7. Action目标验证回调
    # =====================================================

    def goal_callback(self, goal_request):

        try:

            # 验证轨迹

            info = self.validate_trajectory(
                goal_request.trajectory
            )

            # 验证夹爪字段

            self.get_gripper_field()

            self.get_logger().info(
                "收到轨迹请求，"
                f"轨迹点数量：{info['point_count']}"
            )

            return GoalResponse.ACCEPT

        except Exception as exc:

            self.get_logger().warning(
                f"轨迹请求被拒绝：{exc}"
            )

            return GoalResponse.REJECT

    # =====================================================
    # 8. Action取消回调
    # =====================================================

    def cancel_callback(self, goal_handle):

        self.get_logger().info(
            "收到轨迹取消请求"
        )

        return CancelResponse.ACCEPT

    # =====================================================
    # 9. Action执行回调
    # =====================================================

    def execute_callback(self, goal_handle):

        result = FollowJointTrajectory.Result()

        try:

            # -------------------------------------------------
            # 9.1 获取轨迹
            # -------------------------------------------------

            trajectory = goal_handle.request.trajectory

            info = self.validate_trajectory(
                trajectory
            )

            gripper_field = self.get_gripper_field()

            indices = info["indices"]

            points = info["points"]

            # -------------------------------------------------
            # 9.2 输出轨迹信息
            # -------------------------------------------------

            self.get_logger().info(
                "========== 开始轨迹转发 =========="
            )

            self.get_logger().info(
                f"轨迹点数量：{info['point_count']}"
            )

            self.get_logger().info(
                f"轨迹总时长：{info['duration']:.4f}s"
            )

            # -------------------------------------------------
            # 9.3 使用单调时钟
            #
            # 不使用asyncio.get_running_loop()
            # 不使用asyncio.sleep()
            #
            # ROS2 Humble的rclpy执行器不保证
            # 存在Python asyncio event loop。
            # -------------------------------------------------

            start_time = time.monotonic()

            published_count = 0

            # -------------------------------------------------
            # 9.4 按照轨迹时间逐点转发
            # -------------------------------------------------

            for i, point in enumerate(points):

                target_time = self.duration_to_seconds(
                    point.time_from_start
                )

                # =============================================
                # 等待到达目标时间
                # =============================================

                while True:

                    if goal_handle.is_cancel_requested:

                        goal_handle.canceled()

                        result.error_code = (
                            FollowJointTrajectory.Result.INVALID_GOAL
                        )

                        result.error_string = (
                            "轨迹已取消"
                        )

                        self.get_logger().warning(
                            "轨迹转发已取消"
                        )

                        return result

                    elapsed = (
                        time.monotonic() - start_time
                    )

                    remaining = (
                        target_time - elapsed
                    )

                    if remaining <= 0:

                        break

                    time.sleep(
                        min(remaining, 0.02)
                    )

                # =============================================
                # 转换成标准六轴顺序
                # =============================================

                positions = [

                    float(point.positions[j])

                    for j in indices

                ]

                # =============================================
                # 发布JSON控制指令
                # =============================================

                json_data = self.publish_trajectory_point(
                    positions,
                    gripper_field
                )

                published_count += 1

                # =============================================
                # 输出日志
                # =============================================

                self.get_logger().info(

                    f"已转发轨迹点 "
                    f"{published_count}/{len(points)}，"

                    f"time={target_time:.4f}s，"

                    f"positions={positions}"

                )

                self.get_logger().debug(
                    f"JSON：{json_data}"
                )

            # -------------------------------------------------
            # 9.5 轨迹转发完成
            # -------------------------------------------------

            self.get_logger().info(
                "========== 轨迹转发完成 =========="
            )

            self.get_logger().info(
                f"成功发布轨迹点：{published_count}"
            )

            self.get_logger().info(
                "数据已提交至/cboard/command"
            )

            self.get_logger().info(
                "当前仍为DRY RUN模式"
            )

            # -------------------------------------------------
            # 9.6 返回成功
            # -------------------------------------------------

            result.error_code = (
                FollowJointTrajectory.Result.SUCCESSFUL
            )

            result.error_string = (

                "DRY RUN: all trajectory points forwarded "
                "to /cboard/command; "
                "no hardware command sent."

            )

            goal_handle.succeed()

            return result

        except Exception as exc:

            # -------------------------------------------------
            # 9.7 异常处理
            # -------------------------------------------------

            self.get_logger().error(
                f"轨迹转发失败：{exc}"
            )

            result.error_code = (
                FollowJointTrajectory.Result.INVALID_GOAL
            )

            result.error_string = str(exc)

            goal_handle.abort()

            return result


# =========================================================
# 10. ROS2节点入口
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = CBoardTrajectoryBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "正在关闭轨迹桥接节点"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# =========================================================
# 11. Python入口
# =========================================================

if __name__ == "__main__":

    main()