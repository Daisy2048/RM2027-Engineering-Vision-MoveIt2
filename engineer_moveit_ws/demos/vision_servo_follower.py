#!/usr/bin/env python3
"""Continuous valid-PnP-frame following with MoveIt Servo on FakeSystem only.

NOTE: This is a DEMO mapping from uncalibrated prerecorded-video PnP to a small
simulated workspace. Orientation is RECORDED, but not yet actuated. Do not use
on a real robot until camera/tool calibration, collision and control tests.
"""

import csv
import math
import os
import statistics
import time
from collections import deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


class VisionServoFollower(Node):
    def __init__(self):
        super().__init__('vision_servo_follower')
        self.initial_frames = []
        self.origin = None
        self.xyz_buffer = deque(maxlen=7)
        self.filtered_target = None
        self.last_vision_mono = None
        self.frame_count = 0
        self.cmd_count = 0
        self.last_state_print = 0.0
        self.robot_xyz = None
        self.servo_publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )
        self.target_publisher = self.create_publisher(
            PoseStamped, '/vision/servo_target_pose_base', 10
        )
        self.create_subscription(
            PoseStamped, '/vision/reference_pose_base', self.on_pose, 10
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.02, self.servo_tick)  # 50 Hz closed-loop servo input

        folder = os.path.expanduser('~/rm2027/engineer_moveit_ws/logs/servo_demo')
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, 'frames_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
        self.file = open(path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['frame_index', 'source_sec', 'source_nanosec',
                              'x_base', 'y_base', 'z_base', 'qx', 'qy', 'qz', 'qw',
                              'target_x', 'target_y', 'target_z'])
        self.get_logger().info('BIT逐帧记录: ' + path)
        self.get_logger().info('模式: 连续视觉跟随 FakeSystem; 目标输入30Hz左右，Servo速度指令50Hz')
        self.get_logger().warn('仿真相机外参和位姿映射未经真实标定，仅用于软件演示！')

    def on_pose(self, msg):
        if msg.header.frame_id != 'base_link':
            return
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(x) for x in values):
            return
        if abs(sum(v * v for v in values[3:]) - 1.0) > 0.025:
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return

        xyz = (p.x, p.y, p.z)
        self.frame_count += 1
        self.last_vision_mono = time.monotonic()
        self.xyz_buffer.append(xyz)
        if self.origin is None:
            self.initial_frames.append(xyz)
            if len(self.initial_frames) >= 15:
                self.origin = tuple(statistics.median(row[i] for row in self.initial_frames)
                                    for i in range(3))
                self.get_logger().info('视觉参考位姿 X/Y/Z: ' + str(self.origin))

        if self.origin is None:
            return
        # Per-frame median removes one-off PnP outliers, followed by EMA smoothing.
        median_xyz = tuple(statistics.median(row[i] for row in self.xyz_buffer)
                           for i in range(3))
        dx, dy, dz = (median_xyz[i] - self.origin[i] for i in range(3))

        # LOCAL DEMO mapping, not a 6DoF calibrated grasp target:
        # XY: small bounded relative movement; Z: larger bounded relative movement.
        raw = (
            min(0.194, max(0.174, 0.184 + 0.06 * dx)),
            min(0.082, max(0.062, 0.072 + 0.06 * dy)),
            min(0.340, max(0.289, 0.315 + 0.30 * dz)),
        )
        if self.filtered_target is None:
            self.filtered_target = list(raw)
        else:
            for i in range(3):
                self.filtered_target[i] += 0.13 * (raw[i] - self.filtered_target[i])

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'base_link'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.filtered_target
        goal.pose.orientation.w = 1.0  # orientation tracking disabled until calibrated
        self.target_publisher.publish(goal)

        # Save every VALID pose message, including source quaternion, to disk.
        self.writer.writerow([
            self.frame_count, msg.header.stamp.sec, msg.header.stamp.nanosec,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            *self.filtered_target
        ])
        if self.frame_count % 30 == 0:
            self.file.flush()

    def servo_tick(self):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'

        fresh = (self.filtered_target is not None
                 and self.last_vision_mono is not None
                 and time.monotonic() - self.last_vision_mono < 0.5)
        try:
            tf = self.tf_buffer.lookup_transform('base_link', 'bat_center', Time())
            cur = tf.transform.translation
            self.robot_xyz = (cur.x, cur.y, cur.z)
        except TransformException:
            self.robot_xyz = None

        if fresh and self.robot_xyz is not None:
            # Additional guard against an unexpected robot pose.
            x, y, z = self.robot_xyz
            if 0.14 < x < 0.23 and 0.03 < y < 0.13 and 0.25 < z < 0.38:
                errors = [self.filtered_target[i] - self.robot_xyz[i]
                          for i in range(3)]
                # Cartesian proportional control with per-axis speed limit.
                vx, vy, vz = [max(-0.012, min(0.012, 0.8 * e))
                              if abs(e) > 0.002 else 0.0 for e in errors]
                command.twist.linear.x = vx
                command.twist.linear.y = vy
                command.twist.linear.z = vz

        # If video frames/TF disappear, publish zero twist to stop the simulated arm.
        self.servo_publisher.publish(command)
        self.cmd_count += 1
        now = time.monotonic()
        if now - self.last_state_print > 2.0:
            self.last_state_print = now
            if self.filtered_target is None:
                self.get_logger().info('等待 BIT 有效帧和基准位姿...')
            else:
                self.get_logger().info(
                    'valid_frames=%d  target=(%.3f,%.3f,%.3f)  actual=%s  servo_v=(%.3f,%.3f,%.3f)  fresh=%s' % (
                        self.frame_count, *self.filtered_target,
                        tuple(round(v, 3) for v in self.robot_xyz) if self.robot_xyz else 'NO_TF',
                        command.twist.linear.x, command.twist.linear.y, command.twist.linear.z,
                        fresh))

    def close(self):
        self.file.flush()
        self.file.close()


def main():
    rclpy.init()
    node = VisionServoFollower()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
