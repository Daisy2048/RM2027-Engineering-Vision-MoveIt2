#!/usr/bin/env python3
"""将一次实时视觉候选位姿发送到 MoveIt 规划桥（默认仅规划）。

注意：identity 姿态仅供软件链路调试，不能用于真实装配。
"""
import argparse
import copy
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description="发布一次实时视觉候选接近点")
    parser.add_argument("--side", choices=["plus", "minus"], default="plus")
    parser.add_argument("--orientation", choices=["identity", "original"], default="identity")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("rm2027_candidate_once")
    src = f"/vision/pillar_approach_{args.side}_base"
    target = "/vision/target_pose_base"
    received = []

    def callback(msg):
        if msg.header.frame_id == "base_link":
            received.append(msg)

    subscription = node.create_subscription(PoseStamped, src, callback, 10)
    publisher = node.create_publisher(PoseStamped, target, 10)
    deadline = time.monotonic() + args.timeout
    print(f"等待 {src} 的 base_link 位姿 …")
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not received:
        print("ERROR: 等待候选点超时；请检查视觉管线和 TF。")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    goal = copy.deepcopy(received[-1])
    if args.orientation == "identity":
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
    goal.header.stamp = node.get_clock().now().to_msg()
    deadline = time.monotonic() + args.timeout
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print(f"ERROR: 没有节点订阅 {target}，请检查 planner_bridge。")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    p = goal.pose.position
    print(f"发送候选目标: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m")
    print(f"姿态模式: {args.orientation}; 这是软件联调目标，不代表正确工具姿态。")
    publisher.publish(goal)
    time.sleep(0.7)
    print("已发布一次目标。是否规划成功请查看 planner_bridge 日志；此脚本不负责执行。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
