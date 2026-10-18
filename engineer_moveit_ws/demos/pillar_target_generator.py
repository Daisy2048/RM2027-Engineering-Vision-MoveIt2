import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def cross(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    )


def rotate(q, v):
    # 使用单位四元数旋转局部坐标
    u = (q.x, q.y, q.z)
    t = cross(u, v)
    t = tuple(2.0 * x for x in t)
    c = cross(u, t)

    return tuple(
        v[i] + q.w*t[i] + c[i]
        for i in range(3)
    )


class PillarTargetGenerator(Node):

    def __init__(self):
        super().__init__('pillar_target_generator')

        self.create_subscription(
            PoseStamped,
            '/vision/reference_pose_base',
            self.on_pose,
            10
        )

        self.face_pub = self.create_publisher(
            PoseStamped,
            '/vision/pillar_face_center_base',
            10
        )

        self.plus_pub = self.create_publisher(
            PoseStamped,
            '/vision/pillar_approach_plus_base',
            10
        )

        self.minus_pub = self.create_publisher(
            PoseStamped,
            '/vision/pillar_approach_minus_base',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/vision/pillar_markers',
            10
        )

        self.last_log = 0.0

        self.get_logger().info(
            'Pillar target generator started.'
        )

    def on_pose(self, msg):

        if msg.header.frame_id != 'base_link':
            return

        p = msg.pose.position
        q = msg.pose.orientation

        values = [
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ]

        if not all(math.isfinite(v) for v in values):
            return

        norm = math.sqrt(
            q.x*q.x + q.y*q.y +
            q.z*q.z + q.w*q.w
        )

        if abs(norm - 1.0) > 0.01:
            return

        origin = (p.x, p.y, p.z)

        def to_base(local):
            rotated = rotate(q, local)

            return tuple(
                origin[i] + rotated[i]
                for i in range(3)
            )

        # 来自 BIT world_points.yaml
        face = to_base((0.0999, 0.0, 0.0))

        # 装配面两侧的候选接近点
        plus = to_base((0.1499, 0.0, 0.0))
        minus = to_base((0.0499, 0.0, 0.0))

        def publish_pose(publisher, position):
            out = PoseStamped()
            out.header = msg.header

            out.pose.position.x = position[0]
            out.pose.position.y = position[1]
            out.pose.position.z = position[2]
            out.pose.orientation = q

            publisher.publish(out)

        publish_pose(self.face_pub, face)
        publish_pose(self.plus_pub, plus)
        publish_pose(self.minus_pub, minus)

        markers = MarkerArray()

        points = [
            (face, 1.0, 0.1, 0.1),
            (plus, 0.1, 0.3, 1.0),
            (minus, 0.1, 1.0, 0.2)
        ]

        for i, (position, r, g, b) in enumerate(points):

            marker = Marker()

            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'pillar_targets'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.025
            marker.scale.y = 0.025
            marker.scale.z = 0.025

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            markers.markers.append(marker)

        self.marker_pub.publish(markers)

        now = time.monotonic()

        if now - self.last_log >= 1.0:
            self.last_log = now

            self.get_logger().info(
                f'Pillar face: x={face[0]:.3f} '
                f'y={face[1]:.3f} '
                f'z={face[2]:.3f}'
                )

def main():
    rclpy.init()
    node = PillarTargetGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
