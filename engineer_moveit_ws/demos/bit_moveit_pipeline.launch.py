from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # 1. BIT 视频回放
        Node(
            package='virtual_camera_pkg',
            executable='virtual_camera_node',
            output='screen'
        ),

        # 2. YOLO-Pose 检测
        Node(
            package='pose_detect_pkg',
            executable='detect_node',
            output='screen'
        ),

        # 3. PnP 位姿计算
        Node(
            package='pnp_solver_pkg',
            executable='pnp_node',
            output='screen'
        ),

        # 4. 模拟相机外参（仅用于视频演示）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '-0.25',
                '--y', '-0.05',
                '--z', '0.80',
                '--roll', '-1.57079632679',
                '--pitch', '0',
                '--yaw', '-1.57079632679',
            ],
            output='screen'
        ),

        # 5. PnP -> base_link
        Node(
            package='engineer_vision_tf_bridge',
            executable='tf_bridge',
            output='screen'
        ),

        # 6. 预览候选目标
        Node(
            package='engineer_vision_tf_bridge',
            executable='target_pose_preview',
            parameters=[
                {'offset_xyz_m': [0.0, 0.0, 0.04]}
            ],
            output='screen'
        ),

    ])
