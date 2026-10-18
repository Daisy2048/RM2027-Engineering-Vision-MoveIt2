from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='',
                              description='Path to robot config YAML. Empty = use default Follower.yaml'),
        DeclareLaunchArgument('status_publish_rate', default_value='20.0',
                              description='Status publish rate in Hz'),
        DeclareLaunchArgument('max_velocity', default_value='0.5',
                              description='Max joint velocity in rad/s'),

        Node(
            package='panthera_arm_control',
            executable='arm_control_node',
            name='arm_control_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'status_publish_rate': LaunchConfiguration('status_publish_rate'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'max_torque': [21.0, 36.0, 36.0, 21.0, 10.0, 10.0],
            }],
        ),
    ])
