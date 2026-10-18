import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    panthera_config = FindPackageShare('panthera_ht_config')

    # Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            panthera_config, 'robot_param', 'Follower_absolute.yaml'
        ]),
        description='Path to robot configuration YAML file'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position_velocity',
        description='Control mode'
    )

    serial_no_arg = DeclareLaunchArgument(
        'camera_serial_no',
        default_value='',
        description='RealSense camera serial number'
    )

    # 1. Robot hardware + MoveIt (without RViz, we launch our own)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                panthera_config, 'launch', 'hardware_moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'control_mode': LaunchConfiguration('control_mode'),
            'rviz': 'false',
        }.items()
    )

    # 2. RealSense D435/D435i camera (eye-to-hand, fixed in environment)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'enable_infra': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'rgb_camera.color_profile': '640x480x30',
            'camera_name': 'camera',
            'camera_namespace': '',
            'serial_no': LaunchConfiguration('camera_serial_no'),
            'publish_tf': True,
            'tf_publish_rate': 0.0,
        }]
    )

    # 3. Static TF: camera_mount_link -> camera_link (identity since camera is on mount)
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_mount_link', 'camera_link']
    )

    # 4. RViz2 with calibration display
    rviz_config = PathJoinSubstitution([
        panthera_config, 'config', 'calibration.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        config_file_arg,
        control_mode_arg,
        serial_no_arg,
        hardware_launch,
        realsense_node,
        camera_link_tf,
        rviz_node,
    ])
