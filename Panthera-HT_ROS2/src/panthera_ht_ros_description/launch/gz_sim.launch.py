#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # 获取包的路径
    pkg_name = 'panthera_ht_ros_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 设置 Gazebo 资源路径环境变量
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )

    # XACRO 文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'panthera_ht_ros_description.xacro')

    # 使用 xacro 处理文件，生成 URDF（保持 package:// 用于 ROS）
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 为 Gazebo Sim 创建副本，将 package:// 替换为 model://
    robot_desc_gazebo = robot_desc.replace('package://', 'model://')

    # 声明 launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 启动 Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # robot_state_publisher 节点（使用原始 URDF，保持 package://）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # 在 Gazebo 中生成机器人模型（使用转换后的 URDF）
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-name', 'Panthera_HT',
            '-string', robot_desc_gazebo,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ]
    )

    # joint_state_publisher 节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gazebo 到 ROS 的桥接
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_publisher_node,
        gz_ros_bridge
    ])
