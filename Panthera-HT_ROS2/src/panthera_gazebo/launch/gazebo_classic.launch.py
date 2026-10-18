import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gripper_gazebo_path = FindPackageShare('panthera_gazebo')
    panthera_description_path = FindPackageShare('panthera_ht_ros_description')

    controllers_file = PathJoinSubstitution([
        gripper_gazebo_path, 'config', 'ros2_controllers.yaml'
    ])

    urdf_file = PathJoinSubstitution([
        gripper_gazebo_path, 'urdf', 'Panthera-HT_gazebo_classic.urdf.xacro'
    ])

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' ros2_control_params:=', controllers_file
        ]),
        value_type=str
    )

    # Set GAZEBO_MODEL_PATH so Gazebo Classic can find meshes
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            panthera_description_path, ':',
            PathJoinSubstitution([panthera_description_path, '..']), ':',
            os.environ.get('GAZEBO_MODEL_PATH', '')
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    world_file = PathJoinSubstitution([
        gripper_gazebo_path, 'worlds', 'empty.world'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                # ROS Humble rclpy is built for system Python 3.10. Do not
                # let an active Conda environment select Python 3.13 here.
                prefix=['/usr/bin/python3'],
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'panthera_with_gripper',
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager-timeout', '60',
                           '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    arm_controller_spawner = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller',
                           '--controller-manager-timeout', '60',
                           '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    gripper_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller',
                           '--controller-manager-timeout', '60',
                           '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo_model_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
