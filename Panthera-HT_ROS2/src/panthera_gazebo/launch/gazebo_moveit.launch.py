from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    gripper_gazebo_path = FindPackageShare('panthera_gazebo')
    panthera_config_path = FindPackageShare('panthera_ht_config')

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gripper_gazebo_path,
                'launch',
                'gazebo_classic.launch.py'
            ])
        ])
    )

    # Include static virtual joint TFs (publishes world->base_link transform)
    static_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                panthera_config_path,
                'launch',
                'static_virtual_joint_tfs.launch.py'
            ])
        ])
    )

    # Include MoveIt move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                panthera_config_path,
                'launch',
                'move_group_sim.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include MoveIt RViz (delayed 15s to wait for move_group to publish robot_description_semantic)
    moveit_rviz_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        panthera_config_path,
                        'launch',
                        'moveit_rviz_sim.launch.py'
                    ])
                ]),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        static_tfs_launch,
        move_group_launch,
        moveit_rviz_launch,
    ])
