from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('panthera_ht_config')
    bringup_pkg = get_package_share_directory('panthera_bringup')

    # URDF file path - use the one from panthera_ht_config that includes ros2_control
    urdf_file = os.path.join(moveit_config_pkg, 'config', 'panthera_ht_ros_description.urdf.xacro')
    initial_positions_file = os.path.join(moveit_config_pkg, 'config', 'initial_positions.yaml')

    # Load robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file,
        ' initial_positions_file:=', initial_positions_file
    ])

    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    controller_config = os.path.join(bringup_pkg, 'config', 'ros2_controllers.yaml')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controller_config]
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # Gripper Controller Spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # MoveIt move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_launch,
        rviz_node
    ])
