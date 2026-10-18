import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    panthera_config_path = FindPackageShare("panthera_ht_config")
    panthera_description_path = FindPackageShare("panthera_ht_ros_description")
    panthera_config_share = get_package_share_directory("panthera_ht_config")

    config_file = LaunchConfiguration("config_file")
    control_mode = LaunchConfiguration("control_mode")
    rviz = LaunchConfiguration("rviz")

    ros2_controllers_file = os.path.join(
        panthera_config_share,
        "config",
        "ros2_controllers_hardware_gripper.yaml",
    )
    moveit_controllers_file = os.path.join(
        panthera_config_share,
        "config",
        "moveit_controllers_hardware_gripper.yaml",
    )
    semantic_file = os.path.join(
        panthera_config_share,
        "config",
        "panthera_ht_ros_description_gripper.srdf",
    )

    default_config_file = PathJoinSubstitution([
        panthera_config_path,
        "robot_param",
        "Follower_absolute.yaml",
    ])

    urdf_file = PathJoinSubstitution([
        panthera_description_path,
        "urdf",
        "panthera_ht_ros_description_gripper.urdf.xacro",
    ])

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", urdf_file,
            " config_file:=", config_file,
            " control_mode:=", control_mode,
        ]),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder(
            "panthera_ht_ros_description",
            package_name="panthera_ht_config",
        )
        .robot_description_semantic(file_path=semantic_file)
        .trajectory_execution(file_path=moveit_controllers_file)
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_robot_description": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False,
    }

    moveit_params = [
        moveit_config.to_dict(),
        robot_description,
        move_group_configuration,
        {"use_sim_time": False},
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_file,
            robot_description,
            {"use_sim_time": False},
        ],
        output="screen",
        arguments=["--ros-args", "--param", "use_sim_time:=false"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        ),
    )

    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        ),
    )

    static_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
        output="log",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_params,
    )

    rviz_config_file = PathJoinSubstitution([
        panthera_config_path,
        "config",
        "moveit.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=moveit_params,
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to robot configuration YAML file",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="position_velocity",
            description="Control mode: position_velocity, pd_control, or full_control",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz",
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
        static_world_to_base,
        move_group_node,
        rviz_node,
    ])
