import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================
    # 1. 获取功能包路径
    # =========================================

    package_share = get_package_share_directory(
        "panthera_ht_config"
    )

    config_dir = os.path.join(
        package_share,
        "config"
    )

    launch_dir = os.path.join(
        package_share,
        "launch"
    )

    # =========================================
    # 2. 构建专用C板MoveIt配置
    # =========================================

    moveit_config = (
        MoveItConfigsBuilder(
            "panthera_ht_ros_description",
            package_name="panthera_ht_config"
        )
        .trajectory_execution(
            file_path=os.path.join(
                config_dir,
                "moveit_controllers_cboard.yaml"
            )
        )
        .to_moveit_configs()
    )

    robot_description = moveit_config.robot_description

    # =========================================
    # 3. 静态虚拟关节TF
    # =========================================

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "static_virtual_joint_tfs.launch.py"
            )
        )
    )

    # =========================================
    # 4. Robot State Publisher
    # =========================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
    )

    # =========================================
    # 5. FakeSystem控制器
    # =========================================

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            os.path.join(
                config_dir,
                "ros2_controllers.yaml"
            ),
            {"use_sim_time": False},
        ],
    )

    # =========================================
    # 6. MoveGroup
    # 直接使用C板配置，不再调用旧启动文件
    # =========================================

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # =========================================
    # 7. RViz
    # =========================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                config_dir,
                "moveit.rviz"
            )
        ],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # =========================================
    # 8. 启动Joint State Broadcaster
    # =========================================

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "60",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # =========================================
    # 9. 启动FakeSystem Arm Controller
    # =========================================

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout", "60",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # =========================================
    # 10. 返回启动描述
    # =========================================

    return LaunchDescription([

        static_tf,

        robot_state_publisher,

        controller_manager,

        joint_state_spawner,

        delay_arm_controller,

        move_group,

        rviz,

    ])
