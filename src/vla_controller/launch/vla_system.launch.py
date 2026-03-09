import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def _venv_pythonpath_env() -> dict:
    venv = os.environ.get("VIRTUAL_ENV")
    if not venv:
        return {}

    site_packages = os.path.join(
        venv,
        "lib",
        f"python{sys.version_info.major}.{sys.version_info.minor}",
        "site-packages",
    )
    if not os.path.isdir(site_packages):
        return {}

    current_pythonpath = os.environ.get("PYTHONPATH", "")
    if current_pythonpath:
        return {"PYTHONPATH": f"{site_packages}:{current_pythonpath}"}
    return {"PYTHONPATH": site_packages}


def generate_launch_description() -> LaunchDescription:
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=[FindPackageShare("vla_controller"), "/config/vla_params.yaml"],
        description="Path to bridge parameter yaml file",
    )
    openpi_host_arg = DeclareLaunchArgument(
        "openpi_host", default_value="localhost", description="OpenPI server host"
    )
    openpi_port_arg = DeclareLaunchArgument(
        "openpi_port", default_value="8000", description="OpenPI server port"
    )
    task_arg = DeclareLaunchArgument(
        "task", default_value="pick up the block", description="Task instruction prompt"
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="If true, infer actions but do not send trajectories",
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5e", package_name="ur5e_isaac_moveit_config")
        .joint_limits("config/joint_limits.yaml")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur5e_isaac_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": True},
        ],
        output="screen",
    )
    delay_controller_manager = TimerAction(period=3.0, actions=[controller_manager_node])

    spawn_controllers_launch = generate_spawn_controllers_launch(moveit_config)
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[spawn_controllers_launch],
        )
    )

    vla_node = Node(
        package="vla_controller",
        executable="vla_controller_node",
        output="screen",
        additional_env=_venv_pythonpath_env(),
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "openpi_host": LaunchConfiguration("openpi_host"),
                "openpi_port": LaunchConfiguration("openpi_port"),
                "task_instruction": LaunchConfiguration("task"),
                "dry_run": LaunchConfiguration("dry_run"),
            },
        ],
    )
    delay_vla_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[TimerAction(period=4.0, actions=[vla_node])],
        )
    )

    return LaunchDescription(
        [
            params_file_arg,
            openpi_host_arg,
            openpi_port_arg,
            task_arg,
            dry_run_arg,
            robot_state_publisher,
            delay_controller_manager,
            delay_spawners,
            delay_vla_node,
        ]
    )
