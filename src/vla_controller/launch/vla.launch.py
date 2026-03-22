import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _venv_pythonpath_env() -> dict:
    # Inject openpi_client into the node's PYTHONPATH using a Python 3.12 venv
    # that lives in this repo and contains only openpi_client + its pure-Python deps.
    # Using a 3.12 venv (matching the system ROS2 Python) avoids ABI conflicts with
    # compiled extensions like numpy and cv2 that live in the openpi venv (Python 3.11).
    #
    # One-time setup:
    #   python3 -m venv .venv
    #   .venv/bin/pip install openpi-client
    venv_dir = os.path.expanduser("~/Development/VLA_Arm_Controller/.venv")
    site_packages = os.path.join(
        venv_dir,
        "lib",
        f"python{sys.version_info.major}.{sys.version_info.minor}",
        "site-packages",
    )

    if not os.path.isdir(site_packages):
        print(
            "[vla_controller] WARNING: local .venv not found. openpi_client will not be available. "
            "Run: python3 -m venv .venv && .venv/bin/pip install openpi-client"
        )
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

    node = Node(
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

    return LaunchDescription(
        [params_file_arg, openpi_host_arg, openpi_port_arg, task_arg, dry_run_arg, node]
    )
