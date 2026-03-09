from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
