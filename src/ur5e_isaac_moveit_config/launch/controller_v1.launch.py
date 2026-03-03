import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch, generate_move_group_launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # 1. Load the unified MoveIt config
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

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # 3. Controller Manager (The crucial bridge to Isaac Sim)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": True}
        ],
        output="screen",
    )

    # 4. Spawners (Using your helper snippet)
    spawn_controllers_launch = generate_spawn_controllers_launch(moveit_config)

    # Delay spawners until controller manager is ready
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[spawn_controllers_launch],
        )
    )

    # 5. MoveGroup (Using your new helper snippet)
    move_group_launch = generate_move_group_launch(moveit_config)

    # 6. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager_node,
        delay_spawners,
        move_group_launch,
        rviz_node
    ])