from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # 1. Load your MoveIt Configs
    moveit_config = (
        MoveItConfigsBuilder("ur5e", package_name="ur5e_isaac_moveit_config")
        .joint_limits("config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 2. Robot State Publisher (The "Blueprint")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # 3. MoveGroup (The "Brain")
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # 4. RViz (The "Window")
    # This launches the actual GUI with the Motion Planning plugin
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
        run_move_group_node,
        rviz_node
    ])