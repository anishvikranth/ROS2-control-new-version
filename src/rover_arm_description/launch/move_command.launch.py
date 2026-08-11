from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "rover_arm",
            package_name="rover_arm_moveit_config",
        )
        .to_moveit_configs()
    )

    moveit_goal_node = Node(
        package="rover_arm_description",
        executable="moveit_goal_node",
        name="moveit_goal_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription([
        moveit_goal_node,
    ])