import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    pick_place_demo = Node(
        package="ur5e_cell_advanced_pick_n_place",
        executable="pick_place_task_demo",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("ur5e_cell_advanced_pick_n_place"),
                "config",
                "pick_place_task_demo.yaml",
            ),
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
