from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros_extras.actions import LoadMoveitConfig, LoadYaml, GenerateMoveitLaunch
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur5e_workcell", package_name="ur5e_cell_moveit_config"
    ).to_moveit_configs()

    servo_node = Node(
        name="servo_node",
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ur5e_cell_moveit_servo"), "config", "ur_servo.yaml"]
            ),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            servo_node,
        ]
    )
