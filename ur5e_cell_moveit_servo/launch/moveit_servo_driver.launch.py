from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ur_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_fake_hardware": "true",
            "initial_joint_controller": "joint_trajectory_controller",
            "activate_joint_controller": "false",
            "robot_ip": "xxx.yyy.zzz.vvv",
            "description_package": "ur5e_cell_description",
            "description_file": "workcell.urdf.xacro",
            "launch_rviz": "true",
            "controllers_file": PathJoinSubstitution([
                FindPackageShare("ur5e_cell_moveit_servo"),
                "config",
                "ros2_controllers.yaml"
            ])
        }.items(),
    )
    
    forward_velocity_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "-c", "/controller_manager", "--inactive"],
    )

    ld = LaunchDescription([ur_control_launch, forward_velocity_controller_spawner_stopped])

    return ld
