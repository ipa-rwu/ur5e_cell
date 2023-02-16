from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    ur5e_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_bringup'),
                'ur5e.launch.py'
            ])
        ]),

        launch_arguments={
            'robot_ip': '192.168.56.2',
            'ur_type': 'ur5e',
            'launch_rviz': False
        }.items()
    )
    
    ur5e_moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'ur_moveit.launch.py'
            ])
        ]),

        launch_arguments={
            'ur_type': 'ur5e',
            'servo': True
        }.items()
    )
    
    servo_keyboard_input_node = Node(
        package="moveit_servo_demo",
        executable="servo_keyboard_input",
        output="screen",
    )

    
    return LaunchDescription([
        ur5e_driver_launch,
        ur5e_moveit_config_launch,
        servo_keyboard_input_node
    ])