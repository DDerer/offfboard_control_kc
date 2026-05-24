from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            Node(
                package="offboard_control_cpp",
                executable="offboard_node",
                name="offboard_control",
                output="screen",
                parameters=[
                    {
                        "auto_start": LaunchConfiguration("auto_start"),
                        "auto_arm": LaunchConfiguration("auto_arm"),
                    }
                ],
            ),
        ]
    )
