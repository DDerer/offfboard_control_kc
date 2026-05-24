from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("target_z", default_value="-1.0"),
            DeclareLaunchArgument("next_target_x", default_value="0.0"),
            DeclareLaunchArgument("next_target_y", default_value="1.5"),
            DeclareLaunchArgument("next_target_z", default_value="-1.0"),
            DeclareLaunchArgument("hover_duration", default_value="5.0"),
            Node(
                package="offboard_control_cpp",
                executable="simple_takeoff_node",
                name="simple_takeoff",
                output="screen",
                parameters=[
                    {
                        "auto_start": LaunchConfiguration("auto_start"),
                        "auto_arm": LaunchConfiguration("auto_arm"),
                        "target_z": LaunchConfiguration("target_z"),
                        "next_target_x": LaunchConfiguration("next_target_x"),
                        "next_target_y": LaunchConfiguration("next_target_y"),
                        "next_target_z": LaunchConfiguration("next_target_z"),
                        "hover_duration": LaunchConfiguration("hover_duration"),
                    }
                ],
            ),
        ]
    )
