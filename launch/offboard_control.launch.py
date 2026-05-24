from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("warmup_setpoints", default_value="10"),
            DeclareLaunchArgument("position_tolerance", default_value="0.1"),
            DeclareLaunchArgument(
                "local_position_topic",
                default_value="/fmu/out/vehicle_local_position_v1",
            ),
            Node(
                package="offboard_control_cpp",
                executable="offboard_node",
                name="offboard_control",
                output="screen",
                parameters=[
                    {
                        "auto_start": LaunchConfiguration("auto_start"),
                        "auto_arm": LaunchConfiguration("auto_arm"),
                        "warmup_setpoints": LaunchConfiguration("warmup_setpoints"),
                        "position_tolerance": LaunchConfiguration("position_tolerance"),
                        "local_position_topic": LaunchConfiguration("local_position_topic"),
                    }
                ],
            ),
        ]
    )
