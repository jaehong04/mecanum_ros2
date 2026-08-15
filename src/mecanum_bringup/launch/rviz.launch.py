from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "rviz",
        "mecanum.rviz"
    ])

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    ])
