from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    autostart = LaunchConfiguration("autostart")

    params_file = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "autostart",
            default_value="false",
            description="Activate Nav2 lifecycle nodes automatically",
        ),

        GroupAction([
            # Humble navigation_launch.py:
            #
            # controller_server:
            #   cmd_vel -> cmd_vel_nav
            #
            # velocity_smoother:
            #   cmd_vel_smoothed -> cmd_vel
            #
            # 여기서는 최종 smoother 출력만
            # mecanum controller 입력으로 연결한다.
            SetRemap(
                src="cmd_vel_smoothed",
                dst="/mecanum_drive_controller/reference_unstamped",
            ),

            nav2_launch,
        ]),
    ])
