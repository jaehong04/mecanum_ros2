from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    autostart = LaunchConfiguration("autostart")

    params_file = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    map_yaml = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "maps",
        "drive_area.yaml",
    ])

    # --------------------------------------------------
    # Static Map Server
    # 외부카메라 + Global EKF가 localization을 담당하므로
    # AMCL은 실행하지 않는다.
    # --------------------------------------------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "yaml_filename": map_yaml,
                "use_sim_time": False,
            },
        ],
    )

    # map_server는 Nav2 navigation_launch.py의 lifecycle
    # manager 대상이 아니므로 별도 manager로 자동 활성화한다.
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

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

        map_server,
        map_lifecycle_manager,

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
            # standard robot velocity input /cmd_vel로 연결한다.
            SetRemap(
                src="cmd_vel_smoothed",
                dst="/cmd_vel",
            ),

            nav2_launch,
        ]),
    ])
