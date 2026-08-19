from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ==================================================
    # Robot Bringup
    # ==================================================
    # ros2_control
    # IMU
    # Local EKF
    # Global EKF
    # cmd_vel_bridge
    # ==================================================
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mecanum_bringup"),
                "launch",
                "mecanum_bringup.launch.py",
            ])
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_imu": "true",
        }.items(),
    )

    # ==================================================
    # Nav2 + Map Server
    # ==================================================
    # mecanum_nav2.launch.py 내부에서:
    # - map_server 자동 실행
    # - map lifecycle manager 자동 실행
    # - Nav2 자동 활성화
    # ==================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mecanum_bringup"),
                "launch",
                "mecanum_nav2.launch.py",
            ])
        ),
        launch_arguments={
            "autostart": "true",
        }.items(),
    )

    # Robot Bringup이 먼저 올라온 뒤 Nav2 시작
    delayed_nav2 = TimerAction(
        period=6.0,
        actions=[
            nav2_launch,
        ],
    )

    return LaunchDescription([
        bringup_launch,
        delayed_nav2,
    ])
