from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    use_rviz = LaunchConfiguration("use_rviz")
    use_imu = LaunchConfiguration("use_imu")
    imu_address = LaunchConfiguration("imu_address")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("mecanum_bringup"),
                "urdf",
                "mecanum_robot.urdf.xacro",
            ]),
            " serial_port:=",
            serial_port,
        ]),
        value_type=str,
    )

    controllers_file = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "config",
        "controllers.yaml",
    ])
    ekf_file = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "config",
        "ekf.yaml",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
        }],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_file,
        ],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_file],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    imu = Node(
        package="mecanum_bringup",
        executable="witmotion_ble_imu",
        name="witmotion_ble_imu",
        output="screen",
        condition=IfCondition(use_imu),
        parameters=[{
            "port": "/dev/ttyUSB0",
            "baud": 115200,
            "frame_id": "imu_link",
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("mecanum_bringup"), "rviz", "mecanum.rviz"
        ])],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB1",
            description="Arduino serial device (prefer a /dev/serial/by-id path)",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start RViz2 with the mecanum TF display",
        ),
        DeclareLaunchArgument(
            "use_imu", default_value="true",
            description="Connect to the WitMotion WT901 BLE IMU",
        ),
        DeclareLaunchArgument(
            "imu_address", default_value="",
            description="WT901 BLE MAC address; empty scans by device name",
        ),
        robot_state_publisher,
        controller_manager,
        imu,
        ekf,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[mecanum_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mecanum_controller_spawner,
                on_exit=[rviz],
            )
        ),
    ])
