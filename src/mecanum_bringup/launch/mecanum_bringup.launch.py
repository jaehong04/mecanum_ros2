from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("mecanum_bringup"),
                "urdf",
                "mecanum_robot.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    controllers_file = PathJoinSubstitution([
        FindPackageShare("mecanum_bringup"),
        "config",
        "controllers.yaml",
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

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
    ])
