import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

package_controller = "ocs2_quadruped_controller"


def launch_setup(context, *args, **kwargs):
    robot_pkg = context.launch_configurations["robot_pkg"]
    pkg_path = os.path.join(get_package_share_directory(robot_pkg))
    urdf_file = os.path.join(pkg_path, "urdf", "robot.urdf")

    robot_config_file = PathJoinSubstitution(
        [
            FindPackageShare(robot_pkg),
            "config",
            "robot_control.yaml",
        ]
    )

    # change urdf file to string
    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "use_tf_static": True,
                "robot_description": robot_desc,
                "ignore_timestamp": True,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_config_file,
            {
                "urdf_file": urdf_file,
                "task_file": os.path.join(
                    pkg_path,
                    "config",
                    "ocs2",
                    "task.info",
                ),
                "reference_file": os.path.join(
                    pkg_path,
                    "config",
                    "ocs2",
                    "reference.info",
                ),
                "gait_file": os.path.join(
                    pkg_path,
                    "config",
                    "ocs2",
                    "gait.info",
                ),
            },
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ocs2_quadruped_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    cmd_mapping = Node(
        package="cmd_mapping",
        executable="cmd_mapping",
    )

    nodes = [
        node
        for node in [
            cmd_mapping,
            robot_state_publisher,
            controller_manager,
            ocs2_controller,
        ]
        if node is not None
    ]

    return nodes


def generate_launch_description():
    robot_pkg = DeclareLaunchArgument(
        "robot_pkg",
        default_value="galileo_mini_description",
        description="package for robot description",
    )

    rviz_enable = DeclareLaunchArgument(
        "rviz_enable",
        default_value="false",
        description="enable rviz visualization",
    )

    return LaunchDescription(
        [
            robot_pkg,
            rviz_enable,
            OpaqueFunction(function=launch_setup),
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--",
                    "ros2",
                    "run",
                    "keyboard_input",
                    "keyboard_publisher",
                ],
                output="screen",
            ),
        ]
    )
