import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_controller = "ocs2_quadruped_controller"


def launch_setup(context, *args, **kwargs):
    robot_pkg = context.launch_configurations["robot_pkg"]
    rviz_enable = context.launch_configurations["rviz_enable"]

    pkg_path = os.path.join(get_package_share_directory(robot_pkg))
    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    rviz_config_file = os.path.join(
        get_package_share_directory("ocs2_quadruped_controller"),
        "config",
        "visualize_ocs2.rviz",
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(robot_pkg),
            "config",
            "robot_control.yaml",
        ]
    )

    if rviz_enable == "true":
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_ocs2",
            output="screen",
            arguments=["-d", rviz_config_file],
        )
    else:
        rviz = None

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "publish_frequency": 20.0,
                "use_tf_static": True,
                "robot_description": xacro.process_file(xacro_file).toxml(),
                "ignore_timestamp": True,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {
                "urdf_file": os.path.join(
                    pkg_path,
                    "urdf",
                    "robot.urdf",
                ),
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

    nodes = [
        node
        for node in [rviz, robot_state_publisher, controller_manager, ocs2_controller]
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
        ]
    )
