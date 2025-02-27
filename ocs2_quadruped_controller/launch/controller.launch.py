import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

robot_pkg_name = "galileo_mini15_description"

def generate_launch_description():

    ################################ parameters ################################
    robot_pkg_path = os.path.join(get_package_share_directory(robot_pkg_name))
    urdf_file = os.path.join(robot_pkg_path, "urdf", "robot.urdf")
    config_path = os.path.join(robot_pkg_path, "config")

    with open(urdf_file, "r") as f:
        urdf_content = f.read()

    ################################ node ################################
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": urdf_content}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([config_path, "robot_control.yaml"]),
            {
                "urdf_file": urdf_file,
                "task_file": os.path.join(config_path, "ocs2", "task.info"),
                "reference_file": os.path.join(config_path, "ocs2", "reference.info"),
                "gait_file": os.path.join(config_path, "ocs2", "gait.info"),
            },
        ],
        remappings=[("~/robot_description", "/robot_description")],
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

    keyboard_input = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "ros2",
            "run",
            "keyboard_input",
            "keyboard_publisher",
        ],
        output="screen",
    )

    nodes = [
        cmd_mapping,
        robot_state_publisher,
        controller_manager,
        ocs2_controller,
        keyboard_input,
    ]

    return LaunchDescription(nodes)
