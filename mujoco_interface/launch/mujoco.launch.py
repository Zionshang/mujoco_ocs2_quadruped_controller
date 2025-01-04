from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():

    # 设置参数的默认值和描述
    robot_pkg = DeclareLaunchArgument(
        "robot_pkg",
        default_value="galileo_mini_description",
        description="package for robot description",
    )

    # 读取参数
    robot_pkg_name = LaunchConfiguration("robot_pkg")

    # 根据参数找到xml文件
    xml_file_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_pkg_name),  # 包路径
            "xml",
            "scene.xml",
        ]
    )

    # 加载mujoco node
    mujoco_node = Node(
        package="mujoco_interface",
        executable="mujoco_interface",
        output="screen",
        parameters=[{"xml_file_path": xml_file_path}],
    )

    return LaunchDescription([robot_pkg, mujoco_node])
