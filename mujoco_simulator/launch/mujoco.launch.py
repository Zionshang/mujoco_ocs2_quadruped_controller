from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription


robot_pkg_name = "galileo_mini15_description"


def generate_launch_description():

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
        package="mujoco_simulator",
        executable="mujoco_simulator",
        output="screen",
        parameters=[{"xml_file_path": xml_file_path}],
    )

    return LaunchDescription([mujoco_node])
