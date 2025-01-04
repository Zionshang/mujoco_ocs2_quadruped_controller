from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_sim',
            executable='mujoco_sim',
            name='MujocoMsgHandler',
            output='screen'
        ),
    ])
