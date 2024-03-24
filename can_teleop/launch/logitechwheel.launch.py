from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch import LaunchDescription, launch_description_sources
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.0
            }],
        ),
        Node(
            package='can_teleop',
            executable='can_teleop_executable',
            output='screen',
        ),
    ])