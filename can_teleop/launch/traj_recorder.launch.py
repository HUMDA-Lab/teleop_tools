from launch import LaunchDescription, launch_description_sources
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_folder_path = LaunchConfiguration('config_folder')
    BS_config_base = PathJoinSubstitution([
                        config_folder_path,
                        "BS",
                    ])
    BS_config_file = PathJoinSubstitution([
                        BS_config_base,
                        "config.yaml",
                    ])
    return LaunchDescription([
        DeclareLaunchArgument('config_folder', default_value="system_config/config/"),
        SetEnvironmentVariable(name='ROS2_BS_CONFIG_BASE', value=BS_config_base),
        SetEnvironmentVariable(name='ROS2_BS_CONFIG_PATH', value=BS_config_file),
        SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='LOCALHOST'),
        Node(
            package='eav24_bsu',
            executable='eav24_bsu',
            output='screen',
        ),
        IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('can_teleop'), 'launch', 'logitechwheel.launch.py')
            )
        )
    ])
