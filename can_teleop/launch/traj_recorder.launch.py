from launch import LaunchDescription, launch_description_sources
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    BS_config_base = os.path.join(
                        get_package_share_directory("system-configuration"),
                        "config",
                        "BS",
                    )
    BS_config_file = os.path.join(
        BS_config_base,
        "config.yaml",
    )
    return LaunchDescription([
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
