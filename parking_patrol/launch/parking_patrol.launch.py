import os
import sys
import yaml
import fnmatch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    cfg_path = os.path.join(get_package_share_directory(
        'parking_patrol'), 'config', 'parking_patrol.yaml')

    with open(cfg_path, 'r') as config_o:
        config = yaml.safe_load(config_o)
    ros_param = config['/**']['ros__parameters']

    controllers_node = Node(
        package='parking_patrol',
        executable='parking_patrol',
        parameters=[ros_param],
        output={
            'stdout': 'screen',
            'stderr': 'screen'
        },
        emulate_tty= 'true'
    )

    ld.add_action(controllers_node)

    return ld
