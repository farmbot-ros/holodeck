import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
import argparse
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    # param_file = os.path.join(get_package_share_directory('farmbot_holodeck'), 'config', 'params.yaml')

    nodes_array = []

    rerun = Node(
        package='farmbot_holodeck',
        executable="rerun",
        name='rerun',
        namespace=namespace,
        parameters=[
            # yaml.safe_load(open(param_file))['rerun']['ros__parameters'],
            # yaml.safe_load(open(param_file))['global']['ros__parameters'],
            {'controller': controller} if controller != '' else {}
        ]
    )
    nodes_array.append(rerun)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fbot')
    controller_arg = DeclareLaunchArgument('controller', default_value='')

    return LaunchDescription([
        namespace_arg,
        controller_arg,
        OpaqueFunction(function = launch_setup)
        ]
    )
