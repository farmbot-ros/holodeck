import os
import yaml
import argparse
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    tcp = LaunchConfiguration('tcp').perform(context)
    foxglove = LaunchConfiguration('foxglove').perform(context)
    trajectory = int(LaunchConfiguration('trajectory').perform(context))
    # param_file = os.path.join(get_package_share_directory('farmbot_holodeck'), 'config', 'params.yaml')

    nodes_array = []

    rerun = Node(
        package='farmbot_holodeck',
        executable="pose",
        name='pose',
        namespace=namespace,
        parameters=[
            # yaml.safe_load(open(param_file))['rerun']['ros__parameters'],
            # yaml.safe_load(open(param_file))['global']['ros__parameters'],
            {'tcp': tcp} if tcp != '' else {},
            {'trajectory': trajectory}
        ]
    )
    nodes_array.append(rerun)


    # add this lunach: ros launch foxglove_bridge foxglove_bridge_launch.xml

    foxglove_launch_dir = get_package_share_path("foxglove_bridge")
    foxglove_launch_file = os.path.join(foxglove_launch_dir, "launch", "foxglove_bridge_launch.xml")

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_file)
    )
    if foxglove == "true":
        nodes_array.append(foxglove_bridge)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fbot')
    tcp_arg = DeclareLaunchArgument('tcp', default_value='')
    foxglove = DeclareLaunchArgument('foxglove', default_value="false")
    full_trajectory = DeclareLaunchArgument('trajectory', default_value="100")

    return LaunchDescription([
        namespace_arg,
        tcp_arg,
        foxglove,
        full_trajectory,
        OpaqueFunction(function = launch_setup)
        ]
    )
