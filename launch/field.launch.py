from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    tcp = LaunchConfiguration('tcp').perform(context)

    nodes_array = []
    rerun = Node(
        package='farmbot_holodeck',
        executable="field",
        name='field',
        namespace=namespace,
        parameters=[
            {'tcp': tcp} if tcp != '' else {},
        ]
    )
    nodes_array.append(rerun)
    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fbot')
    tcp_arg = DeclareLaunchArgument('tcp', default_value='')

    return LaunchDescription([
        namespace_arg,
        tcp_arg,
        OpaqueFunction(function = launch_setup)
    ])
