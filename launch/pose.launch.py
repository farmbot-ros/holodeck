from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    tcp = LaunchConfiguration("tcp").perform(context)
    trajectory = int(LaunchConfiguration("trajectory").perform(context))
    color = LaunchConfiguration("color").perform(context)

    nodes_array = []

    rerun = Node(
        package="farmbot_holodeck",
        executable="pose",
        name="pose",
        namespace=namespace,
        parameters=[
            # yaml.safe_load(open(param_file))['rerun']['ros__parameters'],
            # yaml.safe_load(open(param_file))['global']['ros__parameters'],
            {"tcp": tcp} if tcp != "" else {},
            {"trajectory": trajectory},
            {"color": color},
        ],
    )
    nodes_array.append(rerun)
    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="fbot")
    tcp_arg = DeclareLaunchArgument("tcp", default_value="")
    full_trajectory = DeclareLaunchArgument("trajectory", default_value="100")
    color = DeclareLaunchArgument("color", default_value="#ff0000")

    return LaunchDescription(
        [
            namespace_arg,
            tcp_arg,
            full_trajectory,
            color,
            OpaqueFunction(function=launch_setup),
        ]
    )
