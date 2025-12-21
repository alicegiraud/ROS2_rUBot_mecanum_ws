from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch navigation waypoints node using a YAML file located in the
    package's config/ directory. Only the filename is required.
    """

    pkg_dir = get_package_share_directory('my_robot_nav_control')
    config_dir = os.path.join(pkg_dir, 'config')

    # Default YAML filename (NOT full path)
    default_yaml = 'waypoints_sw.yaml'

    yaml_file_arg = DeclareLaunchArgument(
        'wp_file',
        default_value=default_yaml,
        description='YAML filename inside my_robot_nav_control/config'
    )

    def launch_setup(context, *args, **kwargs):
        yaml_filename = LaunchConfiguration('wp_file').perform(context)
        params_path = os.path.join(config_dir, yaml_filename)

        if not os.path.isfile(params_path):
            raise RuntimeError(
                f'Parameter file not found: {params_path}'
            )

        navigation_node = Node(
            package='my_robot_nav_control',
            executable='nav_waypoints_exec',
            name='nav_waypoints_node',
            parameters=[params_path],
            output='screen',
        )

        return [navigation_node]

    return LaunchDescription([
        yaml_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
