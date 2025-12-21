import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments for image_width and image_height
    # These arguments will have default values but can be overridden when this launch file is included
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',  # Default width
        description='Width of the camera image'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',  # Default height
        description='Height of the camera image'
    )

    # Get the path to the default parameters file from usb_cam package
    # You might still want to load other parameters from a file,
    # or you can define them all directly here.
    # For simplicity, let's assume we are directly setting width and height.
    # If you still want to load a base YAML and then override, it's more complex.
    # For now, we'll just set width and height directly.

    # Define the usb_cam node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe', # Note: The original launch file uses usb_cam_node_exe
        name='usb_cam',
        output='screen',
        parameters=[
            {'image_width': LaunchConfiguration('image_width')},
            {'image_height': LaunchConfiguration('image_height')},
            # Add any other parameters you need here, e.g., from a custom YAML file
            # If you want to load a base YAML and then override, you'd need to
            # combine parameters. This example just sets width/height.
        ]
    )

    return LaunchDescription([
        image_width_arg,
        image_height_arg,
        usb_cam_node
    ])