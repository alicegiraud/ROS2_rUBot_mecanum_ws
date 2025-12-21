#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """
    Full launch:
      - Starts YOLO node immediately
      - Starts Nav2 node after a configurable delay
      - YAML filenames are passed by CLI (no absolute paths needed)
      - YAML files are assumed to live in:
          <my_robot_ai_identification_share>/config/
    """

    pkg_share = get_package_share_directory('my_robot_ai_identification')
    config_dir = os.path.join(pkg_share, 'config')

    # Pass only filenames (stored in config/)
    nav_params = LaunchConfiguration('nav_params')    # e.g. yolo_targets.yaml
    yolo_params = LaunchConfiguration('yolo_params')  # e.g. yolo_signals.yaml

    # Delay for Nav2 node (seconds)
    nav_start_delay = LaunchConfiguration('nav_start_delay')

    declare_nav_params = DeclareLaunchArgument(
        'nav_params',
        default_value='yolo_targets.yaml',
        description='Nav2 params YAML filename inside my_robot_ai_identification/config/'
    )

    declare_yolo_params = DeclareLaunchArgument(
        'yolo_params',
        default_value='yolo_signals.yaml',
        description='YOLO params YAML filename inside my_robot_ai_identification/config/'
    )

    declare_delay = DeclareLaunchArgument(
        'nav_start_delay',
        default_value='2.0',
        description='Seconds to wait before starting Nav2 node'
    )

    def _launch_setup(context, *args, **kwargs):
        # Resolve filenames to absolute paths inside <pkg_share>/config
        nav_yaml = os.path.join(config_dir, nav_params.perform(context))
        yolo_yaml = os.path.join(config_dir, yolo_params.perform(context))

        yolo_node = Node(
            package='my_robot_ai_identification',
            executable='rubot_detection_wp_yolo_exec',
            name='object_detection',
            output='screen',
            parameters=[yolo_yaml]
        )

        nav_node = Node(
            package='my_robot_ai_identification',
            executable='rubot_nav2_wp_yolo_exec',
            name='custom_nav2',
            output='screen',
            parameters=[nav_yaml]
        )

        nav_delayed = TimerAction(
            period=float(nav_start_delay.perform(context)),
            actions=[nav_node]
        )

        return [
            yolo_node,
            nav_delayed
        ]

    return LaunchDescription([
        declare_nav_params,
        declare_yolo_params,
        declare_delay,
        OpaqueFunction(function=_launch_setup),
    ])
