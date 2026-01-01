#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_ai_identification')
    nav_params = os.path.join(pkg_path, 'config', 'yolo_targets.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    nav_node = Node(
        package='my_robot_ai_identification',  
        executable='rubot_targets_yolo_exec',
        name='custom_nav2',
        output='screen',
        parameters=[
            nav_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        nav_node
    ])
