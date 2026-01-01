#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """
    Full launch:
      - Starts my_robot_navigation2 bringup launch (Nav2 + RViz) first (IncludeLaunchDescription)
      - Starts YOLO node immediately
      - Starts custom Nav waypoint node after a configurable delay
      - YAML filenames are passed by CLI (no absolute paths needed)
      - YAML files for YOLO/nav-waypoints are assumed to live in:
          <my_robot_ai_identification_share>/config/
      - Nav2 bringup launch arguments are exposed/passed-through:
          map_file, params_file, use_sim_time
    """

    # -------------------------------------------------------------------------
    # Paths
    # -------------------------------------------------------------------------
    ai_pkg_share = get_package_share_directory('my_robot_ai_identification')
    ai_config_dir = os.path.join(ai_pkg_share, 'config')

    nav2_pkg_share = get_package_share_directory('my_robot_navigation2')
    nav2_launch_path = os.path.join(nav2_pkg_share, 'launch', 'navigation2_robot.launch.py')
    # IMPORTANT:
    # Replace *.launch.py with the actual filename
    # inside my_robot_navigation2/launch/ (the file that contains the code you pasted).

    # -------------------------------------------------------------------------
    # Launch arguments (filenames only, inside my_robot_ai_identification/config/)
    # -------------------------------------------------------------------------
    nav_params = LaunchConfiguration('nav_params')      # e.g. yolo_targets.yaml
    yolo_params = LaunchConfiguration('yolo_params')    # e.g. yolo_signals.yaml
    nav_start_delay = LaunchConfiguration('nav_start_delay')

    declare_nav_params = DeclareLaunchArgument(
        'nav_params',
        default_value='yolo_targets.yaml',
        description='Nav waypoint params YAML filename inside my_robot_ai_identification/config/'
    )

    declare_yolo_params = DeclareLaunchArgument(
        'yolo_params',
        default_value='yolo_signals.yaml',
        description='YOLO params YAML filename inside my_robot_ai_identification/config/'
    )

    declare_delay = DeclareLaunchArgument(
        'nav_start_delay',
        default_value='2.0',
        description='Seconds to wait before starting the custom waypoint navigation node'
    )

    # -------------------------------------------------------------------------
    # Pass-through arguments for my_robot_navigation2 launch
    # (Expose them here so you can set them from the terminal)
    # -------------------------------------------------------------------------
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='map_square3m_walls.yaml',
        description="Map filename inside my_robot_navigation2/map/"
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='rubot_sw.yaml',
        description="Nav2 params filename inside my_robot_navigation2/param/"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # -------------------------------------------------------------------------
    # Include my_robot_navigation2 launch (Nav2 bringup + RViz)
    # We pass the arguments through exactly by name.
    # -------------------------------------------------------------------------
    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map_file': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Build the YOLO + custom waypoint nodes dynamically to resolve filenames
    # -------------------------------------------------------------------------
    def _launch_setup(context, *args, **kwargs):
        nav_yaml = os.path.join(ai_config_dir, nav_params.perform(context))
        yolo_yaml = os.path.join(ai_config_dir, yolo_params.perform(context))

        yolo_node = Node(
            package='my_robot_ai_identification',
            executable='rubot_detection_yolo_exec',
            name='object_detection',
            output='screen',
            parameters=[
                yolo_yaml,
                {'use_sim_time': use_sim_time.perform(context)},
            ],
        )

        nav_node = Node(
            package='my_robot_ai_identification',
            executable='rubot_targets_yolo_exec',
            name='custom_nav2',
            output='screen',
            parameters=[
                nav_yaml,
                {'use_sim_time': use_sim_time.perform(context)},
            ],
        )

        nav_delayed = TimerAction(
            period=float(nav_start_delay.perform(context)),
            actions=[nav_node],
        )

        return [yolo_node, nav_delayed]

    return LaunchDescription([
        # Expose Nav2 bringup args
        declare_map_file,
        declare_params_file,
        declare_use_sim_time,

        # Expose AI/nav-waypoint YAML args
        declare_nav_params,
        declare_yolo_params,
        declare_delay,

        # 1) Start Nav2 bringup + RViz first (included launch)
        nav2_bringup_include,

        # 2) Start YOLO now, and custom waypoint nav after delay
        OpaqueFunction(function=_launch_setup),
    ])
