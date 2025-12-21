import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ===================================================================================
    #   Declaració dels arguments configurables per a tot el robot
    # ===================================================================================
    
    # Argument per al temps de simulació, per ser consistent a tot el llançament
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Arguments per al driver del robot
    declare_mecanum_serial_port_arg = DeclareLaunchArgument(
        'mecanum_serial_port', default_value='/dev/ttyACM0',
        description='Port sèrie per a la placa dels motors'
    )

    # Arguments per al LiDAR
    declare_rplidar_serial_port_arg = DeclareLaunchArgument(
        'rplidar_serial_port', default_value='/dev/ttyUSB0',
        description='Port sèrie per al RPLiDAR'
    )
    declare_rplidar_frame_id_arg = DeclareLaunchArgument(
        'rplidar_frame_id', default_value='base_link',
        description='Frame ID per a les dades del LiDAR'
    )
    
    # Arguments per a la Càmera
    declare_camera_width_arg = DeclareLaunchArgument(
        'camera_width', default_value='160',
        description='Amplada de la imatge de la càmera'
    )
    declare_camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='120',
        description='Alçada de la imatge de la càmera'
    )

    # Argument per al model del robot
    declare_robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='robot_arm/my_simple_robot.urdf',
        description='Nom del fitxer URDF/XACRO dins del paquet de descripció'
    )

    # ===================================================================================
    #   Preparació de la configuració dels nodes
    # ===================================================================================
    
    # Obtenir el valor dels arguments per poder utilitzar-los
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')

    # Lògica per carregar el model del robot (URDF/XACRO)
    # IMPORTANT: Canvia 'my_robot_description' pel nom real del teu paquet de descripció.
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'), 'urdf', robot_model
        ])
    ])

    # Definició del node robot_state_publisher
    # El node joint_state_publisher esta definit en el driver
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Inclusió dels altres llançadors de drivers
    start_mecanum_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_driver'), 'launch', 'rubot_nano_driver_mecanum.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('mecanum_serial_port'),
            'use_sim_time': use_sim_time
        }.items()
    )
    
    start_rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('rplidar_serial_port'),
            'frame_id': LaunchConfiguration('rplidar_frame_id'),
            'use_sim_time': use_sim_time
        }.items()
    )

    start_usb_cam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_bringup'), 'launch', 'usb_cam_custom.launch.py')
        ),
        launch_arguments={
            'image_width': LaunchConfiguration('camera_width'),
            'image_height': LaunchConfiguration('camera_height'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # ===================================================================================
    #   Creació de la descripció del llançament final
    # ===================================================================================
    ld = LaunchDescription()

    # Afegir tots els arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_mecanum_serial_port_arg)
    ld.add_action(declare_rplidar_serial_port_arg)
    ld.add_action(declare_rplidar_frame_id_arg)
    ld.add_action(declare_camera_width_arg)
    ld.add_action(declare_camera_height_arg)
    ld.add_action(declare_robot_model_arg)
    
    # Afegir tots els nodes i llançadors
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_mecanum_driver_cmd)
    ld.add_action(start_rplidar_cmd)
    ld.add_action(start_usb_cam_cmd)

    return ld