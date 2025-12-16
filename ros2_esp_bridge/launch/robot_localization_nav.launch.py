from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Complete robot navigation with AMCL localization
    
    Usage:
    ros2 launch ros2_esp_bridge robot_localization_nav.launch.py map:=/home/password/maps/my_map.yaml
    """
    
    # Declare arguments
    map_file = LaunchConfiguration('map', default='/home/password/maps/my_map.yaml')
    esp_port = LaunchConfiguration('esp_port', default='/dev/ttyUSB0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/password/maps/my_map.yaml',
        description='Full path to map YAML file'
    )
    
    declare_esp_port = DeclareLaunchArgument(
        'esp_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32'
    )
    
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLidar'
    )
    
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ESP Serial Bridge (odometry + telemetry)
    esp_bridge_node = Node(
        package='ros2_esp_bridge',
        executable='esp_serial_bridge',
        name='esp_serial_bridge',
        parameters=[{
            'port': esp_port,
            'baud': 115200,
            'wheel_radius': 0.035,
            'wheelbase': 0.24,
            'ticks_per_rev': 231
        }],
        output='screen'
    )
    
    # LiDAR Driver (RPLidar A1)
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )
    
    # Static transform: base_footprint -> base_link (robot center at ground level)
    footprint_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    
    # Static transform: base_link -> laser_frame (LiDAR pointing LEFT)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '1.5708', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # Get nav2 params file path (shared by both localization and navigation)
    nav2_params_file = os.path.join(
        get_package_share_directory('ros2_esp_bridge'),
        'config',
        'nav2_params.yaml'
    )
    
    # Nav2 Localization (AMCL + Map Server)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_file,
            'params_file': nav2_params_file
        }.items()
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items()
    )
    
    return LaunchDescription([
        declare_map,
        declare_esp_port,
        declare_lidar_port,
        esp_bridge_node,
        lidar_node,
        footprint_to_base_tf,
        base_to_laser_tf,
        localization_launch,
        navigation_launch
    ])
