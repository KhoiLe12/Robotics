"""
Minimal navigation launch - only controller and planner, NO behavior server
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    map_file = LaunchConfiguration('map')
    esp_port = LaunchConfiguration('esp_port', default='/dev/ttyUSB0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    
    declare_map = DeclareLaunchArgument('map', description='Full path to map YAML file')
    declare_esp_port = DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB0')
    declare_lidar_port = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1')
    
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('ros2_esp_bridge')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    bt_xml_file = os.path.join(pkg_dir, 'config', 'navigate_w_no_recoveries.xml')
    
    # ESP Serial Bridge
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
    
    # LiDAR Driver
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
    
    # Static TFs
    footprint_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0.1', '1.5708', '0', '0', 'base_link', 'laser_frame']
    )
    
    # AMCL Localization
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
    
    # Controller Server (path following only)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    # Planner Server (global path planning)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # BT Navigator (coordinates planning and control - WITHOUT behaviors)
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'default_nav_to_pose_bt_xml': bt_xml_file,
                'default_nav_through_poses_bt_xml': bt_xml_file
            }
        ]
    )
    
    # Lifecycle Manager for navigation nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 'bt_navigator']
        }]
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
        controller_server,
        planner_server,
        bt_navigator,
        lifecycle_manager
    ])
