from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for complete robot navigation system
    Includes: ESP bridge, LiDAR, SLAM/Localization, Nav2
    
    Usage:
    - Mapping mode: ros2 launch ros2_esp_bridge robot_navigation.launch.py slam_mode:=mapping
    - Localization mode: ros2 launch ros2_esp_bridge robot_navigation.launch.py slam_mode:=localization map_file:=/path/to/my_map.yaml
    """
    
    # Declare arguments
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    esp_port = LaunchConfiguration('esp_port', default='/dev/ttyUSB0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    slam_mode = LaunchConfiguration('slam_mode', default='mapping')
    map_file = LaunchConfiguration('map_file', default='')
    
    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to run SLAM Toolbox'
    )
    
    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to run Nav2 navigation'
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
    
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map YAML file for localization mode (e.g., /path/to/my_map.yaml)'
    )
    
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
    
    # CMD_VEL Bridge (converts nav/teleop commands to ESP serial)
    cmd_vel_bridge_node = Node(
        package='ros2_esp_bridge',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        parameters=[{
            'serial_port': esp_port,
            'baud_rate': 115200,
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'wheelbase': 0.24,
            'wheel_radius': 0.035
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
    
    # Static transform: base_link -> laser_frame
    # Adjust based on your LiDAR mounting position
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '1.5708', '0', '0', 'base_link', 'laser_frame', '--ros-args', '--log-level', 'warn'],
        # x y z yaw pitch roll parent child
        # LiDAR pointing LEFT: 90° (1.5708 rad) yaw offset
        output='screen'
    )
    
    # SLAM Toolbox (online mapping or localization)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': slam_mode,  # 'mapping' or 'localization'
            'map_file_name': map_file,  # Used in localization mode
            'map_start_pose': [0.0, 0.0, 0.0],  # [x, y, yaw] - can be set via 2D Pose Estimate in RViz
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'map_update_interval': 5.0
        }],
        condition=IfCondition(use_slam),
        output='screen'
    )
    
    # Nav2 (navigation stack)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(
        get_package_share_directory('ros2_esp_bridge'),
        'config',
        'nav2_params.yaml'
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items(),
        condition=IfCondition(use_nav2)
    )
    
    return LaunchDescription([
        declare_use_slam,
        declare_use_nav2,
        declare_esp_port,
        declare_lidar_port,
        declare_slam_mode,
        declare_map_file,
        esp_bridge_node,
        # cmd_vel_bridge_node,  # DISABLED - conflicts with esp_serial_bridge on same port
        lidar_node,
        base_to_laser_tf,
        slam_toolbox_node,
        nav2_launch
    ])
