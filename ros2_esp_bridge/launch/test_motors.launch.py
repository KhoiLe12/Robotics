"""
Simple launch file for testing motors with goal_relay
Does NOT launch Nav2 - only ESP bridge, LiDAR, and goal_relay
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    esp_port_arg = DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB0')
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1')

    esp_port = LaunchConfiguration('esp_port')
    lidar_port = LaunchConfiguration('lidar_port')

    # ESP32 serial bridge node
    esp_serial_bridge_node = Node(
        package='ros2_esp_bridge',
        executable='esp_serial_bridge',
        name='esp_serial_bridge',
        output='screen',
        parameters=[{'serial_port': esp_port}]
    )

    # Goal relay for testing (sends cmd_vel on /goal_pose clicks)
    goal_relay_node = Node(
        package='ros2_esp_bridge',
        executable='goal_relay',
        name='goal_relay',
        output='screen'
    )

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # Static transform: base_footprint -> base_link
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # Static transform: base_link -> laser_frame (rotated 90° left)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.1', '1.5708', '0', '0', 'base_link', 'laser_frame']
    )

    return LaunchDescription([
        esp_port_arg,
        lidar_port_arg,
        esp_serial_bridge_node,
        goal_relay_node,
        rplidar_node,
        static_tf_base,
        static_tf_laser
    ])
