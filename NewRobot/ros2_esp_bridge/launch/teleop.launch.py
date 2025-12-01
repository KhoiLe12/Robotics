from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Teleoperation launch file - for manual control during mapping
    """
    
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32'
    )
    
    # Teleop Twist Keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'  # Opens in new terminal
    )
    
    return LaunchDescription([
        declare_serial_port,
        teleop_node
    ])
