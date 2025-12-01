#!/usr/bin/env python3
"""
CMD_VEL Bridge - Converts ROS2 cmd_vel to ESP serial commands
Enables teleoperation and autonomous navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import time

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear_speed', 0.3)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('wheelbase', 0.24)  # meters
        self.declare_parameter('wheel_radius', 0.035)  # meters
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to ESP32 on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            raise
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timeout - stop if no command received
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # seconds
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('CMD_VEL Bridge ready for teleoperation and navigation')
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to differential drive wheel speeds and send to ESP32"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Clamp to maximum speeds
        linear = max(-self.max_linear_speed, min(self.max_linear_speed, linear))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        
        # Convert to wheel speeds using differential drive kinematics
        # v_left = linear - (angular * wheelbase / 2)
        # v_right = linear + (angular * wheelbase / 2)
        v_left = linear - (angular * self.wheelbase / 2.0)
        v_right = linear + (angular * self.wheelbase / 2.0)
        
        # Send to ESP32 via PID velocity control
        self.send_velocity_command(v_left, v_right)
        self.last_cmd_time = time.time()
    
    def send_velocity_command(self, v_left: float, v_right: float):
        """Send velocity command to ESP32 in PID mode"""
        try:
            # Switch to PID mode if not already
            # Format: PID_VEL,<left_speed>,<right_speed>
            cmd = f'PID_VEL,{v_left:.3f},{v_right:.3f}\n'
            self.ser.write(cmd.encode())
            
            self.get_logger().debug(f'Sent: L={v_left:.3f} m/s, R={v_right:.3f} m/s')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def safety_check(self):
        """Stop robot if no command received within timeout"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            # Send stop command
            try:
                self.ser.write(b'PID_VEL,0.0,0.0\n')
            except Exception as e:
                self.get_logger().error(f'Failed to send stop command: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        try:
            self.ser.write(b'PID_VEL,0.0,0.0\n')  # Stop robot
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
