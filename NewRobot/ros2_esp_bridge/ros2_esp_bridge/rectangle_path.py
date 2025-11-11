#!/usr/bin/env python3
"""
Rectangle Path Navigation Script

Prompts user for rectangle dimensions, then commands the robot to:
1. Drive straight for width distance
2. Turn 90° left  
3. Drive straight for height distance
4. Turn 90° left
5. Repeat until rectangle is complete
6. Log corner coordinates using odometry data
"""

import serial
import time
import math
import json
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

class RectanglePathController:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200, ros2_mode=False):
        """Initialize the rectangle path controller"""
        # Robot parameters (from odometry calculations)
        self.wheel_radius = 0.035  # meters
        self.wheelbase = 0.24      # meters  
        self.ticks_per_rev = 231   # encoder ticks per wheel revolution
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Movement parameters  
        self.forward_speed = 200   # PWM value for forward movement
        self.speed = 0.2  # meters/second (needs calibration)
        self.turn_duration = 900  # milliseconds for 90° turn (reduced from 1500ms)

        self.ros2_mode = ros2_mode
        self.ser = None
        if not self.ros2_mode:
            # Serial connection
            try:
                import serial
                self.ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
                print(f"✅ Connected to robot at {serial_port}")
                time.sleep(2)  # Wait for ESP32 to be ready
            except Exception as e:
                print(f"❌ Failed to connect to robot: {e}")
                raise
        else:
            print("ℹ️  RectanglePathController running in ROS2 mode (no serial)")

        # Path logging
        self.path_log = []
        self.corner_positions = []
        
    def get_rectangle_dimensions(self):
        """Get rectangle dimensions from user input"""
        print("\n" + "="*50)
        print("🏃 RECTANGLE PATH NAVIGATOR")
        print("="*50)
        
        while True:
            try:
                width = float(input("📏 Enter rectangle width (meters): "))
                if width <= 0:
                    print("❌ Width must be positive!")
                    continue
                break
            except ValueError:
                print("❌ Please enter a valid number!")
        
        while True:
            try:
                height = float(input("📏 Enter rectangle height (meters): "))
                if height <= 0:
                    print("❌ Height must be positive!")
                    continue
                break
            except ValueError:
                print("❌ Please enter a valid number!")
        
        print(f"\n📋 Rectangle Plan:")
        print(f"   Width:  {width}m")
        print(f"   Height: {height}m")
        print(f"   Perimeter: {2*(width+height)}m")
        
        # Calculate expected encoder ticks
        width_ticks = int(width / self.meters_per_tick)
        height_ticks = int(height / self.meters_per_tick)
        print(f"   Expected ticks - Width: {width_ticks}, Height: {height_ticks}")
        
        confirm = input("\n✅ Start rectangle path? (y/n): ").lower().strip()
        if confirm != 'y':
            print("❌ Path cancelled")
            return None, None
            
        return width, height
    
    def send_command(self, command, duration_ms=None, speed=None):
        """Send command to ESP32 (serial) or publish Twist in ROS2 mode"""
        if self.ros2_mode:
            # ROS2-native: publish to /cmd_vel
            # Import ROS2 dependencies here to avoid import errors on non-ROS2 systems or in serial mode
            if not hasattr(self, 'cmd_vel_pub'):
                import rclpy
                from geometry_msgs.msg import Twist
                self._rclpy = rclpy
                self._Twist = Twist
                self.cmd_vel_pub = self.ros2_node.create_publisher(Twist, '/cmd_vel', 10)
            twist = self._Twist()
            if command.startswith('FORWARD'):
                twist.linear.x = speed if speed is not None else self.speed
                twist.angular.z = 0.0
            elif command.startswith('TURN_LEFT'):
                twist.linear.x = 0.0
                twist.angular.z = math.pi/2 / (duration_ms/1000.0) if duration_ms else math.pi/2
            else:
                print(f"[ROS2 mode] Unknown command: {command}")
                return None
            print(f"[ROS2 mode] Publishing Twist: {twist}")
            self.cmd_vel_pub.publish(twist)
            # Sleep for duration, then stop
            if duration_ms:
                time.sleep(duration_ms/1000.0)
                stop = self._Twist()
                self.cmd_vel_pub.publish(stop)
            return None
        # Serial mode
        print(f"📤 Sending: {command}")
        self.ser.write(f"{command}\n".encode())
        # Wait for acknowledgment
        start_time = time.time()
        while time.time() - start_time < 5.0:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if response.startswith('A,'):
                    print(f"📥 Response: {response}")
                    return response
                elif response.startswith('T,'):
                    self.parse_telemetry(response)
        print("⚠️  No response received")
        return None
    
    def parse_telemetry(self, line):
        """Parse telemetry line: T,<ms>,<L>,<R>,<s1>,<s2>,<s3>,<s4>,<s5>"""
        try:
            parts = line.split(',')
            if len(parts) == 9:
                timestamp = int(parts[1])
                left_ticks = int(parts[2])
                right_ticks = int(parts[3])
                sensors = [int(parts[i]) for i in range(4, 9)]
                
                # Log the position data
                position_data = {
                    'timestamp': timestamp,
                    'left_ticks': left_ticks,
                    'right_ticks': right_ticks,
                    'sensors': sensors
                }
                self.path_log.append(position_data)
                
                return position_data
        except (ValueError, IndexError):
            pass
        return None
    
    def reset_encoders(self):
        """Reset encoder counts to zero"""
        if self.ros2_mode:
            print("[ROS2 mode] Would reset encoders via ROS2 service")
            # Optionally, call a ROS2 service here
            if hasattr(self, 'ros2_node'):
                from std_srvs.srv import Empty
                from rclpy.task import Future
                client = self.ros2_node.create_client(Empty, 'esp/reset_encoders')
                if not client.wait_for_service(timeout_sec=2.0):
                    print("[ROS2 mode] Reset encoder service not available!")
                    return False
                req = Empty.Request()
                future = client.call_async(req)
                while not future.done():
                    time.sleep(0.1)
                print("[ROS2 mode] Encoder reset service called.")
                return True
            return True
        print("🔄 Resetting encoders...")
        response = self.send_command("R")
        time.sleep(0.5)
        return response is not None
    
    def get_current_position(self):
        """Get current encoder readings (serial mode only)"""
        if self.ros2_mode:
            print("[ROS2 mode] get_current_position() not available; override in subclass for odom.")
            return None
        self.ser.write(b"GET\n")
        time.sleep(0.1)
        latest_data = None
        for _ in range(10):
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('T,'):
                    latest_data = self.parse_telemetry(line)
        return latest_data
    
    def move_straight(self, distance_meters):
        """Move robot straight for specified distance"""
        expected_ticks = int(distance_meters / self.meters_per_tick)
        duration_ms = int((distance_meters / 0.16) * 1000)
        print(f"🏃 Moving straight {distance_meters}m (≈{expected_ticks} ticks, {duration_ms}ms)")
        if self.ros2_mode:
            self.send_command('FORWARD', duration_ms=duration_ms, speed=self.speed)
            return True
        # Serial mode
        start_pos = self.get_current_position()
        if start_pos:
            start_ticks = (start_pos['left_ticks'] + start_pos['right_ticks']) // 2
        else:
            start_ticks = 0
        response = self.send_command(f"FORWARD,{self.forward_speed},{duration_ms}")
        time.sleep(0.5)
        end_pos = self.get_current_position()
        if end_pos:
            end_ticks = (end_pos['left_ticks'] + end_pos['right_ticks']) // 2
            actual_distance = (end_ticks - start_ticks) * self.meters_per_tick
            print(f"📊 Actual distance: {actual_distance:.3f}m ({end_ticks - start_ticks} ticks)")
        return response is not None
    
    def turn_left_90(self):
        """Turn robot 90 degrees to the left"""
        print(f"↺ Turning left 90° ({self.turn_duration}ms)")
        if self.ros2_mode:
            self.send_command('TURN_LEFT', duration_ms=self.turn_duration)
            return True
        # Serial mode
        start_pos = self.get_current_position()
        response = self.send_command(f"TURN_LEFT,{self.turn_duration}")
        time.sleep(0.5)
        end_pos = self.get_current_position()
        return response is not None
    
    def log_corner_position(self, corner_num):
        """Log current position as a corner (encoder or odometry)"""
        pos_data = self.get_current_position()
        if self.ros2_mode and pos_data and ('x' in pos_data and 'y' in pos_data):
            # Log odometry-based position
            corner_info = {
                'corner': corner_num,
                'timestamp': pos_data.get('timestamp', None),
                'x': pos_data['x'],
                'y': pos_data['y'],
                'time': datetime.now().isoformat()
            }
            self.corner_positions.append(corner_info)
            print(f"📍 Corner {corner_num} logged: x={pos_data['x']:.3f}, y={pos_data['y']:.3f}")
        elif pos_data and ('left_ticks' in pos_data and 'right_ticks' in pos_data):
            # Log encoder-based position
            corner_info = {
                'corner': corner_num,
                'timestamp': pos_data['timestamp'],
                'left_ticks': pos_data['left_ticks'],
                'right_ticks': pos_data['right_ticks'],
                'time': datetime.now().isoformat()
            }
            self.corner_positions.append(corner_info)
            print(f"📍 Corner {corner_num} logged: L={pos_data['left_ticks']}, R={pos_data['right_ticks']}")
    
    def execute_rectangle_path(self, width, height):
        """Execute the complete rectangle path"""
        print(f"\n🚀 Starting rectangle path: {width}m × {height}m")
        
        # Reset encoders to start fresh
        if not self.reset_encoders():
            print("❌ Failed to reset encoders")
            return False
        
        # Log starting position (corner 0)
        time.sleep(1)
        self.log_corner_position(0)
        
        # Execute rectangle path
        sides = [width, height, width, height]  # Width, Height, Width, Height
        side_names = ["Width", "Height", "Width", "Height"]
        
        for i, (distance, name) in enumerate(zip(sides, side_names)):
            print(f"\n--- Side {i+1}: {name} ({distance}m) ---")
            
            # Move straight
            if not self.move_straight(distance):
                print(f"❌ Failed to move {name}")
                return False
            
            # Log corner position
            self.log_corner_position(i + 1)
            
            # Turn left (except after the last side)
            if i < 3:
                if not self.turn_left_90():
                    print("❌ Failed to turn left")
                    return False
                time.sleep(1)  # Pause between sides
        
        print("\n🎉 Rectangle path completed!")
        return True
    
    def plot_rectangle_path(self, width, height):
        """Plot the robot's path with highlighted corners (supports encoder or odometry)"""
        if len(self.corner_positions) < 2:
            print("❌ Not enough corner data for plotting")
            return
        # Set up the plot with square aspect ratio
        plt.figure(figsize=(10, 10))
        # Expected rectangle corners for comparison
        expected_corners = [
            (0.0, 0.0),      # Corner 0: Start
            (width, 0.0),    # Corner 1: After first side
            (width, height), # Corner 2: After second side  
            (0.0, height),   # Corner 3: After third side
            (0.0, 0.0)       # Corner 4: Back to start
        ]
        # Try to extract odometry-based positions if present
        if 'x' in self.corner_positions[0] and 'y' in self.corner_positions[0]:
            # Odometry-based
            x_positions = [corner['x'] for corner in self.corner_positions]
            y_positions = [corner['y'] for corner in self.corner_positions]
        elif 'left_ticks' in self.corner_positions[0] and 'right_ticks' in self.corner_positions[0]:
            # Encoder-based (dead reckoning)
            x_positions = [0.0]
            y_positions = [0.0]
            current_x, current_y = 0.0, 0.0
            current_angle = 0.0
            for i in range(1, len(self.corner_positions)):
                prev_corner = self.corner_positions[i-1]
                curr_corner = self.corner_positions[i]
                prev_total = prev_corner['left_ticks'] + prev_corner['right_ticks']
                curr_total = curr_corner['left_ticks'] + curr_corner['right_ticks']
                tick_diff = curr_total - prev_total
                distance_moved = tick_diff * self.meters_per_tick / 2.0
                current_x += distance_moved * math.cos(current_angle)
                current_y += distance_moved * math.sin(current_angle)
                x_positions.append(current_x)
                y_positions.append(current_y)
                if i < len(self.corner_positions) - 1:
                    current_angle += math.pi / 2
        else:
            print("❌ Unknown corner position format for plotting")
            return
        # Convert to centimeters for better readability
        x_cm = [x * 100 for x in x_positions]
        y_cm = [y * 100 for y in y_positions]
        exp_x_cm = [x * 100 for x, y in expected_corners]
        exp_y_cm = [y * 100 for x, y in expected_corners]
        # Plot the paths
        plt.plot(exp_x_cm, exp_y_cm, 'k--', linewidth=3, marker='s', markersize=12,
                label='Expected Rectangle', alpha=0.7, markerfacecolor='black')
        plt.plot(x_cm, y_cm, 'b-', linewidth=4, marker='o', markersize=14,
                label='Actual Robot Path', alpha=0.9, markerfacecolor='blue')
        # Highlight corners with numbers
        for i, (x, y) in enumerate(zip(x_cm, y_cm)):
            plt.annotate(f'C{i}', (x, y), xytext=(10, 10), textcoords='offset points',
                        fontsize=12, fontweight='bold', color='red',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
        # Set up 100x100cm grid
        max_dim = max(width, height) * 100 + 20  # Add 20cm border
        grid_size = max(100, int(max_dim // 10) * 10 + 10)  # At least 100cm, round up to 10cm
        plt.xlim(-10, grid_size)
        plt.ylim(-10, grid_size)
        # Grid every 10cm
        major_ticks = np.arange(0, grid_size + 10, 10)
        minor_ticks = np.arange(0, grid_size + 5, 5)
        plt.gca().set_xticks(major_ticks)
        plt.gca().set_yticks(major_ticks)
        plt.gca().set_xticks(minor_ticks, minor=True)
        plt.gca().set_yticks(minor_ticks, minor=True)
        # Grid styling
        plt.grid(True, which="major", alpha=0.6, linewidth=1.0, color='gray')
        plt.grid(True, which="minor", alpha=0.3, linewidth=0.5, linestyle=':', color='gray')
        # Labels and formatting
        plt.xlabel('X Position (cm)', fontsize=14, fontweight='bold')
        plt.ylabel('Y Position (cm)', fontsize=14, fontweight='bold')
        plt.title(f'Rectangle Path Navigation\n{width}×{height}m ({width*100:.0f}×{height*100:.0f}cm)', 
                 fontsize=16, fontweight='bold', pad=20)
        # Legend
        plt.legend(fontsize=12, loc='upper right', framealpha=0.9)
        # Equal aspect ratio
        plt.axis('equal')
        # Save and show
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_filename = f'rectangle_path_plot_{timestamp}.png'
        plt.tight_layout()
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight', facecolor='white')
        plt.show()
        print(f"📊 Path plot saved as: {plot_filename}")
        # Calculate and display accuracy metrics
        if len(expected_corners) == len(x_positions):
            print(f"\n📐 Path Accuracy Analysis:")
            total_error = 0
            for i, ((exp_x, exp_y), act_x, act_y) in enumerate(zip(expected_corners, x_positions, y_positions)):
                error = math.sqrt((exp_x - act_x)**2 + (exp_y - act_y)**2)
                total_error += error
                print(f"   Corner {i}: Expected({exp_x:.3f}, {exp_y:.3f})m, Actual({act_x:.3f}, {act_y:.3f})m, Error: {error:.3f}m")
            avg_error = total_error / len(expected_corners)
            print(f"   📊 Average corner error: {avg_error:.3f}m ({avg_error*100:.1f}cm)")
    
    def save_results(self, width, height):
        """Save path results to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"rectangle_path_{timestamp}.json"
        
        results = {
            'rectangle_dimensions': {
                'width': width,
                'height': height,
                'perimeter': 2 * (width + height)
            },
            'robot_parameters': {
                'wheel_radius': self.wheel_radius,
                'wheelbase': self.wheelbase,
                'ticks_per_rev': self.ticks_per_rev,
                'meters_per_tick': self.meters_per_tick
            },
            'movement_parameters': {
                'forward_speed': self.forward_speed,
                'turn_duration': self.turn_duration
            },
            'corner_positions': self.corner_positions,
            'full_path_log': self.path_log[-100:],  # Last 100 telemetry entries
            'timestamp': timestamp
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(results, f, indent=2)
            print(f"💾 Results saved to: {filename}")
            
            # Print corner summary
            print(f"\n📍 Corner Summary:")
            for corner in self.corner_positions:
                total_ticks = corner['left_ticks'] + corner['right_ticks']
                print(f"   Corner {corner['corner']}: L={corner['left_ticks']:4d}, R={corner['right_ticks']:4d}, Total={total_ticks:4d}")
            
        except Exception as e:
            print(f"❌ Failed to save results: {e}")
    
    def close(self):
        """Close serial connection"""
        if self.ser:
            self.ser.close()
            print("📴 Disconnected from robot")

def main():
    controller = None
    try:
        # Initialize controller
        controller = RectanglePathController(ros2_mode=True)
        
        # Get rectangle dimensions from user
        width, height = controller.get_rectangle_dimensions()
        if width is None:
            return
        
        # Execute rectangle path
        success = controller.execute_rectangle_path(width, height)
        
        if success:
            # Plot the path
            controller.plot_rectangle_path(width, height)
            
            # Save results
            controller.save_results(width, height)
            print("\n✅ Rectangle path mission accomplished!")
        else:
            print("\n❌ Rectangle path failed")
    
    except KeyboardInterrupt:
        print("\n⏹️  Path interrupted by user")
    except Exception as e:
        print(f"\n💥 Error: {e}")
    finally:
        if controller:
            controller.close()
        controller = None
        try:
            # Initialize controller
            import rclpy
            rclpy.init()
            from rclpy.node import Node
            class DummyNode(Node):
                def __init__(self):
                    super().__init__('rectangle_path_controller')
            ros2_node = DummyNode()
            controller = RectanglePathController(ros2_mode=True)
            controller.ros2_node = ros2_node
            # Get rectangle dimensions from user
            width, height = controller.get_rectangle_dimensions()
            if width is None:
                return
            # Execute rectangle path
            success = controller.execute_rectangle_path(width, height)
            if success:
                controller.plot_rectangle_path(width, height)
                controller.save_results(width, height)
                print("\n✅ Rectangle path mission accomplished!")
            else:
                print("\n❌ Rectangle path failed")
        except KeyboardInterrupt:
            print("\n⏹️  Path interrupted by user")
        except Exception as e:
            print(f"\n💥 Error: {e}")
        finally:
            if controller:
                controller.close()

if __name__ == '__main__':
    main()