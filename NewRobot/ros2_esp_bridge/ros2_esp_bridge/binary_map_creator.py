#!/usr/bin/env python3
"""
Binary Map Creator - Records and fuses LIDAR maps into 30x30 binary grids
WITH LiDAR Yaw Offset for RPLidar A1 pointing left
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import pickle
from datetime import datetime

class BinaryMapCreator(Node):
    def __init__(self):
        super().__init__('binary_map_creator')
        
        # Fixed 30x30 grid parameters
        self.grid_size = 30
        self.map_resolution = 0.1  # 10cm per cell
        self.max_lidar_range = 3.0
        
        # LiDAR configuration for RPLidar A1 pointing LEFT
        self.lidar_yaw_offset = np.pi / 2  # 90 degrees counterclockwise (pointing left)
        
        # Map storage
        self.individual_maps = []
        self.final_binary_map = None
        
        # Current state
        self.current_scan = None
        self.is_recording = False
        self.recorded_scans = []
        
        # Subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        
        self._initialize_grid()
        self.get_logger().info("🗺️ Binary Map Creator Ready")
        self.get_logger().info(f"LiDAR Yaw Offset: {np.degrees(self.lidar_yaw_offset):.1f}° (pointing left)")
    
    def _initialize_grid(self):
        """Initialize an empty 30x30 grid"""
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.grid_origin = np.array([-1.5, -1.5])
    
    def lidar_callback(self, msg):
        """Process incoming LIDAR data"""
        self.current_scan = msg
        if self.is_recording:
            self.recorded_scans.append({
                'scan': msg,
                'timestamp': time.time(),
                'pose': self.get_robot_pose()
            })
    
    def get_robot_pose(self):
        """Get current robot pose"""
        return Pose2D(x=0.0, y=0.0, theta=0.0)
    
    def apply_lidar_offset(self, angle, robot_theta):
        """Apply LiDAR yaw offset to correct for LiDAR pointing left"""
        # LiDAR is mounted pointing LEFT (90° counterclockwise from robot front)
        # So we need to add 90° to each LiDAR reading to align with robot frame
        return angle + robot_theta + self.lidar_yaw_offset
    
    def scan_to_grid(self, scan, robot_pose):
        """Convert LIDAR scan to grid occupancy probabilities WITH LiDAR offset"""
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        
        if scan is None:
            return grid
        
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        
        for i, (angle, range_val) in enumerate(zip(angles, scan.ranges)):
            if (scan.range_min <= range_val <= self.max_lidar_range and 
                not np.isinf(range_val) and not np.isnan(range_val)):
                
                # Apply LiDAR yaw offset to correct for mounting orientation
                corrected_angle = self.apply_lidar_offset(angle, robot_pose.theta)
                
                # Calculate world coordinates with corrected angle
                world_x = robot_pose.x + range_val * np.cos(corrected_angle)
                world_y = robot_pose.y + range_val * np.sin(corrected_angle)
                
                grid_x, grid_y = self.world_to_grid(world_x, world_y)
                
                if self.is_within_grid(grid_x, grid_y):
                    grid[grid_y, grid_x] = 1.0
                    self.mark_free_cells(grid, robot_pose, world_x, world_y, corrected_angle)
        
        return grid
    
    def mark_free_cells(self, grid, robot_pose, end_x, end_y, beam_angle):
        """Mark cells along the LIDAR beam as free space"""
        start_x, start_y = robot_pose.x, robot_pose.y
        grid_start_x, grid_start_y = self.world_to_grid(start_x, start_y)
        grid_end_x, grid_end_y = self.world_to_grid(end_x, end_y)
        
        points = self.get_line_points(grid_start_x, grid_start_y, grid_end_x, grid_end_y)
        for px, py in points:
            if self.is_within_grid(px, py) and grid[py, px] == 0:
                grid[py, px] = 0.1
    
    def get_line_points(self, x0, y0, x1, y1):
        """Get grid points between two points"""
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points
    
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_x - self.grid_origin[0]) / self.map_resolution)
        grid_y = int((world_y - self.grid_origin[1]) / self.map_resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.map_resolution + self.grid_origin[0]
        world_y = grid_y * self.map_resolution + self.grid_origin[1]
        return world_x, world_y
    
    def is_within_grid(self, grid_x, grid_y):
        """Check if grid coordinates are within bounds"""
        return (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size)
    
    def record_map(self, map_name=None, duration=20):
        """Record a map for specified duration"""
        if map_name is None:
            map_name = f"map_{len(self.individual_maps) + 1}"
        
        self.get_logger().info(f"🎥 Recording {map_name} for {duration} seconds")
        self.get_logger().info("LiDAR offset active: 90° counterclockwise (pointing left)")
        
        self.is_recording = True
        self.recorded_scans = []
        
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            time.sleep(0.1)
        
        self.is_recording = False
        
        probability_grid = self.process_recorded_scans()
        individual_map = {
            'name': map_name,
            'probability_grid': probability_grid,
            'timestamp': datetime.now().isoformat(),
            'num_scans': len(self.recorded_scans),
            'lidar_yaw_offset': self.lidar_yaw_offset  # Store offset in map data
        }
        
        self.individual_maps.append(individual_map)
        self.get_logger().info(f"✅ Map recorded: {map_name}")
        return probability_grid
    
    def process_recorded_scans(self):
        """Combine multiple scans into a probability grid"""
        combined_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        scan_count = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        for scan_data in self.recorded_scans:
            scan_grid = self.scan_to_grid(scan_data['scan'], scan_data['pose'])
            combined_grid += scan_grid
            scan_count[scan_grid > 0] += 1
        
        avg_grid = np.zeros_like(combined_grid)
        mask = scan_count > 0
        avg_grid[mask] = combined_grid[mask] / scan_count[mask]
        return avg_grid
    
    def create_final_binary_map(self, occupancy_threshold=0.3):
        """Combine all maps and create final binary map"""
        if len(self.individual_maps) < 1:
            self.get_logger().warn("No maps to combine!")
            return None
        
        self.get_logger().info(f"🔄 Combining {len(self.individual_maps)} maps...")
        
        combined_probability = self.individual_maps[0]['probability_grid'].copy()
        for i in range(1, len(self.individual_maps)):
            combined_probability += self.individual_maps[i]['probability_grid']
        
        combined_probability /= len(self.individual_maps)
        self.final_binary_map = (combined_probability >= occupancy_threshold).astype(np.uint8)
        
        self.get_logger().info("✅ Final binary map created!")
        return self.final_binary_map
    
    def visualize_map(self, binary_map, title="30×30 Binary Occupancy Grid"):
        """Visualize the binary map"""
        plt.figure(figsize=(8, 8))
        plt.imshow(binary_map, cmap='binary', origin='lower', 
                  extent=[self.grid_origin[0], 
                         self.grid_origin[0] + self.grid_size * self.map_resolution,
                         self.grid_origin[1],
                         self.grid_origin[1] + self.grid_size * self.map_resolution])
        
        # Add coordinate system indicator to show LiDAR offset
        plt.arrow(0, 0, 0.2, 0, head_width=0.05, head_length=0.05, fc='red', ec='red', label='Robot Front')
        plt.arrow(0, 0, 0, 0.2, head_width=0.05, head_length=0.05, fc='blue', ec='blue', label='LiDAR Front')
        
        plt.title(title, fontsize=14, fontweight='bold')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        plt.gca().set_xticks(np.arange(self.grid_origin[0], 
                                     self.grid_origin[0] + self.grid_size * self.map_resolution + 0.1, 
                                     self.map_resolution))
        plt.gca().set_yticks(np.arange(self.grid_origin[1],
                                     self.grid_origin[1] + self.grid_size * self.map_resolution + 0.1,
                                     self.map_resolution))
        plt.gca().grid(True, which='both', alpha=0.2)
        
        plt.tight_layout()
        plt.show()
    
    def save_binary_map(self, filename=None):
        """Save the binary map to file"""
        if self.final_binary_map is None:
            self.get_logger().warn("No binary map to save!")
            return False
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"binary_map_30x30_{timestamp}"
        
        # Save as pickle
        map_data = {
            'binary_map': self.final_binary_map,
            'resolution': self.map_resolution,
            'origin': self.grid_origin,
            'grid_size': self.grid_size,
            'lidar_yaw_offset': self.lidar_yaw_offset,  # Save offset for reference
            'timestamp': datetime.now().isoformat()
        }
        
        with open(f"{filename}.pkl", 'wb') as f:
            pickle.dump(map_data, f)
        
        # Save as image
        plt.figure(figsize=(8, 8))
        plt.imshow(self.final_binary_map, cmap='binary', origin='lower')
        
        # Add orientation indicators
        plt.arrow(15, 15, 3, 0, head_width=1, head_length=1, fc='red', ec='red')
        plt.arrow(15, 15, 0, 3, head_width=1, head_length=1, fc='blue', ec='blue')
        plt.text(20, 15, 'Robot Front', color='red', fontsize=10)
        plt.text(15, 20, 'LiDAR Front', color='blue', fontsize=10)
        
        plt.title('30×30 Binary Occupancy Grid\n(LiDAR pointing left)')
        plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f"💾 Map saved as {filename}.pkl")
        return True

def main():
    rclpy.init()
    creator = BinaryMapCreator()
    
    def spin_node():
        rclpy.spin(creator)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        print("\n" + "="*50)
        print("🗺️  BINARY MAP CREATOR")
        print("="*50)
        print(f"Grid: 30×30 cells, Resolution: {creator.map_resolution}m/cell")
        print(f"LiDAR Configuration: RPLidar A1 pointing LEFT (90° offset)")
        
        while rclpy.ok():
            print("\nOptions:")
            print("1. Record new map (20 seconds)")
            print("2. Combine all maps → Create binary map")
            print("3. Visualize current binary map")
            print("4. Save binary map")
            print("5. Exit")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == '1':
                map_name = input("Enter map name: ").strip() or None
                creator.record_map(map_name, duration=20)
                
            elif choice == '2':
                if len(creator.individual_maps) >= 1:
                    binary_map = creator.create_final_binary_map()
                    if binary_map is not None:
                        print("✅ Binary map created!")
                        creator.visualize_map(binary_map)
                else:
                    print("❌ Record at least one map first!")
                    
            elif choice == '3':
                if creator.final_binary_map is not None:
                    creator.visualize_map(creator.final_binary_map)
                else:
                    print("❌ Create binary map first!")
                    
            elif choice == '4':
                if creator.final_binary_map is not None:
                    filename = input("Enter filename: ").strip()
                    creator.save_binary_map(filename)
                else:
                    print("❌ No binary map to save!")
                    
            elif choice == '5':
                print("👋 Exiting...")
                break
                
            else:
                print("❌ Invalid choice!")
                
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        creator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()