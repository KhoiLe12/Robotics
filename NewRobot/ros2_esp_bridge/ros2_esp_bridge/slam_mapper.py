#!/usr/bin/env python3
"""
SLAM Mapper - Provides interface for SLAM Toolbox mapping
Saves and loads maps for navigation
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
import pickle
import os
from datetime import datetime


class SLAMMapper(Node):
    def __init__(self):
        super().__init__('slam_mapper')
        
        # Parameters
        self.declare_parameter('map_directory', os.path.expanduser('~/robot_maps'))
        self.map_directory = self.get_parameter('map_directory').value
        
        # Create map directory if it doesn't exist
        os.makedirs(self.map_directory, exist_ok=True)
        
        # Subscribe to map topic from SLAM Toolbox
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.current_map = None
        self.get_logger().info(f'SLAM Mapper ready - maps saved to {self.map_directory}')
    
    def map_callback(self, msg: OccupancyGrid):
        """Store latest map from SLAM Toolbox"""
        self.current_map = msg
        self.get_logger().debug(f'Map updated: {msg.info.width}x{msg.info.height}')
    
    def save_map(self, map_name=None):
        """Save current map to file"""
        if self.current_map is None:
            self.get_logger().warn('No map to save!')
            return False
        
        if map_name is None:
            map_name = f'map_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        
        # Save as pickle for easy loading
        map_path = os.path.join(self.map_directory, f'{map_name}.pkl')
        
        try:
            with open(map_path, 'wb') as f:
                pickle.dump(self.current_map, f)
            
            self.get_logger().info(f'Map saved: {map_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')
            return False
    
    def load_map(self, map_name):
        """Load a saved map"""
        map_path = os.path.join(self.map_directory, f'{map_name}.pkl')
        
        try:
            with open(map_path, 'rb') as f:
                self.current_map = pickle.load(f)
            
            self.get_logger().info(f'Map loaded: {map_path}')
            return self.current_map
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            return None
    
    def list_maps(self):
        """List all saved maps"""
        try:
            maps = [f.replace('.pkl', '') for f in os.listdir(self.map_directory) if f.endswith('.pkl')]
            return maps
        except Exception as e:
            self.get_logger().error(f'Failed to list maps: {e}')
            return []


def main(args=None):
    rclpy.init(args=args)
    mapper = SLAMMapper()
    
    print("\n" + "="*50)
    print("SLAM MAPPER")
    print("="*50)
    print("Commands:")
    print("  save <name>  - Save current map")
    print("  list         - List saved maps")
    print("  load <name>  - Load a map")
    print("  quit         - Exit")
    
    import threading
    
    def spin_thread():
        rclpy.spin(mapper)
    
    spinner = threading.Thread(target=spin_thread, daemon=True)
    spinner.start()
    
    try:
        while rclpy.ok():
            cmd = input("\nCommand: ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'quit':
                break
            elif cmd[0] == 'save':
                name = cmd[1] if len(cmd) > 1 else None
                mapper.save_map(name)
            elif cmd[0] == 'list':
                maps = mapper.list_maps()
                print(f"\nSaved maps ({len(maps)}):")
                for m in maps:
                    print(f"  - {m}")
            elif cmd[0] == 'load':
                if len(cmd) < 2:
                    print("Usage: load <map_name>")
                else:
                    mapper.load_map(cmd[1])
            else:
                print("Unknown command!")
    
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
