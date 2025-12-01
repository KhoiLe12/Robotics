#!/usr/bin/env python3
"""
A* Path Planner - Loads binary maps and performs path planning
WITH LiDAR orientation information
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

class AStarPlanner:
    def __init__(self):
        self.grid_size = 30
        self.map_resolution = 0.1
        self.grid_origin = np.array([-1.5, -1.5])
        self.loaded_map = None
        self.lidar_yaw_offset = np.pi / 2  # Same offset for reference
    
    def load_binary_map(self, filename):
        """Load a binary map from pickle file"""
        try:
            with open(filename, 'rb') as f:
                map_data = pickle.load(f)
            
            self.loaded_map = map_data['binary_map']
            self.map_resolution = map_data['resolution']
            self.grid_origin = map_data['origin']
            self.grid_size = map_data['grid_size']
            self.lidar_yaw_offset = map_data.get('lidar_yaw_offset', np.pi / 2)
            
            print(f"✅ Map loaded: {filename}")
            print(f"   Grid: {self.grid_size}x{self.grid_size}")
            print(f"   Resolution: {self.map_resolution}m/cell")
            print(f"   LiDAR Offset: {np.degrees(self.lidar_yaw_offset):.1f}°")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load map: {e}")
            return False
    
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_x - self.grid_origin[0]) / self.map_resolution)
        grid_y = int((world_y - self.grid_origin[1]) / self.map_resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = (grid_x+0.5) * self.map_resolution + self.grid_origin[0]
        world_y = (grid_y+0.5) * self.map_resolution + self.grid_origin[1]
        return world_x, world_y
    
    def is_within_grid(self, grid_x, grid_y):
        """Check if grid coordinates are within bounds"""
        return (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size)
    
    def a_star_path_planning(self, start_grid, goal_grid):
        """A* path planning on the binary grid map"""
        if self.loaded_map is None:
            print("❌ No map loaded!")
            return None
        
        if not self.is_within_grid(start_grid[0], start_grid[1]) or not self.is_within_grid(goal_grid[0], goal_grid[1]):
            print("❌ Start or goal outside grid bounds!")
            return None
        
        if self.loaded_map[start_grid[1], start_grid[0]] == 1:
            print("❌ Start position is occupied!")
            return None
            
        if self.loaded_map[goal_grid[1], goal_grid[0]] == 1:
            print("❌ Goal position is occupied!")
            return None
        
        # A* algorithm
        open_set = PriorityQueue()
        open_set.put((0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        
        open_set_hash = {start_grid}
        
        while not open_set.empty():
            current = open_set.get()[1]
            open_set_hash.remove(current)
            
            if current == goal_grid:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    
                    if neighbor not in open_set_hash:
                        open_set_hash.add(neighbor)
                        open_set.put((f_score[neighbor], neighbor))
        
        print("❌ No path found!")
        return None
    
    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def distance(self, a, b):
        """Distance between two grid points"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, point):
        """Get valid neighboring points (8-connected)"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                    
                neighbor = (point[0] + dx, point[1] + dy)
                
                if (self.is_within_grid(neighbor[0], neighbor[1]) and 
                    self.loaded_map[neighbor[1], neighbor[0]] == 0):
                    neighbors.append(neighbor)
        
        return neighbors
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def visualize_path(self, path, start_grid, goal_grid):
        """Visualize the map with the planned path - WITH CLEAR GRID LINES"""
        if self.loaded_map is None:
            print("❌ No map loaded!")
            return
        
        plt.figure(figsize=(12, 10))
        
        # Create a custom colormap: Black=free, White=obstacle, with clear contrast
        from matplotlib.colors import ListedColormap
        cmap = ListedColormap(['black', 'white'])  # 0=black (free), 1=white (obstacle)
        
        # Plot the binary map with higher contrast
        plt.imshow(self.loaded_map, cmap=cmap, origin='lower', 
                extent=[self.grid_origin[0], 
                        self.grid_origin[0] + self.grid_size * self.map_resolution,
                        self.grid_origin[1],
                        self.grid_origin[1] + self.grid_size * self.map_resolution],
                alpha=0.8)  # Slightly transparent to see grid lines
        
        # Plot path with bold styling
        if path is not None:
            path_x = [self.grid_to_world(x, y)[0] for x, y in path]
            path_y = [self.grid_to_world(x, y)[1] for x, y in path]
            plt.plot(path_x, path_y, 'r-', linewidth=4, label='A* Path', alpha=0.9)
            # Plot path points as dots
            plt.plot(path_x, path_y, 'ro', markersize=4, alpha=0.6)
        
        # Plot start and goal with clear styling
        start_world = self.grid_to_world(start_grid[0], start_grid[1])
        goal_world = self.grid_to_world(goal_grid[0], goal_grid[1])
        
        plt.plot(start_world[0], start_world[1], 'go', markersize=15, 
                label='Start', markeredgecolor='darkgreen', markeredgewidth=2)
        plt.plot(goal_world[0], goal_world[1], 'bo', markersize=15, 
                label='Goal', markeredgecolor='darkblue', markeredgewidth=2)
        
        # Add orientation indicators
        center_x = (self.grid_origin[0] + self.grid_origin[0] + self.grid_size * self.map_resolution) / 2
        center_y = (self.grid_origin[1] + self.grid_origin[1] + self.grid_size * self.map_resolution) / 2
        
        plt.arrow(center_x, center_y, 0.2, 0, head_width=0.05, head_length=0.05, 
                fc='red', ec='red', linewidth=2, label='Robot Front')
        plt.arrow(center_x, center_y, 0, 0.2, head_width=0.05, head_length=0.05, 
                fc='blue', ec='blue', linewidth=2, label='LiDAR Front')
        
        # ADD CLEAR GRID LINES
        # Major grid lines (every cell)
        for i in range(self.grid_size + 1):
            x_pos = self.grid_origin[0] + i * self.map_resolution
            y_pos = self.grid_origin[1] + i * self.map_resolution
            
            # Vertical lines
            plt.axvline(x=x_pos, color='gray', linewidth=0.5, alpha=0.7, linestyle='-')
            # Horizontal lines  
            plt.axhline(y=y_pos, color='gray', linewidth=0.5, alpha=0.7, linestyle='-')
        
        # Every 5th grid line should be bolder
        for i in range(0, self.grid_size + 1, 5):
            x_pos = self.grid_origin[0] + i * self.map_resolution
            y_pos = self.grid_origin[1] + i * self.map_resolution
            
            plt.axvline(x=x_pos, color='darkgray', linewidth=1.5, alpha=0.8)
            plt.axhline(y=y_pos, color='darkgray', linewidth=1.5, alpha=0.8)
        
        # Add coordinate labels every 5 cells
        for i in range(0, self.grid_size, 5):
            x_pos = self.grid_origin[0] + i * self.map_resolution + self.map_resolution / 2
            y_pos = self.grid_origin[1] + i * self.map_resolution + self.map_resolution / 2
            
            plt.text(self.grid_origin[0] - 0.1, y_pos, f'{i}', 
                    ha='right', va='center', fontsize=8, color='darkred', alpha=0.8)
            plt.text(x_pos, self.grid_origin[1] - 0.1, f'{i}', 
                    ha='center', va='top', fontsize=8, color='darkred', alpha=0.8)
        
        plt.title('A* Path Planning on 30×30 Binary Grid\n(LiDAR pointing left)', 
                fontsize=14, fontweight='bold', pad=20)
        plt.xlabel('X (meters)', fontsize=12, fontweight='bold')
        plt.ylabel('Y (meters)', fontsize=12, fontweight='bold')
        
        # Add a legend with better positioning
        plt.legend(fontsize=10, loc='upper right', framealpha=0.9, 
                facecolor='white', edgecolor='black')
        
        # Set equal aspect ratio
        plt.gca().set_aspect('equal')
        
        # Add a border around the grid
        from matplotlib.patches import Rectangle
        border = Rectangle((self.grid_origin[0], self.grid_origin[1]), 
                        self.grid_size * self.map_resolution, 
                        self.grid_size * self.map_resolution,
                        fill=False, edgecolor='black', linewidth=2)
        plt.gca().add_patch(border)
        
        plt.tight_layout()
        plt.show()
    
    def print_path_coordinates(self, path):
        """Print path coordinates in both grid and world coordinates"""
        if not path:
            return
        
        print("\n📍 Path Coordinates:")
        print("Step | Grid (x,y) | World (x,y)")
        print("-" * 35)
        for i, (grid_x, grid_y) in enumerate(path):
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            print(f"{i:4d} | ({grid_x:2d}, {grid_y:2d})   | ({world_x:5.2f}, {world_y:5.2f})")

def main():
    planner = AStarPlanner()
    
    print("\n" + "="*50)
    print("🧭 A* PATH PLANNER")
    print("="*50)
    print("Note: Maps are created with LiDAR pointing LEFT (90° offset)")
    
    while True:
        print("\nOptions:")
        print("1. Load binary map file")
        print("2. Plan path")
        print("3. Exit")
        
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == '1':
            filename = input("Enter map filename (.pkl): ").strip()
            if not filename.endswith('.pkl'):
                filename += '.pkl'
            planner.load_binary_map(filename)
            
        elif choice == '2':
            if planner.loaded_map is None:
                print("❌ Load a map first!")
                continue
            
            print("\n🎯 Path Planning")
            print("Enter grid coordinates (0-29 for both x and y)")
            
            try:
                start_x = int(input("Start X (0-29): "))
                start_y = int(input("Start Y (0-29): "))
                goal_x = int(input("Goal X (0-29): "))
                goal_y = int(input("Goal Y (0-29): "))
                
                start_grid = (start_x, start_y)
                goal_grid = (goal_x, goal_y)
                
                path = planner.a_star_path_planning(start_grid, goal_grid)
                
                if path:
                    print(f"✅ Path found with {len(path)} steps!")
                    planner.visualize_path(path, start_grid, goal_grid)
                    planner.print_path_coordinates(path)
                else:
                    print("❌ No path found!")
                    
            except ValueError:
                print("❌ Please enter valid integers!")
                
        elif choice == '3':
            print("👋 Exiting...")
            break
            
        else:
            print("❌ Invalid choice!")

if __name__ == '__main__':
    main()