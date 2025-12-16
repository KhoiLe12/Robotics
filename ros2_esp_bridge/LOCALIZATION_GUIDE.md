# Robot Localization and Navigation Guide

## Setup and Launch

### 1. Rebuild the Package
After updating the code, rebuild on your Pi:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_esp_bridge
source install/setup.bash
```

### 2. Launch Localization and Navigation
```bash
ros2 launch ros2_esp_bridge robot_localization_nav.launch.py map:=/home/password/maps/my_map.yaml
```

## Setting Initial Pose in RViz

AMCL needs an initial pose estimate to create the `map → odom` transform. Without it, you'll see errors about missing transforms.

### Steps:
1. **Open RViz** (on your laptop/desktop):
   ```bash
   ros2 run rviz2 rviz2
   ```

2. **Configure RViz displays**:
   - Add `Map` display → Topic: `/map`
   - Add `LaserScan` display → Topic: `/scan`
   - Add `RobotModel` display
   - Add `TF` display
   - Add `PoseArray` display → Topic: `/particlecloud` (AMCL particles)
   - Set Fixed Frame to `map`

3. **Set Initial Pose**:
   - Click the **"2D Pose Estimate"** button in the RViz toolbar
   - Click on the map where your robot actually is
   - Drag to set the orientation (direction robot is facing)
   
   This publishes to `/initialpose` and AMCL will:
   - Create the `map → odom` transform
   - Initialize particle filter around that pose
   - Start localizing using laser scans

4. **Verify**:
   - The particle cloud should appear around your robot
   - The laser scan should align with map walls
   - The robot model should appear correctly positioned
   - TF tree should show: `map → odom → base_footprint → base_link → laser_frame`

## Navigation

Once localized (after setting initial pose):

1. **Send Navigation Goal**:
   - Click **"2D Nav Goal"** button in RViz
   - Click destination on map and drag for orientation
   
2. **Monitor Progress**:
   - Green path shows planned route
   - Red local costmap shows obstacles
   - Robot should navigate autonomously

## Troubleshooting

### No map visible
- Check map file path is correct
- Verify map_server is running: `ros2 node list | grep map_server`
- Check topic: `ros2 topic echo /map --once`

### Transform errors
- **Before initial pose**: Normal, set initial pose in RViz
- **After initial pose**: Check TF tree: `ros2 run tf2_tools view_frames`

### Robot not localizing well
- Adjust AMCL parameters in nav2_params.yaml:
  - Increase `max_particles` (default: 2000)
  - Decrease `min_particles` (default: 500)
  - Adjust `laser_model_type` if needed

### Navigation fails
- Check costmaps are updating: `ros2 topic hz /local_costmap/costmap`
- Verify cmd_vel is published: `ros2 topic echo /cmd_vel`
- Check ESP32 is receiving commands (LED should blink)

## Key Topics

- `/map` - Static map from map_server
- `/scan` - LiDAR data
- `/odom` - Odometry from ESP32
- `/cmd_vel` - Velocity commands to robot
- `/initialpose` - Set robot's initial pose
- `/goal_pose` - Navigation goal
- `/particlecloud` - AMCL particle filter visualization

## TF Tree Structure

```
map (from AMCL)
 └── odom (from ESP32 odometry)
      └── base_footprint (from ESP32 odometry)
           └── base_link (static TF)
                └── laser_frame (static TF)
```
