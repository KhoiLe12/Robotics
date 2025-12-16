# Quick Start Guide

## Step 1: Build the Package

```bash
cd ~/NewRobot
colcon build --packages-select ros2_esp_bridge
source install/setup.bash
```

## Step 2: Upload Firmware to ESP32

Make sure your ESP32 has the latest firmware with PID velocity control and WiFi support.

```bash
cd ~/NewRobot
pio run --target upload
```

## Step 3: Create a Map (One-Time Setup)

### Terminal 1: Launch Robot System
```bash
source install/setup.bash
ros2 launch ros2_esp_bridge robot_navigation.launch.py
```

### Terminal 2: Launch Teleop
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive around your environment using keyboard controls:
- `i` = forward
- `k` = stop
- `,` = backward  
- `j` = turn left
- `l` = turn right

### Terminal 3: Save Map
```bash
# Wait until you've driven around the entire area, then:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_room'}}"
```

The map will be saved to `~/.ros/my_room.yaml` and `~/.ros/my_room.pgm`

## Step 4: Navigate Autonomously

### Terminal 1: Launch Robot
```bash
source install/setup.bash
ros2 launch ros2_esp_bridge robot_navigation.launch.py use_slam:=false
```

### Terminal 2: Launch Nav2 with Your Map
```bash
source install/setup.bash
ros2 launch nav2_bringup localization_launch.py map:=$HOME/.ros/my_room.yaml
```

### Terminal 3: Launch Nav2 Navigation
```bash
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py params_file:=$HOME/NewRobot/ros2_esp_bridge/config/nav2_params.yaml
```

### Terminal 4: Launch RViz2
```bash
source install/setup.bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set Fixed Frame to `map`
2. Add -> Map (topic: `/map`)
3. Add -> LaserScan (topic: `/scan`)
4. Add -> RobotModel
5. Use "2D Pose Estimate" button to set initial robot position
6. Use "Nav2 Goal" button to send navigation goals

The robot will:
- Navigate to the goal
- Avoid obstacles (including dynamic ones)
- Spin 360° when it reaches the goal

## Troubleshooting

**Robot doesn't move:**
```bash
# Check if cmd_vel_bridge is receiving commands
ros2 topic echo /cmd_vel

# Test manual command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
```

**No map in RViz:**
```bash
# Check if map is published
ros2 topic echo /map --once
```

**Navigation fails:**
```bash
# Check if costmaps are working
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
```

## Common Commands

**Stop everything:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

**Check node status:**
```bash
ros2 node list
ros2 topic list
```

**Monitor odometry:**
```bash
ros2 topic echo /odom
```

**Monitor LiDAR:**
```bash
ros2 topic echo /scan
```
