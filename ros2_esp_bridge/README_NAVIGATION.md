# Robot Navigation System

Complete autonomous navigation system with SLAM mapping, obstacle avoidance, and goal-reaching capabilities.

## Features

✅ **Teleoperation** - Manual control via keyboard  
✅ **SLAM Mapping** - Real-time map creation with SLAM Toolbox  
✅ **Autonomous Navigation** - Nav2-based path planning  
✅ **Obstacle Avoidance** - Dynamic obstacle detection and avoidance  
✅ **Goal Actions** - Performs action (360° spin) upon reaching goals  

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Navigation System                     │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐      ┌──────────────┐                │
│  │   Teleop     │─────▶│  cmd_vel     │                │
│  │  Keyboard    │      │   Bridge     │                │
│  └──────────────┘      └──────┬───────┘                │
│                                │                         │
│  ┌──────────────┐             │                         │
│  │    Nav2      │─────────────┘                         │
│  │  Navigator   │                                        │
│  └──────┬───────┘                                        │
│         │                 ┌──────────────┐              │
│         └────────────────▶│    ESP32     │              │
│                           │   Serial     │              │
│  ┌──────────────┐         │   Bridge     │              │
│  │    LiDAR     │         └──────┬───────┘              │
│  │  (RPLidar)   │                │                       │
│  └──────┬───────┘                ▼                       │
│         │                 ┌──────────────┐              │
│         │                 │    Robot     │              │
│         │                 │   Hardware   │              │
│         │                 └──────────────┘              │
│         │                                                │
│         ▼                                                │
│  ┌──────────────┐      ┌──────────────┐                │
│  │  SLAM        │─────▶│     Map      │                │
│  │  Toolbox     │      │              │                │
│  └──────────────┘      └──────────────┘                │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

## Installation

### 1. Install Dependencies

```bash
# Install ROS2 Humble packages
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rplidar-ros

# Install Python dependencies
pip3 install pyserial
```

### 2. Build the Package

```bash
cd ~/NewRobot
colcon build --packages-select ros2_esp_bridge
source install/setup.bash
```

## Hardware Setup

### Connections

1. **ESP32** - Connect to `/dev/ttyUSB0` (or update in launch file)
2. **RPLidar A1** - Connect to `/dev/ttyUSB1` (or update in launch file)

### LiDAR Orientation

The LiDAR is mounted pointing **LEFT** (90° offset from robot front). This is configured in the static transform:

```python
# In robot_navigation.launch.py
# x y z yaw pitch roll parent child
['0', '0', '0.1', '1.5708', '0', '0', 'base_link', 'laser_frame']
```

## Usage

### Phase 1: Mapping (SLAM)

Create a map of your environment using teleoperation.

#### Terminal 1: Launch Robot System

```bash
source install/setup.bash
ros2 launch ros2_esp_bridge robot_navigation.launch.py
```

This starts:
- ESP serial bridge (odometry)
- cmd_vel bridge (motor control)
- RPLidar driver
- SLAM Toolbox (mapping)

#### Terminal 2: Teleoperation

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Forward
- `k` - Stop
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `q/z` - Increase/decrease speed

Drive around your environment to build a complete map.

#### Terminal 3: Save the Map

```bash
source install/setup.bash
ros2 run ros2_esp_bridge slam_mapper

# Commands:
save my_map        # Save current map
list               # List saved maps
```

Or use SLAM Toolbox's built-in map saving:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: my_map}}"
```

### Phase 2: Navigation (Autonomous)

Navigate autonomously using the created map.

#### Terminal 1: Launch Robot with Nav2

```bash
source install/setup.bash

# First, launch the base system
ros2 launch ros2_esp_bridge robot_navigation.launch.py use_slam:=false

# In another terminal, launch Nav2 with your map
ros2 launch nav2_bringup localization_launch.py map:=/path/to/my_map.yaml

# In another terminal, launch Nav2 navigation
ros2 launch nav2_bringup navigation_launch.py
```

#### Terminal 2: Set Goals

Option A: **Command Line Interface**

```bash
source install/setup.bash
ros2 run ros2_esp_bridge goal_navigator

# Commands:
goal 1.0 0.5 90    # Navigate to (1.0, 0.5) facing 90°
goal 2.0 1.0       # Navigate to (2.0, 1.0) facing 0°
cancel             # Cancel current goal
clear              # Clear goal queue
```

Option B: **RViz2 Interactive**

```bash
ros2 run rviz2 rviz2

# In RViz:
# 1. Add -> By display type -> Map
# 2. Add -> By display type -> LaserScan
# 3. Use "2D Pose Estimate" to set initial position
# 4. Use "Nav2 Goal" to set navigation goals
```

#### Terminal 3: Monitor Status

```bash
# Monitor robot position
ros2 topic echo /odom

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Monitor obstacles
ros2 topic echo /scan
```

## Goal Actions

When the robot reaches a goal, it performs a **360° spin** by default.

To customize the action, modify `goal_navigator.py`:

```python
def perform_goal_action(self):
    """Perform action upon reaching goal"""
    if self.goal_action == 'spin':
        self.spin_360()
    elif self.goal_action == 'release_candy':
        self.release_candy()  # Implement this
    elif self.goal_action == 'play_sound':
        self.play_sound()      # Implement this
```

## Configuration Files

### Robot Parameters

Edit `robot_navigation.launch.py` to adjust:

```python
parameters=[{
    'wheel_radius': 0.035,      # meters (3.5cm)
    'wheelbase': 0.24,          # meters (24cm)
    'ticks_per_rev': 231,       # encoder ticks per revolution
    'max_linear_speed': 0.3,    # m/s
    'max_angular_speed': 1.0,   # rad/s
}]
```

### Serial Ports

If your ESP32 or LiDAR use different ports:

```bash
ros2 launch ros2_esp_bridge robot_navigation.launch.py \
    serial_port:=/dev/ttyUSB2
```

Edit `robot_navigation.launch.py` for LiDAR port:

```python
lidar_node = Node(
    parameters=[{
        'serial_port': '/dev/ttyUSB1',  # Change this
```

## Troubleshooting

### Robot doesn't move

1. Check serial connection: `ls /dev/ttyUSB*`
2. Verify ESP32 is in PID mode (check serial monitor)
3. Test cmd_vel: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"`

### No LiDAR data

1. Check connection: `ls /dev/ttyUSB*`
2. Test LiDAR: `ros2 topic echo /scan`
3. Check permissions: `sudo chmod 666 /dev/ttyUSB1`

### SLAM not creating map

1. Verify LiDAR data: `ros2 topic echo /scan`
2. Check odometry: `ros2 topic echo /odom`
3. Ensure transforms are published: `ros2 run tf2_ros tf2_echo base_link laser_frame`

### Navigation fails

1. Set initial pose in RViz2 using "2D Pose Estimate"
2. Check costmaps: `ros2 topic echo /local_costmap/costmap`
3. Verify obstacles are detected: `ros2 topic echo /scan`

### Goal action doesn't execute

1. Check if goal was reached: `ros2 topic echo /navigate_to_pose/_action/status`
2. Monitor cmd_vel during spin: `ros2 topic echo /cmd_vel`

## File Structure

```
ros2_esp_bridge/
├── launch/
│   ├── robot_navigation.launch.py  # Main system launch
│   ├── teleop.launch.py            # Teleoperation only
│   └── esp_with_lidar.launch.py    # Legacy launch
├── ros2_esp_bridge/
│   ├── esp_serial_bridge.py        # ESP32 communication + odometry
│   ├── cmd_vel_bridge.py           # cmd_vel → ESP serial
│   ├── slam_mapper.py              # Map save/load utility
│   ├── goal_navigator.py           # Autonomous navigation
│   ├── a_star.py                   # A* path planner (legacy)
│   ├── localization_comparison.py  # Localization testing (legacy)
│   └── rectangle_path.py           # Rectangle path (legacy)
└── README_NAVIGATION.md            # This file
```

## Testing Checklist

- [ ] ESP32 connects and publishes odometry
- [ ] LiDAR publishes scan data
- [ ] Teleop controls robot movement
- [ ] SLAM creates a map while driving
- [ ] Map can be saved and loaded
- [ ] Nav2 accepts navigation goals
- [ ] Robot avoids obstacles
- [ ] Robot reaches goals accurately
- [ ] Goal action (spin) executes correctly

## Next Steps

1. **Tune Nav2 parameters** for better obstacle avoidance
2. **Add custom goal actions** (release candy, play sound, etc.)
3. **Create predefined waypoint missions**
4. **Add battery monitoring integration**
5. **Implement multi-goal patrol missions**

## Support

For issues or questions:
1. Check ROS2 logs: `ros2 topic list`, `ros2 node list`
2. Enable debug output: Add `output='screen'` to nodes
3. Monitor serial: `screen /dev/ttyUSB0 115200`
