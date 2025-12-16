#!/usr/bin/env python3
"""
System Visualization - Shows the data flow in the navigation system
"""

print("""
╔══════════════════════════════════════════════════════════════════════╗
║              ROBOT NAVIGATION SYSTEM ARCHITECTURE                    ║
╚══════════════════════════════════════════════════════════════════════╝

┌─────────────────────────────────────────────────────────────────────┐
│                        MAPPING PHASE                                 │
└─────────────────────────────────────────────────────────────────────┘

    ┌──────────────┐
    │   Keyboard   │
    │    Input     │
    └──────┬───────┘
           │
           ▼
    ┌──────────────────┐
    │  teleop_twist    │
    │    _keyboard     │
    └──────┬───────────┘
           │
           ▼ /cmd_vel
    ┌──────────────────┐      ┌──────────────┐
    │  cmd_vel_bridge  │─────▶│    ESP32     │
    │                  │      │   (Serial)   │
    └──────────────────┘      └──────┬───────┘
                                     │
                                     ▼
                              ┌──────────────┐
                              │    Motors    │
                              │  + Encoders  │
                              └──────┬───────┘
                                     │
                                     ▼ Odometry
    ┌──────────────────┐      ┌──────────────┐
    │  esp_serial_     │◀─────│   Encoders   │
    │     bridge       │      │              │
    └──────┬───────────┘      └──────────────┘
           │
           ▼ /odom
    ┌──────────────────┐      ┌──────────────┐
    │  SLAM Toolbox    │◀─────│   RPLidar    │ /scan
    │  (Mapping Mode)  │      │              │
    └──────┬───────────┘      └──────────────┘
           │
           ▼ /map
    ┌──────────────────┐
    │   Save Map       │
    │  (map.yaml)      │
    └──────────────────┘


┌─────────────────────────────────────────────────────────────────────┐
│                      NAVIGATION PHASE                                │
└─────────────────────────────────────────────────────────────────────┘

    ┌──────────────────┐
    │   RViz2 or       │
    │  goal_navigator  │
    └──────┬───────────┘
           │
           ▼ Goal Pose
    ┌──────────────────────────────────────┐
    │            Nav2 Stack                 │
    │  ┌────────────────────────────────┐  │
    │  │  1. Localization (AMCL)        │  │
    │  │     - Uses saved map + /odom   │  │
    │  │     - Estimates robot position │  │
    │  └────────────────────────────────┘  │
    │                                       │
    │  ┌────────────────────────────────┐  │
    │  │  2. Global Planner             │  │
    │  │     - Creates path on map      │  │
    │  │     - Avoids static obstacles  │  │
    │  └────────────────────────────────┘  │
    │                                       │
    │  ┌────────────────────────────────┐  │
    │  │  3. Local Planner (DWB)        │  │
    │  │     - Real-time path following │  │
    │  │     - Avoids dynamic obstacles │  │
    │  │     - Generates velocities     │  │
    │  └────────────────────────────────┘  │
    └───────────────┬───────────────────────┘
                    │
                    ▼ /cmd_vel
    ┌──────────────────────────────────────┐
    │         cmd_vel_bridge                │
    │  Converts to wheel velocities        │
    │  PID_VEL,left_mps,right_mps          │
    └───────────────┬───────────────────────┘
                    │
                    ▼ Serial
    ┌──────────────────────────────────────┐
    │              ESP32                    │
    │  ┌────────────────────────────────┐  │
    │  │  PID Velocity Controller       │  │
    │  │  - Reads encoder feedback      │  │
    │  │  - Computes PWM for each wheel │  │
    │  │  - Maintains desired speeds    │  │
    │  └────────────────────────────────┘  │
    └───────────────┬───────────────────────┘
                    │
                    ▼
    ┌──────────────────────────────────────┐
    │         Motor Hardware                │
    │  - L298N Motor Driver                │
    │  - Quadrature Encoders               │
    │  - Differential Drive Wheels         │
    └───────────────┬───────────────────────┘
                    │
                    ▼ Encoder Counts
    ┌──────────────────────────────────────┐
    │       esp_serial_bridge              │
    │  - Reads encoder counts              │
    │  - Computes odometry                 │
    │  - Publishes /odom + TF              │
    └───────────────┬───────────────────────┘
                    │
                    ▼
              Back to Nav2 Loop


┌─────────────────────────────────────────────────────────────────────┐
│                       GOAL ACTION                                    │
└─────────────────────────────────────────────────────────────────────┘

    When Goal Reached:
    
    ┌──────────────────┐
    │  goal_navigator  │
    │  .goal_reached() │
    └──────┬───────────┘
           │
           ▼
    ┌──────────────────┐
    │  Spin 360°       │
    │  - Publish twist │
    │  - angular.z=1.0 │
    │  - Duration: 2π  │
    └──────┬───────────┘
           │
           ▼
    ┌──────────────────┐
    │  Next Goal       │
    │  (if queued)     │
    └──────────────────┘


┌─────────────────────────────────────────────────────────────────────┐
│                    KEY TOPICS & FRAMES                               │
└─────────────────────────────────────────────────────────────────────┘

Topics:
  /cmd_vel          - Velocity commands (Twist)
  /odom             - Odometry (nav_msgs/Odometry)
  /scan             - LiDAR data (sensor_msgs/LaserScan)
  /map              - Occupancy grid map
  /local_costmap    - Local obstacle map
  /global_costmap   - Global obstacle map
  /navigate_to_pose - Navigation action

TF Frames:
  map               - Global map frame
  odom              - Odometry frame
  base_link         - Robot center
  laser_frame       - LiDAR sensor (90° left of base_link)


┌─────────────────────────────────────────────────────────────────────┐
│                      DATA FLOW SUMMARY                               │
└─────────────────────────────────────────────────────────────────────┘

MAPPING:
  Keyboard → Twist → Motors → Movement → Encoders → Odom → SLAM → Map

NAVIGATION:
  Goal → Planner → Twist → Motors → Movement → Encoders → Odom → Localization
                                                                      ↓
                                    LiDAR → Costmap → Planner ← ─ ─ ─ ┘

OBSTACLE AVOIDANCE:
  LiDAR → Costmap → Local Planner → Modified Path → Safe Velocities

""")
