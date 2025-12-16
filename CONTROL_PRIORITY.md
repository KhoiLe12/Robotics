# Control Priority System

## Overview

The robot firmware implements a priority-based control system to prevent conflicts between WiFi web interface and ROS2 autonomous navigation commands.

## Control Sources

Defined in `include/config.h`:

```cpp
enum ControlSource {
  SOURCE_NONE = 0,       // No active control
  SOURCE_WIFI = 1,       // WiFi web interface (manual tuning)
  SOURCE_SERIAL = 2,     // ROS2 serial commands (teleoperation)
  SOURCE_AUTONOMOUS = 3  // Reserved for future autonomous modes
};
```

**Priority Order**: AUTONOMOUS > SERIAL > WIFI > NONE

## How It Works

### 1. Control Source Tracking
- `currentControlSource`: Tracks who currently controls the robot
- `lastControlTime`: Timestamp of last command received
- Automatic timeout after 1000ms of inactivity releases control

### 2. ROS2 Navigation Priority
When ROS2 sends `PID_VEL,left_mps,right_mps` commands via serial:
- Sets `currentControlSource = SOURCE_SERIAL`
- Updates `lastControlTime = millis()`
- **Blocks WiFi web interface** from changing motor speeds or modes

### 3. WiFi Web Interface Behavior
When user tries to control via web interface:
- **If ROS2 active** (within 1s timeout): Returns HTTP 403 error "Control locked by ROS2"
- **If ROS2 timed out** (>1s since last command): WiFi takes control with `SOURCE_WIFI`
- **Emergency Stop** always works regardless of control source

### 4. Automatic Control Release
PID control task checks every 100ms:
```cpp
if (currentControlSource != SOURCE_NONE && 
    (millis() - lastControlTime) > CONTROL_TIMEOUT_MS) {
  currentControlSource = SOURCE_NONE;
  desiredSpeedL = 0.0;
  desiredSpeedR = 0.0;
}
```

## Usage Scenarios

### Scenario 1: Manual WiFi Tuning
1. Robot starts → `currentControlSource = SOURCE_NONE`
2. Access web interface → adjust PID parameters
3. Web server sets `currentControlSource = SOURCE_WIFI`
4. Robot operates under WiFi control

### Scenario 2: ROS2 Navigation Takes Over
1. ROS2 sends first `PID_VEL` command
2. `currentControlSource = SOURCE_SERIAL` (priority 2 > WiFi priority 1)
3. WiFi interface **locked out** - returns 403 errors
4. Robot follows ROS2 cmd_vel commands

### Scenario 3: ROS2 Navigation Stops
1. ROS2 stops sending commands
2. After 1000ms timeout → `currentControlSource = SOURCE_NONE`
3. Speeds reset to 0.0
4. WiFi interface **unlocked** - manual control restored

### Scenario 4: Emergency Stop
1. Click STOP button on web interface OR send `STOP` via serial
2. `currentControlSource = SOURCE_NONE` (releases all control)
3. Motors immediately stopped
4. Works regardless of who had control

## Serial Protocol Integration

### Commands Affected by Priority
- `PID_VEL,left_mps,right_mps` → Sets SOURCE_SERIAL, takes priority
- `STOP` → Releases control source to SOURCE_NONE
- `MODE_PID` / `MODE_OPENLOOP` → Currently no priority check (future enhancement)

### Web Endpoints Affected by Priority
- `/update` → Blocked if SOURCE_SERIAL active
- `/mode/pid` → Blocked if SOURCE_SERIAL active
- `/mode/openloop` → Blocked if SOURCE_SERIAL active
- `/stop` → **Always works** (emergency safety)

## Monitoring Control Source

The PID task prints control source status:
```
Spd L:0.35 R:0.42 Src:2
```
Where `Src:2` means SOURCE_SERIAL (ROS2) is in control.

## Configuration

Timeout can be adjusted in `include/config.h`:
```cpp
#define CONTROL_TIMEOUT_MS 1000  // 1 second
```

**Recommendations**:
- 1000ms: Good for teleoperation (10Hz cmd_vel updates)
- 500ms: More responsive, but requires faster ROS2 publishing
- 2000ms: More forgiving, but slower WiFi unlock

## Safety Features

1. **Automatic Timeout**: Prevents runaway if ROS2 crashes
2. **Speed Limiting**: PID_VEL commands constrained to ±2.0 m/s
3. **Emergency Stop**: Always bypasses priority system
4. **Clear Feedback**: Serial prints control source for debugging

## Future Enhancements

- Add SOURCE_AUTONOMOUS for Nav2 goal navigation
- Implement priority override key sequence for WiFi
- Add control source indicator to web UI
- Log control transitions for debugging
