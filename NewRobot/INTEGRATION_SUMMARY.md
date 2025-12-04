# WiFi PID Integration Summary

## What Was Integrated

Successfully merged the WiFi PID control system (`wifiPID.cpp`) into the existing RTOS-based robot control system.

## Major Changes

### 1. **Updated Hardware Configuration** (`config.h`)
- **Motor Pins**: Updated to match wifiPID hardware
  - ENA (Left PWM): 19
  - IN1/IN2 (Left Direction): 5, 18
  - ENB (Right PWM): 4
  - IN3/IN4 (Right Direction): 17, 16
  
- **Encoder Pins**: Updated to match wifiPID
  - Left: A=26, B=27
  - Right: A=13, B=14

- **Robot Parameters**: Updated from wifiPID (higher precision)
  - Wheel diameter: 6.5cm (was 3.5cm)
  - Encoder resolution: 990 PPR (was 231 PPR)
  - Formula: 11 × 46.8 × 2 = 990 pulses per revolution

- **New Features**:
  - WiFi credentials configuration
  - PID sample interval (100ms)
  - Control mode enumeration (OPEN_LOOP vs PID_VELOCITY)

### 2. **Enhanced Motor Control** (`motor.h/cpp`)
Added PID velocity control alongside existing open-loop control:

**New Functions**:
- `setMotorLeft(float pwm)` - Direct PWM control (-255 to +255)
- `setMotorRight(float pwm)` - Direct PWM control (-255 to +255)
- `computePID_Left(desired, measured)` - PID calculation with anti-windup
- `computePID_Right(desired, measured)` - PID calculation with anti-windup
- `resetPID()` - Reset integral terms

**PID Features**:
- Conditional integration (prevents windup at saturation)
- Automatic integral reset on direction change
- Volatile PID gains (Kp, Ki, Kd) for live web tuning

### 3. **Encoder Velocity Measurement** (`encoder.h/cpp`)
Added velocity calculation functions for PID control:

**New Functions**:
- `getLeftPulseCountAndReset()` - Atomic read-and-reset
- `getRightPulseCountAndReset()` - Atomic read-and-reset
- `getLeftVelocityMps()` - Calculate left wheel velocity (m/s)
- `getRightVelocityMps()` - Calculate right wheel velocity (m/s)

**Updated ISRs**:
- Changed from `RISING` to `CHANGE` trigger for higher resolution
- Quadrature decoding logic matches wifiPID implementation

### 4. **Main System** (`main.cpp`)
Integrated WiFi and PID control into RTOS architecture:

**New Tasks**:
- `PIDControlTask` (Core 1, 10Hz) - Runs PID control loop when in PID mode
- `WiFiTask` (Core 0, 100Hz) - Handles web server requests

**WiFi Web Interface**:
- Real-time PID parameter tuning (Kp, Ki, Kd)
- Desired velocity setpoints for both motors
- Mode switching (PID ↔ Open Loop)
- Emergency stop button
- Access at: `http://<ESP32_IP>/`

**WiFi Setup**:
- Connects to WiFi on startup
- 10-second timeout if connection fails
- Continues operation without WiFi if unavailable

### 5. **Serial Protocol Extensions** (`serial_proto.cpp`)
Added new serial commands for PID control:

**New Commands**:
- `MODE_PID` - Switch to PID velocity control mode
- `MODE_OPENLOOP` - Switch to open-loop control mode
- `PID_VEL,left_speed,right_speed` - Set desired velocities (m/s)

**Updated Commands**:
- `STOP` - Now also resets desired PID speeds to 0

## Operating Modes

### Mode 1: Open Loop (Original)
- Direct PWM control via serial commands
- Functions: `FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `VEL`
- No velocity feedback

### Mode 2: PID Velocity Control (New)
- Closed-loop velocity control
- Set target speeds in m/s
- Automatic PWM adjustment based on encoder feedback
- Tunable via web interface or serial

## Usage Examples

### Via Serial (from Raspberry Pi)
```python
# Switch to PID mode
ser.write(b"MODE_PID\n")

# Set velocities: 0.2 m/s left, 0.2 m/s right (straight)
ser.write(b"PID_VEL,0.2,0.2\n")

# Differential drive: turn right
ser.write(b"PID_VEL,0.3,0.1\n")

# Stop
ser.write(b"STOP\n")

# Back to open loop
ser.write(b"MODE_OPENLOOP\n")
```

### Via Web Interface
1. Connect to WiFi network "Superior 523"
2. Check ESP32 serial output for IP address
3. Navigate to `http://192.168.x.x/` in browser
4. Adjust PID parameters in real-time
5. Click "Switch to PID Mode"
6. Set desired speeds and click "Update"

## RTOS Task Overview

| Task | Core | Priority | Frequency | Purpose |
|------|------|----------|-----------|---------|
| ControlTask | 1 | 2 | 100Hz | Line sensor reading (currently disabled) |
| SerialTask | 0 | 2 | 200Hz | Process serial commands |
| TelemetryTask | 0 | 1 | 20Hz | Send encoder/sensor data |
| BatteryTask | 0 | 1 | 0.5Hz | Monitor battery voltage |
| PIDControlTask | 1 | 2 | 10Hz | PID velocity control loop |
| WiFiTask | 0 | 1 | 100Hz | Handle web server requests |

## PID Tuning Guide

**Default Values** (from wifiPID):
- Left: Kp=1000, Ki=1000, Kd=10
- Right: Kp=1000, Ki=1000, Kd=10

**Tuning Process**:
1. Start with Kp only (set Ki=0, Kd=0)
2. Increase Kp until oscillation
3. Add Ki to eliminate steady-state error
4. Add Kd to reduce overshoot

**Web Interface Benefits**:
- Real-time parameter updates (no recompile needed)
- Immediate visual feedback on robot behavior
- Safe testing with emergency stop button

## Backward Compatibility

✅ **All original functionality preserved**:
- Serial commands still work in open-loop mode
- Line tracking code intact (currently disabled)
- Battery monitoring unchanged
- Telemetry format unchanged
- ROS2 bridge compatibility maintained

## Next Steps / Recommendations

1. **Calibrate PID gains** for your specific motors
2. **Update ROS2 bridge** to support PID velocity commands
3. **Re-enable line following** if needed (currently commented out)
4. **Add velocity telemetry** to serial output for debugging
5. **Consider adding IMU** for heading control in PID mode

## File Changes Summary

**Modified Files**:
- `include/config.h` - Hardware pins and parameters
- `include/encoder.h` - Added velocity functions
- `include/motor.h` - Added PID functions
- `include/serial_proto.h` - Exported PID variables
- `src/main.cpp` - Added WiFi and PID tasks
- `src/encoder.cpp` - Implemented velocity measurement
- `src/motor.cpp` - Implemented PID control
- `src/serial_proto.cpp` - Added PID commands

**Preserved Files**:
- `line_tracking.cpp/h` - Unchanged
- `battery.cpp/h` - Unchanged
- All ROS2 bridge files - Unchanged
- All Python navigation scripts - Unchanged

## Building and Uploading

```bash
# Using PlatformIO
cd /Users/khoimain/NewRobot
pio run --target upload

# Monitor serial output to see IP address
pio device monitor
```

## Troubleshooting

**WiFi won't connect**:
- Check SSID/password in `config.h`
- Look for "WiFi connected!" message in serial output
- System continues to work without WiFi

**PID not responding**:
- Verify mode is set to PID (check web interface or send `MODE_PID`)
- Check encoder connections
- Monitor serial output for encoder counts

**Motors running backward**:
- Swap motor wire polarity or
- Negate PWM values in `setMotorLeft`/`setMotorRight`

**Web interface not accessible**:
- Check IP address in serial output
- Ensure device is on same WiFi network
- Try ping `<ESP32_IP>` to verify connectivity
