// Central configuration for pins, sensors, speeds and I2C
#pragma once

// I2C pins for PCF8574
#define SDA_PIN 21
#define SCL_PIN 22
#define PCF_ADDR 0x20

// Sensor bit positions on PCF8574
#define S1_SENSOR_BIT 4 // Far left
#define S2_SENSOR_BIT 3 // Left
#define S3_SENSOR_BIT 2 // Center
#define S4_SENSOR_BIT 1 // Right
#define S5_SENSOR_BIT 0 // Far right

// Motor driver pins (ORIGINAL - CORRECT)
#define IN1 16
#define IN2 25
#define ENA 18
#define IN3 33
#define IN4 32
#define ENB 4

// Quadrature encoder pins (ORIGINAL - CORRECT)
#define leftENCA 26
#define leftENCB 27
#define rightENCA 14
#define rightENCB 13

// Robot physical parameters (ORIGINAL - CORRECT)
#define WHEEL_DIAMETER_M 0.07     // 7 cm wheels
#define PULSES_PER_REV 231          // Encoder ticks per wheel revolution
#define WHEELBASE_M 0.24            // Distance between wheels

// Speed constants for open-loop control
#define SPEED_FAST 220    // Straight line speed
#define SPEED_NORMAL 200  // Normal speed
#define SPEED_TURN 180    // Gentle turns
#define SPEED_SHARP 160   // Sharp turns

// PID control parameters
#define PID_SAMPLE_INTERVAL_MS 100  // 10 Hz control loop

// WiFi credentials
#define WIFI_SSID "Fulbright Student1"
#define WIFI_PASSWORD "fulbright2018"
#define WIFI_TIMEOUT_MS 10000

// Control mode
enum ControlMode {
  MODE_OPEN_LOOP = 0,  // Direct PWM control (original)
  MODE_PID_VELOCITY = 1 // PID velocity control (from wifiPID)
};

// Control source priority (who gets to control the robot)
enum ControlSource {
  SOURCE_NONE = 0,
  SOURCE_WIFI = 1,      // WiFi web interface (manual tuning)
  SOURCE_SERIAL = 2,    // ROS2 serial commands (navigation) - HIGHER PRIORITY
  SOURCE_AUTONOMOUS = 3 // Autonomous modes (e.g., line following)
};

#define CONTROL_TIMEOUT_MS 2000  // Release control if no command in 2 seconds (Nav2 sends at 200ms = 5Hz)
