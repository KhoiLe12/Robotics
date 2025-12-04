#include "encoder.h"

// Internal state
static volatile long leftPulseCount_ = 0;  // For PID (delta counts, reset each cycle)
static volatile long rightPulseCount_ = 0; // For PID (delta counts, reset each cycle)
static volatile long leftCumulativeCount_ = 0;  // For odometry (never reset)
static volatile long rightCumulativeCount_ = 0; // For odometry (never reset)
static bool leftEnabled_ = false;
static bool rightEnabled_ = false;

// Timestamp tracking for velocity calculation
static unsigned long lastVelocityUpdate_ = 0;

void encoderInit() {
  // Check for pin conflicts with motor pins
  if (leftENCA != IN1 && leftENCA != IN2 && leftENCA != ENA &&
      leftENCA != IN3 && leftENCA != IN4 && leftENCA != ENB) {
    pinMode(leftENCA, INPUT_PULLUP);
    pinMode(leftENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(leftENCA), onLeftA, CHANGE);
    leftEnabled_ = true;
  } else {
    Serial.println("[Encoder] WARNING: leftENCA pin conflicts with motor pins. Left encoder disabled.");
    leftEnabled_ = false;
  }

  if (rightENCA != IN1 && rightENCA != IN2 && rightENCA != ENA &&
      rightENCA != IN3 && rightENCA != IN4 && rightENCA != ENB) {
    pinMode(rightENCA, INPUT_PULLUP);
    pinMode(rightENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rightENCA), onRightA, CHANGE);
    rightEnabled_ = true;
  } else {
    Serial.println("[Encoder] WARNING: rightENCA pin conflicts with motor pins. Right encoder disabled.");
    rightEnabled_ = false;
  }
  
  lastVelocityUpdate_ = millis();
}

void IRAM_ATTR onLeftA() {
  bool b = digitalRead(leftENCB);
  if (digitalRead(leftENCB) != digitalRead(leftENCA)) {
    leftPulseCount_++;  // Clockwise (for PID)
    leftCumulativeCount_++;  // Clockwise (for odometry)
  } else {
    leftPulseCount_--;  // Counter-clockwise (for PID)
    leftCumulativeCount_--;  // Counter-clockwise (for odometry)
  }
}

void IRAM_ATTR onRightA() {
  if (digitalRead(rightENCB) != digitalRead(rightENCA)) {
    rightPulseCount_++;  // Clockwise (for PID)
    rightCumulativeCount_++;  // Clockwise (for odometry)
  } else {
    rightPulseCount_--;  // Counter-clockwise (for PID)
    rightCumulativeCount_--;  // Counter-clockwise (for odometry)
  }
}

long getLeftPulseCount() {
  noInterrupts();
  long v = leftPulseCount_;
  interrupts();
  return v;
}

long getRightPulseCount() {
  noInterrupts();
  long v = rightPulseCount_;
  interrupts();
  return v;
}

// Get cumulative counts for odometry (never reset except by explicit command)
long getLeftCumulativeCount() {
  noInterrupts();
  long v = leftCumulativeCount_;
  interrupts();
  return v;
}

long getRightCumulativeCount() {
  noInterrupts();
  long v = rightCumulativeCount_;
  interrupts();
  return v;
}

void resetEncoderCounts() {
  noInterrupts();
  leftPulseCount_ = 0;
  rightPulseCount_ = 0;
  leftCumulativeCount_ = 0;
  rightCumulativeCount_ = 0;
  interrupts();
}

// For PID control - get count and reset atomically
long getLeftPulseCountAndReset() {
  noInterrupts();
  long v = leftPulseCount_;
  leftPulseCount_ = 0;
  interrupts();
  return v;
}

long getRightPulseCountAndReset() {
  noInterrupts();
  long v = rightPulseCount_;
  rightPulseCount_ = 0;
  interrupts();
  return v;
}

// Calculate velocity in m/s
float getLeftVelocityMps() {
  unsigned long now = millis();
  unsigned long dt = now - lastVelocityUpdate_;
  if (dt == 0) dt = 1;  // Prevent division by zero
  
  long count = getLeftPulseCountAndReset();
  float revs = (float)count / PULSES_PER_REV;
  float wheelCirc = PI * WHEEL_DIAMETER_M;
  float velocity = (revs * wheelCirc) / (dt / 1000.0);
  
  lastVelocityUpdate_ = now;
  return velocity;
}

float getRightVelocityMps() {
  unsigned long now = millis();
  unsigned long dt = now - lastVelocityUpdate_;
  if (dt == 0) dt = 1;  // Prevent division by zero
  
  long count = getRightPulseCountAndReset();
  float revs = (float)count / PULSES_PER_REV;
  float wheelCirc = PI * WHEEL_DIAMETER_M;
  float velocity = (revs * wheelCirc) / (dt / 1000.0);
  
  return velocity;
}
