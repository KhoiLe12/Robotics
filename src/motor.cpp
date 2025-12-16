#include "motor.h"

// PID tuning parameters (volatile for safe web updates)
// TUNED FOR SMOOTH, NON-JITTERY NAV2 INTEGRATION
volatile float KpL = 150.0;   // Proportional gain - reduced for smoother response
volatile float KiL = 80.0;    // Integral gain - reduced to prevent oscillation
volatile float KdL = 15.0;    // Derivative gain - increased for better damping
volatile float desiredSpeedL = 0.0;

volatile float KpR = 150.0;   // Proportional gain - reduced for smoother response
volatile float KiR = 80.0;    // Integral gain - reduced to prevent oscillation
volatile float KdR = 15.0;    // Derivative gain - increased for better damping
volatile float desiredSpeedR = 0.0;

// Feedforward gain (helps with initial response to velocity commands)
const float Kf = 650.0;  // PWM per m/s - slightly increased to compensate for lower Kp
const float minPWM = 50;  // Minimum PWM to overcome motor deadzone - increased slightly

// PID state variables
static float prevErrorLeft = 0, integralLeft = 0;
static float prevErrorRight = 0, integralRight = 0;
static float prevDesiredSpeedL = 0.0;
static float prevDesiredSpeedR = 0.0;
static const float dt = PID_SAMPLE_INTERVAL_MS / 1000.0;

// ========== OPEN-LOOP CONTROL (Original) ==========

void go_Advance(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void go_GentleLeft() {
  // Right motor faster, left motor slower
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN - 20);  // Left motor slower
  analogWrite(ENB, SPEED_TURN);       // Right motor normal
}

void go_GentleRight() {
  // Left motor faster, right motor slower
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN);       // Left motor normal
  analogWrite(ENB, SPEED_TURN - 20);  // Right motor slower
}

void go_Left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN);
  analogWrite(ENB, SPEED_TURN);
}

void go_Right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED_TURN);
  analogWrite(ENB, SPEED_TURN);
}

void go_SharpLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_SHARP);
  analogWrite(ENB, SPEED_SHARP);
}

void go_SharpRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED_SHARP);
  analogWrite(ENB, SPEED_SHARP);
}

void stop_car() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ========== CELEBRATION/DEMO FUNCTIONS ==========

void celebrationSpin(int durationMs) {
  // Fast spin in place at max PWM - very visible!
  // Left motor backward, right motor forward for clockwise spin
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);  // Max PWM
  analogWrite(ENB, 255);  // Max PWM
  
  delay(durationMs);
  
  // Stop after spin
  stop_car();
}

// ========== LOW-LEVEL MOTOR CONTROL (For PID) ==========

void setMotorLeft(float pwm) {
  pwm = constrain(pwm, -255, 255);
  int speed = abs(pwm);

  if (pwm > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  analogWrite(ENA, speed);
}

void setMotorRight(float pwm) {
  pwm = constrain(pwm, -255, 255);
  int speed = abs(pwm);

  if (pwm > 0) {
    // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENB, speed);
}

// ========== PID CONTROL ==========

float computePID_Left(float desired, float measured) {
  // Reset integral when desired speed sign changes to prevent windup in wrong direction
  if ((prevDesiredSpeedL > 0 && desired < 0) || (prevDesiredSpeedL < 0 && desired > 0)) {
    integralLeft = 0.0;
  }
  prevDesiredSpeedL = desired;
  
  float error = desired - measured;
  
  // Calculate proportional and derivative terms first
  float derivative = (error - prevErrorLeft) / dt;
  prevErrorLeft = error;
  float proportional = KpL * error;
  float derivativeTerm = KdL * derivative;
  
  // Calculate output with current integral to check for saturation
  float testOutput = proportional + KiL * integralLeft + derivativeTerm;
  
  // Conditional integration: only accumulate integral if output is not saturated
  bool isSaturated = (testOutput >= 255 && error > 0) || (testOutput <= -255 && error < 0);
  
  if (!isSaturated) {
    // Only integrate when not saturated to prevent windup
    integralLeft += error * dt;
    // Clamp integral to prevent excessive accumulation (±2.0 allows ~100 PWM from I-term)
    integralLeft = constrain(integralLeft, -2.0, 2.0);
  }

  // Calculate final PID output
  float output = proportional + KiL * integralLeft + derivativeTerm;
  
  // Add feedforward term based on desired velocity (helps initial response)
  float feedforward = Kf * desired;
  output += feedforward;
  
  // Apply minimum PWM threshold to overcome motor deadzone (but not for zero command)
  if (output > 0 && output < minPWM && desired > 0.01) {
    output = minPWM;
  } else if (output < 0 && output > -minPWM && desired < -0.01) {
    output = -minPWM;
  }
  
  return output;
}

float computePID_Right(float desired, float measured) {
  // Reset integral when desired speed sign changes to prevent windup in wrong direction
  if ((prevDesiredSpeedR > 0 && desired < 0) || (prevDesiredSpeedR < 0 && desired > 0)) {
    integralRight = 0.0;
  }
  prevDesiredSpeedR = desired;
  
  float error = desired - measured;
  
  // Calculate proportional and derivative terms first
  float derivative = (error - prevErrorRight) / dt;
  prevErrorRight = error;
  float proportional = KpR * error;
  float derivativeTerm = KdR * derivative;
  
  // Calculate output with current integral to check for saturation
  float testOutput = proportional + KiR * integralRight + derivativeTerm;
  
  // Conditional integration: only accumulate integral if output is not saturated
  bool isSaturated = (testOutput >= 255 && error > 0) || (testOutput <= -255 && error < 0);
  
  if (!isSaturated) {
    // Only integrate when not saturated to prevent windup
    integralRight += error * dt;
    // Clamp integral to prevent excessive accumulation (±2.0 allows ~100 PWM from I-term)
    integralRight = constrain(integralRight, -2.0, 2.0);
  }

  // Calculate final PID output
  float output = proportional + KiR * integralRight + derivativeTerm;
  
  // Add feedforward term based on desired velocity (helps initial response)
  float feedforward = Kf * desired;
  output += feedforward;
  
  // Apply minimum PWM threshold to overcome motor deadzone (but not for zero command)
  if (output > 0 && output < minPWM && desired > 0.01) {
    output = minPWM;
  } else if (output < 0 && output > -minPWM && desired < -0.01) {
    output = -minPWM;
  }
  
  return output;
}

void resetPID() {
  prevErrorLeft = 0;
  integralLeft = 0;
  prevErrorRight = 0;
  integralRight = 0;
  prevDesiredSpeedL = 0.0;
  prevDesiredSpeedR = 0.0;
}
