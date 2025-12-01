#pragma once
#include <Arduino.h>
#include "config.h"

// Open-loop motor control functions (original)
void go_Advance(int speed);
void go_GentleLeft();
void go_GentleRight();
void go_Left();
void go_Right();
void go_SharpLeft();
void go_SharpRight();
void stop_car();

// Low-level motor control (for PID)
void setMotorLeft(float pwm);   // -255 to +255
void setMotorRight(float pwm);  // -255 to +255

// PID control functions
float computePID_Left(float desired, float measured);
float computePID_Right(float desired, float measured);
void resetPID();  // Reset integral terms

// PID tuning parameters (volatile for web updates)
extern volatile float KpL, KiL, KdL;
extern volatile float KpR, KiR, KdR;
extern volatile float desiredSpeedL, desiredSpeedR;
