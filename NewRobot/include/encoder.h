#pragma once
#include <Arduino.h>
#include "config.h"

// Initialize encoder pins and attach interrupts. Handles pin conflicts.
void encoderInit();

// ISR prototypes (attached internally)
void IRAM_ATTR onLeftA();
void IRAM_ATTR onRightA();

// Pulse counter API (cumulative counts)
long getLeftPulseCount();
long getRightPulseCount();

// Cumulative counts for odometry (never reset except by explicit command)
long getLeftCumulativeCount();
long getRightCumulativeCount();

void resetEncoderCounts();

// Velocity measurement API (for PID control)
long getLeftPulseCountAndReset();   // Get count and reset atomically
long getRightPulseCountAndReset();  // Get count and reset atomically
float getLeftVelocityMps();         // Get left wheel velocity in m/s
float getRightVelocityMps();        // Get right wheel velocity in m/s
