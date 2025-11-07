#include "dead_reckoning.h"
#include <Arduino.h>
#include <math.h>

static long lastLeft = 0, lastRight = 0;
static float _x = 0, _y = 0, _theta = 0;
static float _wheelBase = 0.24f;        // meters – set to your robot’s wheelbase
static float wheel_radius = 0.035f;   // meters – set to your robot’s wheel radius
static const int TICKS_PER_REV = 231; // encoder ticks per wheel revolution
static float _metersPerTick = (2.0 * M_PI * wheel_radius) / TICKS_PER_REV;

void deadReckoningInit(float wheelBase, float metersPerTick) {
    _wheelBase = wheelBase;
    _metersPerTick = metersPerTick;
    resetPose();
}

void resetPose() {
    lastLeft = getLeftPulseCount();
    lastRight = getRightPulseCount();
    _x = _y = _theta = 0;
}

void deadReckoningUpdate() {
    long leftNow = getLeftPulseCount();
    long rightNow = getRightPulseCount();

    long dL = leftNow - lastLeft;
    long dR = rightNow - lastRight;

    lastLeft = leftNow;
    lastRight = rightNow;

    float leftDist = dL * _metersPerTick;
    float rightDist = dR * _metersPerTick;
    float dist = (leftDist + rightDist) * 0.5f;
    float dTheta = (rightDist - leftDist) / _wheelBase;

    // integrate
    _x     += dist * cosf(_theta + dTheta * 0.5f);
    _y     += dist * sinf(_theta + dTheta * 0.5f);
    _theta += dTheta;

    // keep θ in −π..π
    if (_theta > M_PI)  _theta -= 2*M_PI;
    if (_theta < -M_PI) _theta += 2*M_PI;
}

Pose getCurrentPose() {
    return {_x, _y, _theta};
}
