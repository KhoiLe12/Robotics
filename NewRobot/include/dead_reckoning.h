#pragma once
#include "encoder.h"
#include "config.h"

struct Pose {
    float x;
    float y;
    float theta;
};

void deadReckoningInit(float wheelBase, float metersPerTick);
void deadReckoningUpdate();          // call every 10–20 ms
Pose getCurrentPose();               // current (x,y,θ)
void resetPose();                    // zero everything
