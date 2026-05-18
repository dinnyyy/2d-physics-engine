#pragma once

#include "main.h"

struct Ball {
    // All values are in meters or meters per second.
    float x{ 0.0f };
    float y{ 0.0f };
    float vx{ 0.0f };
    float vy{ 0.0f };
    float radius{ 0.4f };
};

constexpr float kPixelsPerMeter{ 100.0f };

float metersToPixels( float meters );
int metersToPixelInt( float meters );
void updateBall( Ball& ball, double dt );