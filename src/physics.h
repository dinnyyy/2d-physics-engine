#pragma once

#include "main.h"

struct Ball {
    // All values are in meters or meters per second.
    float x{ 0.0f };
    float y{ 0.0f };
    float vx{ 0.0f };
    float vy{ 0.0f };
    float radius{ 0.4f };

    float mass{ 1.0f };

    float force_x{ 0.0f };
    float force_y{ 0.0f };
};

constexpr float kPixelsPerMeter{ 100.0f };

float metersToPixels( float meters );
int metersToPixelInt( float meters );
void updateBall( Ball& ball, double dt );
Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha );