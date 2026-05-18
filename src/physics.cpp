#include "physics.h"

#include <cmath>

constexpr float kGravityMetersPerSecondSquared{ 9.8f };
constexpr float kBounceDamping{ 0.8f };
constexpr float kWorldWidthMeters{ static_cast<float>( kScreenWidth ) / kPixelsPerMeter };
constexpr float kWorldHeightMeters{ static_cast<float>( kScreenHeight ) / kPixelsPerMeter };

float metersToPixels( float meters )
{
    return meters * kPixelsPerMeter;
}

int metersToPixelInt( float meters )
{
    return static_cast<int>( std::lround( metersToPixels( meters ) ) );
}

void updateBall( Ball& ball, double dt )
{
    ball.force_x = 0.0f;
    ball.force_y = ball.mass * kGravityMetersPerSecondSquared;

    float acceleration_y = ball.force_y / ball.mass;
    ball.vy += acceleration_y * dt;

    ball.x += ball.vx * static_cast<float>( dt );
    ball.y += ball.vy * static_cast<float>( dt );

    if( ball.y + ball.radius >= kWorldHeightMeters )
    {
        ball.y = kWorldHeightMeters - ball.radius;
        ball.vy *= -kBounceDamping;
    }

    if( ball.x - ball.radius <= 0.0f )
    {
        ball.x = ball.radius;
        ball.vx *= -kBounceDamping;
    }

    if( ball.x + ball.radius >= kWorldWidthMeters )
    {
        ball.x = kWorldWidthMeters - ball.radius;
        ball.vx *= -kBounceDamping;
    }
}

Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha )
{
    Ball out;
    out.x = prev.x + (curr.x - prev.x) * alpha;
    out.y = prev.y + (curr.y - prev.y) * alpha;
    out.vx = prev.vx + (curr.vx - prev.vx) * alpha;
    out.vy = prev.vy + (curr.vy - prev.vy) * alpha;
    out.radius = prev.radius + (curr.radius - prev.radius) * alpha;
    out.mass = curr.mass;
    out.force_x = curr.force_x;
    out.force_y = curr.force_y;
    return out;
}