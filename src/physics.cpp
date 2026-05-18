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
    ball.vy += kGravityMetersPerSecondSquared * static_cast<float>( dt );

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