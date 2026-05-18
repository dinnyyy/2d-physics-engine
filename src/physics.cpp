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
    // Initialize inertia if needed (solid disk: I = 0.5 * m * r^2)
    if( ball.inertia <= 0.0f ) {
        ball.inertia = 0.5f * ball.mass * ball.radius * ball.radius;
    }

    // Linear motion
    ball.force = { 0.0f, ball.mass * kGravityMetersPerSecondSquared };

    Vec2 acceleration = (1.0f / ball.mass) * ball.force;
    ball.v = ball.v + acceleration * static_cast<float>( dt );

    ball.p = ball.p + ball.v * static_cast<float>( dt );


    //angular motion
    float angular_acceleration = ball.torque/ball.inertia;

    ball.av += angular_acceleration * static_cast<float>( dt );
    ball.angle += ball.av * static_cast<float>( dt );
    ball.torque = 0.0f;

    const float PI = 3.14159265358979323846f;
    while( ball.angle > PI ) ball.angle -= 2.0f * PI;
    while( ball.angle < -PI ) ball.angle += 2.0f * PI;

    if( ball.p.y + ball.radius >= kWorldHeightMeters )
    {
        ball.p.y = kWorldHeightMeters - ball.radius;
        ball.v.y *= -kBounceDamping;
    }

    if( ball.p.x - ball.radius <= 0.0f )
    {
        ball.p.x = ball.radius;
        ball.v.x *= -kBounceDamping;
    }

    if( ball.p.x + ball.radius >= kWorldWidthMeters )
    {
        ball.p.x = kWorldWidthMeters - ball.radius;
        ball.v.x *= -kBounceDamping;
    }
}

Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha )
{
    // For rendering we only need position interpolation. Copy other fields
    // from the current state so rendering uses the most recent physical state.
    Ball out = curr;
    out.p = prev.p + (curr.p - prev.p) * alpha;
    return out;
}