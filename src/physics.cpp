#include "physics.h"

#include <cmath>
#include <algorithm>

constexpr float kGravityMetersPerSecondSquared{ 9.8f };
constexpr float kBounceDamping{ 0.8f };
constexpr float restitution{0.6f}; //bouncines
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

void apply_rel_v(Ball& ball, Vec2 v_rel, Vec2 n, Vec2 contact_point){
    float v_sep = v_rel.x * n.x + v_rel.y * n.y;  // dot product

    Vec2 r;
    r = contact_point - ball.p;
    // compute impulse magnitude
    float inv_mass = 1.0f / ball.mass;

    //rotational resistance - for ground, its just r.x
    float r_cross_n_x = r.x * n.y - r.y*n.x;
    float rot_resistance = (r_cross_n_x * r_cross_n_x) / ball.inertia;

    float j = -(1.0f + restitution) *v_sep/(inv_mass+rot_resistance);

    // apply impulse to linear v
    ball.v += (j*n) *inv_mass;

    // apply impulse to av
    float torque = r.x * (j * n.y) - r.y * (j * n.x);
    ball.av += torque / ball.inertia;

}

Vec2 compute_rel_v (Ball& ball, Vec2 contact_point) {
    Vec2 r;
    r = contact_point - ball.p;

    Vec2 v_rel;
    v_rel = ball.v;

    Vec2 v_spin;
    v_spin.x = -ball.av * r.y;
    v_spin.y = ball.av *r.x;

    v_rel += v_spin;
    return v_rel;
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

    std::vector<Vec2> ballCorners;
    ballCorners.reserve( 4 );

    const float halfSize = ball.radius;
    const Vec2 localCorners[4] = {
        { -halfSize, -halfSize },
        { halfSize, -halfSize },
        { halfSize, halfSize },
        { -halfSize, halfSize }
    };

    const float c = std::cos( ball.angle );
    const float s = std::sin( ball.angle );

    for( int i = 0; i < 4; i++ ) {
        float rx = localCorners[i].x * c - localCorners[i].y * s;
        float ry = localCorners[i].x * s + localCorners[i].y * c;
        ballCorners.push_back( { ball.p.x + rx, ball.p.y + ry } );
    }
    bool hit_x {false}, hit_y {false};
    for (const Vec2& v: ballCorners){
        // Left/right walls: bounce only if moving into the wall
        if (v.x <= 0.0f && ball.v.x < 0.0f && !hit_x){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{1.0f,0.0f}, v);
            hit_x = true;
        } 
        if (v.x >= kWorldWidthMeters && ball.v.x > 0.0f && !hit_x){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{-1.0f,0.0f}, v);
            hit_x = true;
        }
        // Floor/ceiling: bounce only if moving into the boundary
        if (v.y <= 0.0f && ball.v.y < 0.0f && !hit_y){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{0.0f,-1.0f}, v);
            hit_y = true;
        }
        if (v.y >= kWorldHeightMeters && ball.v.y > 0.0f && !hit_y){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{0.0f,1.0f}, v);
            hit_y = true;
        }
    }
    
    // Keep ball center at valid distance from boundaries to prevent sinking
    ball.p.y = std::max( ball.p.y, ball.radius );
    ball.p.x = std::clamp( ball.p.x, ball.radius, kWorldWidthMeters - ball.radius );
}

Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha )
{
    // For rendering we only need position interpolation. Copy other fields
    // from the current state so rendering uses the most recent physical state.
    Ball out = curr;
    out.p = prev.p + (curr.p - prev.p) * alpha;
    return out;
}
