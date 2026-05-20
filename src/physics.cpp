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
        float width = ball.radius * 2.0f;
        ball.inertia = (1.0f / 12.0f) * ball.mass * (width * width + width * width);
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
        // Left wall
        if (v.x <= 0.0f && !hit_x){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{1.0f,0.0f}, v);
            hit_x = true;
        } 
        // Right wall
        if (v.x >= kWorldWidthMeters && !hit_x){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{-1.0f,0.0f}, v);
            hit_x = true;
        }
        // Ceiling (assuming y=0 is the top)
        if (v.y <= 0.0f && !hit_y){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{0.0f,-1.0f}, v); // Make sure your normal faces the right way here!
            hit_y = true;
        }
        // Floor 
        if (v.y >= kWorldHeightMeters && !hit_y){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{0.0f,1.0f}, v);
            hit_y = true;
        }
    }
    
    // 1. Find the extreme bounds of our rotated corners
    float min_x = ballCorners[0].x, max_x = ballCorners[0].x;
    float min_y = ballCorners[0].y, max_y = ballCorners[0].y;
    
    for(int i = 1; i < 4; ++i) {
        min_x = std::min(min_x, ballCorners[i].x);
        max_x = std::max(max_x, ballCorners[i].x);
        min_y = std::min(min_y, ballCorners[i].y);
        max_y = std::max(max_y, ballCorners[i].y);
    }

    // 2. Resolve Penetration: Push the center of the box out by the exact amount the deepest corner sank
    if (min_x < 0.0f) {
        ball.p.x -= min_x; // Push right out of the left wall
    } else if (max_x > kWorldWidthMeters) {
        ball.p.x -= (max_x - kWorldWidthMeters); // Push left out of the right wall
    }
    
    if (min_y < 0.0f) {
        ball.p.y -= min_y; // Push down out of the ceiling
    } else if (max_y > kWorldHeightMeters) {
        ball.p.y -= (max_y - kWorldHeightMeters); // Push up out of the floor
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
