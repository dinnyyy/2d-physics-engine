#include "physics.h"

#include <cmath>
#include <algorithm>

constexpr float kGravityMetersPerSecondSquared{ 9.8f };
constexpr float restitution{0.2f}; //bouncines
constexpr float frictionConstant{0.1f}; //floor friction

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
    // Only apply impulse if separating velocity is negative
    if (v_sep >= 0.0f) return;
    Vec2 r;
    r = contact_point - ball.p;
    // compute impulse magnitude
    float inv_mass = 1.0f / ball.mass;

    //rotational resistance - for ground, its just r.x
    float r_cross_n_x = r.x * n.y - r.y*n.x;
    float rot_resistance = (r_cross_n_x * r_cross_n_x) / ball.inertia;

    float j = -(1.0f + restitution) * v_sep / (inv_mass + rot_resistance);

    // apply impulse to linear v
    ball.v += (j*n) *inv_mass;

    // apply impulse to av
    float torque = r.x * (j * n.y) - r.y * (j * n.x);
    ball.av += torque / ball.inertia;

    Vec2 n_tan = { -n.y, n.x };;
    n_tan.x *= -1.0f;

    float v_tan = v_rel.x*n_tan.x + v_rel.y*n_tan.y;
    float r_cross_n_x_tan = r.x * n_tan.y - r.y*n_tan.x;
    float rot_resistance_tan = (r_cross_n_x_tan * r_cross_n_x_tan) / ball.inertia;
    float j_tan = -v_tan / (inv_mass + rot_resistance_tan);
    float max_friction = j * frictionConstant; // 'j' is the normal impulse from earlier

    // Clamp j_t between -max_friction and +max_friction
    j_tan = std::clamp(j_tan, -max_friction, max_friction);

    ball.v.x += (j_tan * n_tan.x) * inv_mass;
    ball.v.y += (j_tan * n_tan.y) * inv_mass;

    // 7. Apply the friction torque to angular velocity
    float torque_t = r.x * (j_tan * n_tan.y) - r.y * (j_tan * n_tan.x);
    ball.av += torque_t / ball.inertia;

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

std::vector<Vec2> compute_world_corners( const Ball& ball ) {
    std::vector<Vec2> corners;
    corners.reserve( 4 );

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
        corners.push_back( { ball.p.x + rx, ball.p.y + ry } );
    }
    return corners;
}

void detect_boundary_collisions( Ball& ball, const std::vector<Vec2>& corners ) {
    bool hit_x {false}, hit_y {false};
    for (const Vec2& v: corners){
        // Left wall (Normal points RIGHT)
        if (v.x <= 0.0f && !hit_x){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{1.0f, 0.0f}, v);
            hit_x = true;
        } 
        // Right wall (Normal points LEFT)
        if (v.x >= kWorldWidthMeters && !hit_x){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{-1.0f, 0.0f}, v);
            hit_x = true;
        }
        // Ceiling (Normal points DOWN)
        if (v.y <= 0.0f && !hit_y){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{0.0f, 1.0f}, v); 
            hit_y = true;
        }
        // Floor (Normal points UP)
        if (v.y >= kWorldHeightMeters && !hit_y){
            Vec2 rel_v = compute_rel_v(ball, v);
            apply_rel_v(ball, rel_v, Vec2{0.0f, -1.0f}, v); // Changed to -1.0f!
            hit_y = true;
        }
    }
    
    // Resolve penetration: push out by exact amount
    float min_x = corners[0].x, max_x = corners[0].x;
    float min_y = corners[0].y, max_y = corners[0].y;
    
    for(int i = 1; i < 4; ++i) {
        min_x = std::min(min_x, corners[i].x);
        max_x = std::max(max_x, corners[i].x);
        min_y = std::min(min_y, corners[i].y);
        max_y = std::max(max_y, corners[i].y);
    }

    if (min_x < 0.0f) {
        ball.p.x -= min_x;
    } else if (max_x > kWorldWidthMeters) {
        ball.p.x -= (max_x - kWorldWidthMeters);
    }
    
    if (min_y < 0.0f) {
        ball.p.y -= min_y;
    } else if (max_y > kWorldHeightMeters) {
        ball.p.y -= (max_y - kWorldHeightMeters);
    }
}

void update_linear_motion( Ball& ball, float dt ) {
    ball.force = { 0.0f, ball.mass * kGravityMetersPerSecondSquared };
    Vec2 acceleration = (1.0f / ball.mass) * ball.force;
    ball.v = ball.v + acceleration * dt;
    ball.p = ball.p + ball.v * dt;
}

void update_angular_motion( Ball& ball, float dt ) {
    float angular_acceleration = ball.torque / ball.inertia;
    ball.av += angular_acceleration * dt;
    ball.angle += ball.av * dt;
    ball.torque = 0.0f;

    const float PI = 3.14159265358979323846f;
    while( ball.angle > PI ) ball.angle -= 2.0f * PI;
    while( ball.angle < -PI ) ball.angle += 2.0f * PI;
}

void detect_boundary_collisions (){
    
}

void updateBall( Ball& ball, double dt )
{
    // Initialize inertia if needed
    if( ball.inertia <= 0.0f ) {
        float width = ball.radius * 2.0f;
        ball.inertia = (1.0f / 12.0f) * ball.mass * (width * width + width * width);
    }

    update_linear_motion( ball, static_cast<float>( dt ) );
    update_angular_motion( ball, static_cast<float>( dt ) );

    std::vector<Vec2> corners = compute_world_corners( ball );
    detect_boundary_collisions( ball, corners );
}

Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha )
{
    // For rendering we only need position interpolation. Copy other fields
    // from the current state so rendering uses the most recent physical state.
    Ball out = curr;
    out.p = prev.p + (curr.p - prev.p) * alpha;
    return out;
}
