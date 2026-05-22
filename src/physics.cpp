#include "physics.h"

#include <array>
#include <cmath>
#include <algorithm>

constexpr float kGravityMetersPerSecondSquared{ 9.8f };
constexpr float restitution{0.4f}; //bouncines
constexpr float frictionConstant{0.05f}; //floor friction

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

struct PixelPoint {
    int x;
    int y;
};

// The function returns an array of exactly 4 PixelPoints
std::array<PixelPoint, 4> getCorners(const Ball& ball) {
    int cx = metersToPixelInt(ball.p.x);
    int cy = metersToPixelInt(ball.p.y);

    // Corners of the ball square centered at origin
    float halfSize = ball.radius;
    const Vec2 localCorners[4] = {
        { -halfSize, -halfSize },
        { halfSize, -halfSize },
        { halfSize, halfSize },
        { -halfSize, halfSize }
    };

    // Rotate corners by ball.angle
    float c = std::cos(ball.angle);
    float s = std::sin(ball.angle);

    std::array<PixelPoint, 4> screenCorners;

    for( int i = 0; i < 4; i++ ) {
        float rx = localCorners[i].x * c - localCorners[i].y * s;
        float ry = localCorners[i].x * s + localCorners[i].y * c;
        
        screenCorners[i].x = cx + metersToPixelInt(rx);
        screenCorners[i].y = cy + metersToPixelInt(ry);
    }

    return screenCorners;
}

std::vector<Vec2> find_testing_axes( const Ball& ball ) {
    std::array<PixelPoint, 4> screenCorners = getCorners( ball );

    Vec2 corner1{ static_cast<float>( screenCorners[0].x ), static_cast<float>( screenCorners[0].y ) };
    Vec2 corner2{ static_cast<float>( screenCorners[1].x ), static_cast<float>( screenCorners[1].y ) };
    Vec2 corner3{ static_cast<float>( screenCorners[2].x ), static_cast<float>( screenCorners[2].y ) };
    
    Vec2 edge_vector1{corner2-corner1};
    Vec2 edge_vector2{corner3-corner2};
    
    float len1 = std::sqrt(edge_vector1.x * edge_vector1.x +
                       edge_vector1.y * edge_vector1.y);

    float len2 = std::sqrt(edge_vector2.x * edge_vector2.x +
                        edge_vector2.y * edge_vector2.y);

    Vec2 axes1{
        -edge_vector1.y / len1,
        edge_vector1.x / len1
    };

    Vec2 axes2{
        -edge_vector2.y / len2,
        edge_vector2.x / len2
    };

    return {axes1, axes2};
}


bool detect_ball_collision( const Ball& b1, const Ball& b2 ){
    std::array<PixelPoint, 4> b1_corners = getCorners( b1 );
    std::array<PixelPoint, 4> b2_corners = getCorners( b2 );
    std::vector<Vec2> b1_axes{find_testing_axes(b1)};
    std::vector<Vec2> b2_axes{find_testing_axes(b2)};

    std::vector<Vec2> axes = b1_axes;

    axes.insert(
        axes.end(),
        b2_axes.begin(),
        b2_axes.end()
    );
    
    for (Vec2& axis : axes) {
        std::array<float, 4> b1_projections;
        std::array<float, 4> b2_projections;
        
        for (float i=0; i<4; i++) {
            b1_projections[i] = b1_corners[i].x*axis.x + b1_corners[i].y*axis.y;
            b2_projections[i] = b2_corners[i].x*axis.x + b2_corners[i].y*axis.y;
        }
        auto [b1_min_it, b1_max_it] = std::minmax_element( b1_projections.begin(), b1_projections.end() );
        auto [b2_min_it, b2_max_it] = std::minmax_element( b2_projections.begin(), b2_projections.end() );

        float b1_min = *b1_min_it;
        float b1_max = *b1_max_it;
        float b2_min = *b2_min_it;
        float b2_max = *b2_max_it;

        if( b1_max < b2_min || b1_min > b2_max ) {
            return false;
        }
    }

    return true;

    return false;
}

void updateBall( Ball& ball, double dt, const std::vector<Ball>& balls )
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
    ball.isHit = false;
    for( const Ball& other : balls ) {
        if( &other == &ball ) {
            continue;
        }

        if( detect_ball_collision( ball, other ) ) {
            ball.isHit = true;
            break;
        }
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
