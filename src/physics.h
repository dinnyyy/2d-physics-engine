#pragma once
#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>

#include "main.h"

struct Vec2 {
    float x{ 0.0f };
    float y{ 0.0f };

    Vec2& operator+=( const Vec2& other )
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=( const Vec2& other )
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }
};

inline Vec2 operator+( const Vec2& a, const Vec2& b ) { return { a.x + b.x, a.y + b.y }; }
inline Vec2 operator-( const Vec2& a, const Vec2& b ) { return { a.x - b.x, a.y - b.y }; }
inline Vec2 operator*( const Vec2& v, float s ) { return { v.x * s, v.y * s }; }
inline Vec2 operator*( float s, const Vec2& v ) { return { v.x * s, v.y * s }; }

// Vec2 utility helpers
inline float dot( const Vec2& a, const Vec2& b ) { return a.x * b.x + a.y * b.y; }
// 2D cross product (returns scalar z-component)
inline float cross( const Vec2& a, const Vec2& b ) { return a.x * b.y - a.y * b.x; }
inline float length( const Vec2& v ) { return std::sqrt( dot( v, v ) ); }
inline Vec2 normalize( const Vec2& v ) { float l = length( v ); return l > 0.0f ? v * (1.0f / l) : Vec2{0.0f, 0.0f}; }

struct Ball {
    // All values are in meters or meters per second.
    Vec2 p; // position
    Vec2 v; // velocity
    float radius{ 0.4f };

    float mass{ 1.0f };
    float invmass{0.0f};

    Vec2 force; // accumulated force

    float angle{0.0f};
    float av{0.0f};
    float aa{0.0f};

    float torque{0.0f};
    float inertia{0.0f};
    bool isHit{ false };
    


};

constexpr float kPixelsPerMeter{ 100.0f };

float metersToPixels( float meters );
int metersToPixelInt( float meters );
void updateBall( Ball& ball, double dt, const std::vector<Ball>& balls );
Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha );