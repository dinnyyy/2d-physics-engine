#pragma once

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

struct Ball {
    // All values are in meters or meters per second.
    Vec2 p; // position
    Vec2 v; // velocity
    float radius{ 0.4f };

    float mass{ 1.0f };

    Vec2 force; // accumulated force
};

constexpr float kPixelsPerMeter{ 100.0f };

float metersToPixels( float meters );
int metersToPixelInt( float meters );
void updateBall( Ball& ball, double dt );
Ball interpolateBall( const Ball& prev, const Ball& curr, float alpha );