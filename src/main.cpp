/* Headers */
//Using SDL and STL string
#include <SDL3/SDL_main.h>
#include "main.h"
#include "physics.h"
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>

SDL_Window* gWindow{ nullptr };
SDL_Surface* gScreenSurface{ nullptr };
SDL_Surface* gHelloWorld{ nullptr };

constexpr double kMaxFrameTime{ 0.25 }; // clamp to avoid spiral of death
constexpr double kFixedDt{ 1.0 / 60.0 };

/* Function Implementations */
bool init()
{
    //Initialization flag
    bool success{ true };

    //Initialize SDL
    if( SDL_Init( SDL_INIT_VIDEO ) == false )
    {
        SDL_Log( "SDL could not initialize! SDL error: %s\n", SDL_GetError() );
        success = false;
    }
    else
    {
        //Create window
        if( gWindow = SDL_CreateWindow( "SDL3 Tutorial: Hello SDL3", kScreenWidth, kScreenHeight, 0 ); gWindow == nullptr )
        {
            SDL_Log( "Window could not be created! SDL error: %s\n", SDL_GetError() );
            success = false;
        }
        else
        {
            //Get window surface
            gScreenSurface = SDL_GetWindowSurface( gWindow );
        }
    }

    return success;
}

void close()
{
    //Clean up surface
    SDL_DestroySurface( gHelloWorld );
    gHelloWorld = nullptr;
    
    //Destroy window
    SDL_DestroyWindow( gWindow );
    gWindow = nullptr;
    gScreenSurface = nullptr;

    //Quit SDL subsystems
    SDL_Quit();
}

void renderBall (const Ball& ball) {
    int cx = metersToPixelInt(ball.p.x);
    int cy = metersToPixelInt(ball.p.y);

    // Corners of the ball square centered at origin
    float halfSize = ball.radius;
    Vec2 corners[4] = {
        { -halfSize, -halfSize },
        { halfSize, -halfSize },
        { halfSize, halfSize },
        { -halfSize, halfSize }
    };

    // Rotate corners by ball.angle
    float c = std::cos( ball.angle );
    float s = std::sin( ball.angle );

    int px[4], py[4];
    for( int i = 0; i < 4; i++ ) {
        float rx = corners[i].x * c - corners[i].y * s;
        float ry = corners[i].x * s + corners[i].y * c;
        px[i] = cx + metersToPixelInt( rx );
        py[i] = cy + metersToPixelInt( ry );
    }
    
    // Draw rotated square filled with black
    Uint32 black = SDL_MapSurfaceRGB( gScreenSurface, 0, 0, 0 );

    int minY = std::min( {py[0], py[1], py[2], py[3]} );
    int maxY = std::max( {py[0], py[1], py[2], py[3]} );

    for( int y = minY; y <= maxY; y++ ) {
        std::vector<float> intersections;
        intersections.reserve( 4 );

        for( int i = 0; i < 4; i++ ) {
            int j = ( i + 1 ) % 4;
            int y0 = py[i];
            int y1 = py[j];

            // Use a half-open interval to avoid double-counting shared vertices.
            if( ( y0 <= y && y1 > y ) || ( y1 <= y && y0 > y ) ) {
                float t = static_cast<float>( y - y0 ) / static_cast<float>( y1 - y0 );
                float x = static_cast<float>( px[i] ) + t * static_cast<float>( px[j] - px[i] );
                intersections.push_back( x );
            }
        }

        if( intersections.size() < 2 ) {
            continue;
        }

        std::sort( intersections.begin(), intersections.end() );
        for( size_t k = 0; k + 1 < intersections.size(); k += 2 ) {
            int xStart = static_cast<int>( std::ceil( intersections[k] ) );
            int xEnd = static_cast<int>( std::floor( intersections[k + 1] ) );
            if( xEnd >= xStart ) {
                SDL_Rect line{ xStart, y, xEnd - xStart + 1, 1 };
                SDL_FillSurfaceRect( gScreenSurface, &line, black );
            }
        }
    }
}

void clearScreen()
{
    SDL_FillSurfaceRect(
        gScreenSurface,
        nullptr,
        SDL_MapSurfaceRGB( gScreenSurface, 255, 255, 255 )
    );
}

void handleEvents( bool& quit )
{
    SDL_Event e;

    while( SDL_PollEvent( &e ) )
    {
        if( e.type == SDL_EVENT_QUIT )
        {
            quit = true;
        }
    }
}

int main( int argc, char* args[] )
{
    //Final exit code
    int exitCode{ 0 };

    //Initialize
    if( init() == false )
    {
        SDL_Log( "Unable to initialize program!\n" );
        exitCode = 1;
    }
    else
    {
        std::vector<Ball> balls;

        Ball ball1;
        ball1.p.x = 1.0f;
        ball1.p.y = 1.0f;
        ball1.v.x = 1.0f;
        ball1.v.y = 0.0f;
        ball1.radius = 0.4f;
        ball1.av = 3.0f;  // Initial angular velocity for visible rotation
        ball1.inertia = 1.0f;
        balls.push_back( ball1 );

        Ball ball2;
        ball2.p.x = 3.0f;
        ball2.p.y = 3.0f;
        ball2.v.x = -1.0f;
        ball2.v.y = 0.0f;
        ball2.radius = 0.4f;
        ball2.av = -3.0f;  // Initial angular velocity for visible rotation
        ball2.inertia = 1.0f;
        balls.push_back( ball2 );


        Uint64 lastCounter = SDL_GetPerformanceCounter();
        
        //The quit flag
        bool quit{ false };

        // accumulator for fixed-step physics
        double accumulator = 0.0;
        std::vector<Ball> previousBalls = balls;

        //The main loop
        while( quit == false )
        {
            Uint64 currentCounter = SDL_GetPerformanceCounter();
            Uint64 elapsed = currentCounter - lastCounter;
            lastCounter = currentCounter;

            double frameTime = static_cast<double>( elapsed ) / SDL_GetPerformanceFrequency();
            // clamp to avoid huge frame times after pauses
            frameTime = std::min( frameTime, kMaxFrameTime );

            accumulator += frameTime;

            handleEvents( quit );

            while( accumulator >= kFixedDt )
            {
                previousBalls = balls;
                for( Ball& b : balls ) {
                    updateBall( b, kFixedDt );
                }
                accumulator -= kFixedDt;
            }

            float alpha = static_cast<float>( accumulator / kFixedDt );

            clearScreen();
            for( size_t i = 0; i < balls.size(); ++i ) {
                Ball renderState = interpolateBall( previousBalls[i], balls[i], alpha );
                renderBall( renderState );
            }

            SDL_UpdateWindowSurface( gWindow );
        }
    }
    //Clean up
    close();

    return exitCode;
}
