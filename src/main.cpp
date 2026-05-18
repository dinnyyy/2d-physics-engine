/* Headers */
//Using SDL and STL string
#include <SDL3/SDL_main.h>
#include "main.h"
#include "physics.h"
#include <string>

SDL_Window* gWindow{ nullptr };
SDL_Surface* gScreenSurface{ nullptr };
SDL_Surface* gHelloWorld{ nullptr };

constexpr double kMaxDeltaTime{ 1.0 / 30.0 };

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
    SDL_Rect rect
    {
        metersToPixelInt( ball.x - ball.radius ),
        metersToPixelInt( ball.y - ball.radius ),
        metersToPixelInt( ball.radius * 2.0f ),
        metersToPixelInt( ball.radius * 2.0f )
    };

    Uint32 black = SDL_MapSurfaceRGB( gScreenSurface, 0, 0, 0 );

    SDL_FillSurfaceRect( gScreenSurface, &rect, black );
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
        Ball ball;

        ball.x = 1.0f;
        ball.y = 1.0f;

        ball.vx = 1.0f;
        ball.vy = 0.0f;
        ball.radius = 0.4f;

        Uint64 lastCounter = SDL_GetPerformanceCounter();
        
        //The quit flag
        bool quit{ false };

        //The event data
        //The main loop
        while( quit == false )
        {

            Uint64 currentCounter = SDL_GetPerformanceCounter();

            Uint64 elapsed = currentCounter - lastCounter;

            lastCounter = currentCounter;

            double dt = static_cast<double>( elapsed ) / SDL_GetPerformanceFrequency();
            if( dt > kMaxDeltaTime )
            {
                dt = kMaxDeltaTime;
            }

            handleEvents( quit );

            updateBall( ball, dt );

            clearScreen();

            renderBall( ball );

            SDL_UpdateWindowSurface( gWindow );
        }
    }
    //Clean up
    close();

    return exitCode;
}