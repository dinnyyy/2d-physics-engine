#pragma once

#include <SDL3/SDL.h>

/* Constants */
//Screen dimension constants
constexpr int kScreenWidth{ 1000 };
constexpr int kScreenHeight{ 1000 };

/* Function Prototypes */
//Starts up SDL and creates window
bool init();

//Loads media
bool loadMedia();

//Frees media and shuts down SDL
void close();

/* Global Variables */
//The window we'll be rendering to
extern SDL_Window* gWindow;
    
//The surface contained by the window
extern SDL_Surface* gScreenSurface;

//The image we will load and show on the screen
extern SDL_Surface* gHelloWorld;
