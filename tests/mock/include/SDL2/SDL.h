#ifndef MOCK_SDL_H
#define MOCK_SDL_H

/* Host stand-in for the few SDL types that src/input.h and src/input.c name. */

#include <stdint.h>

typedef int32_t SDL_Keycode;
typedef struct { SDL_Keycode sym; } SDL_Keysym;
typedef struct { uint32_t type; struct { SDL_Keysym keysym; } key; } SDL_Event;

#define SDL_KEYDOWN 0x300
#define SDL_KEYUP 0x301
#define SDLK_RETURN 13
#define SDLK_UP 1073741906
#define SDLK_DOWN 1073741905
#define SDLK_LEFT 1073741904
#define SDLK_RIGHT 1073741903

#endif
