// SDLRenderer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

// Guide: https://www.wikihow.com/Set-Up-SDL-with-Visual-Studio
#include "SDL.h"
#include "SDLRendererConstants.h"
#include "Game.h"

// After the include!
#define SDL_MAIN_HANDLED

// https://gamedev.stackexchange.com/questions/136055/why-doesnt-sdl-surface-rendering-work-in-sdl2
// https://forums.libsdl.org/viewtopic.php?p=22987
int main(int argc, char* argv[])
{

	Game* game = new Game();
	game->init("NES", 0, 0, false);
	while (game->running()) {
		//std::cout << "loop!\n";
		game->handleEvents();
		game->update();
		game->render();
	}

	return 0;

}