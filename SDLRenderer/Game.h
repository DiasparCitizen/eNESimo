#pragma once


#include "SDL.h"
#include "stdio.h"
#include "../MyNESEmulator/Cartridge.h"
#include "SDLRendererConstants.h"
#include "../MyNESEmulator/Bus.h"

class Game {

public:
	Game();
	~Game();

	void init(const char* title, int xPos, int yPos, int w, int h, bool fullscreen);
	void render();
	void handleEvents();

	bool running();

private:
	bool _isRunning;
	SDL_Window* _window;
	SDL_Renderer* _renderer;
	SDL_Texture* _texture;

};