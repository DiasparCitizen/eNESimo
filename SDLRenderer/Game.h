#pragma once


#include "SDL.h"
#include "stdio.h"
#include "../MyNESEmulator/Cartridge.h"
#include "SDLRendererConstants.h"
#include "../MyNESEmulator/Bus.h"

constexpr double frameTime = 1000.0 / 60.0;

class Game {

public:
	Game();
	~Game();

	void init(const char* title, int xPos, int yPos, bool fullscreen);
	void render();
	void handleEvents();
	void clean();
	void update();
	bool running();

private:
	Bus nes;

	bool _isRunning;
	pixel_st* _currentPixel;
	uint32_t* frameBuffer;

	std::shared_ptr<Cartridge> _cart;

	SDL_Window* _window;
	SDL_Renderer* _renderer;
	SDL_Texture* _texture;

	double currentTime;
	double prevTime;

	bool _renderFrame;
	bool _handleEvents;

};