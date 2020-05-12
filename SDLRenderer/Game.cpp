
#include "Game.h"
#include "../MyNESEmulator/NesCoreApi.h"

Game::Game()
{
}

Game::~Game()
{
}

void Game::init(const char* title, int xPos, int yPos, int w, int h, bool fullscreen)
{
	if (0 == SDL_Init(SDL_INIT_VIDEO)) {
		return;
	}

	_window = SDL_CreateWindow(title, // window's title
		xPos, yPos, // coordinates on the screen, in pixels, of the window's upper left corner
		w * 3, h * 3, // window's length and height in pixels
		SDL_WINDOW_OPENGL);

	SDL_SetWindowResizable(_window, SDL_TRUE);

	_renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED);

	_texture = SDL_CreateTexture(_renderer,
		SDL_PIXELFORMAT_RGBA8888,
		SDL_TEXTUREACCESS_STATIC,
		w, h);

	// Start NES

}

void Game::render()
{

}

void Game::handleEvents()
{
}

bool Game::running()
{
	return _isRunning;
}
