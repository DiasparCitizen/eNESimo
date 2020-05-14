
#include "Game.h"
#include "../MyNESEmulator/NesCoreApi.h"

Game::Game()
{
	frameBuffer = new uint32_t[NES_RESOLUTION_WIDTH * NES_RESOLUTION_HEIGHT];
	_renderFrame = false;
	_handleEvents = false;
}

Game::~Game()
{
	delete frameBuffer;
}

void Game::init(const char* title, int xPos, int yPos, bool fullscreen)
{

	// Initialize
	std::string gamePath;

	if (0 != SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
		goto exit;
	}

	_window = SDL_CreateWindow(title, // window's title
		xPos, yPos, // coordinates on the screen, in pixels, of the window's upper left corner
		NES_RESOLUTION_WIDTH * 3, NES_RESOLUTION_HEIGHT * 3, // window's length and height in pixels
		SDL_WINDOW_OPENGL);

	if (_window) {
		std::cout << "Window created!\n";
	}
	else {
		goto exit;
	}

	SDL_SetWindowResizable(_window, SDL_TRUE);

	_renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED);
	if (_renderer) {
		std::cout << "Renderer created!\n";
	}
	else {
		goto exit;
	}

	_texture = SDL_CreateTexture(_renderer,
		SDL_PIXELFORMAT_RGBA8888,
		SDL_TEXTUREACCESS_STATIC,
		NES_RESOLUTION_WIDTH, NES_RESOLUTION_HEIGHT);


	// Disable Vsync
	//SDL_GL_SetSwapInterval(0);

	// Start NES

	gamePath = CARTRIDGE_NAME;
	gamePath = "../Games/" + gamePath;
	_cart = std::make_shared<Cartridge>(gamePath);
	_currentPixel = nes.getPtrToLastPixelDrawn();
	nes.insertCartridge(_cart);
	nes.resetNES();
	nes._controllers[0].setConnected(true);

	_isRunning = true;

	return;

exit:
	std::cout << "Error while constructing\n";

}

void Game::render()
{

	if (!_renderFrame) {
		return;
	}

	_renderFrame = false;

	SDL_UpdateTexture(_texture, NULL, frameBuffer, NES_RESOLUTION_WIDTH * 4 /* Pitch: num of bytes that make up a single row */);
	SDL_RenderCopy(_renderer, _texture, NULL, NULL);
	SDL_RenderPresent(_renderer);

}

void Game::handleEvents()
{

	if (!_handleEvents) {
		return;
	}

	_handleEvents = false;

	SDL_Event event;
	/* Poll for events */
	while (SDL_PollEvent(&event)) {

		//If a key was pressed
		if (event.type == SDL_KEYDOWN) {
			switch (event.key.keysym.sym) {
			case SDLK_DOWN:nes._controllers[0].setDOWN(true); break;
			case SDLK_UP: nes._controllers[0].setUP(true); break;
			case SDLK_LEFT: nes._controllers[0].setLEFT(true); break;
			case SDLK_RIGHT: nes._controllers[0].setRIGHT(true); break;
			case SDLK_a: nes._controllers[0].setA(true); break;
			case SDLK_b:nes._controllers[0].setB(true); break;
			case SDLK_s: nes._controllers[0].setStart(true); break;
			case SDLK_p: nes._controllers[0].setSelect(true); break;
			}
		}
		else if (event.type == SDL_KEYUP) {
			switch (event.key.keysym.sym) {
			case SDLK_DOWN:nes._controllers[0].setDOWN(false); break;
			case SDLK_UP: nes._controllers[0].setUP(false); break;
			case SDLK_LEFT: nes._controllers[0].setLEFT(false); break;
			case SDLK_RIGHT: nes._controllers[0].setRIGHT(false); break;
			case SDLK_a: nes._controllers[0].setA(false); break;
			case SDLK_b:nes._controllers[0].setB(false); break;
			case SDLK_s: nes._controllers[0].setStart(false); break;
			case SDLK_p: nes._controllers[0].setSelect(false); break;
			}
		}

	}

}

void Game::clean()
{
	SDL_DestroyWindow(_window);
	SDL_DestroyRenderer(_renderer);
	SDL_Quit();
	std::cout << "Game cleaned!\n";
}

void Game::update()
{

	currentTime = SDL_GetTicks();
	uint32_t deltaTime = currentTime - prevTime;

	if (deltaTime > frameTime) {
		prevTime = currentTime;
	}
	else {
		return;
	}

	do {

		nes.clockNES();

		int16_t x = _currentPixel->x - 1;
		int16_t y = _currentPixel->y;
		if (x >= 0 && x < NES_RESOLUTION_WIDTH && y >= 0 && y < NES_RESOLUTION_HEIGHT) {
			frameBuffer[y * NES_RESOLUTION_WIDTH + x] = _currentPixel->pixelVal;
		}

	} while (!nes._ppu._frameComplete);

	nes._ppu._frameComplete = false;
	_renderFrame = true;
	_handleEvents = true;

}

bool Game::running()
{
	return _isRunning;
}
