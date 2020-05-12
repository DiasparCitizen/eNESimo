// SDLRenderer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

// Guide: https://www.wikihow.com/Set-Up-SDL-with-Visual-Studio
#include "SDL.h"
#include "../MyNESEmulator/NesCoreApi.h"
#include "../MyNESEmulator/Cartridge.h"
#include "SDLRendererConstants.h"
#include "../MyNESEmulator/Bus.h"

// After the include!
#define SDL_MAIN_HANDLED

// Globals
SDL_Window* _window;
SDL_Renderer* _renderer;
SDL_Surface* _surface;
SDL_Texture* _texture;

pixel_st* _currPixel;
std::shared_ptr<Cartridge> cart;
Bus nes;

uint32_t* frameBuffer;

constexpr int nes_w = 236;
constexpr int nes_h = 240;

uint32_t getColor(int r, int g, int b) {
	uint32_t colorCode = b << 8;
	colorCode |= g << 16;
	colorCode |= r << 24;
	return colorCode;
}

void startNES() {

	// Load cartridge
	std::string gamePath = CARTRIDGE_NAME;
	gamePath = "../Games/" + gamePath;
	cart = std::make_shared<Cartridge>(gamePath);
	_currPixel = nes.getPtrToLastPixelDrawn();
	nes.insertCartridge(cart);
	nes.resetNES();


	frameBuffer = new uint32_t[nes_w * nes_h];

}

void runNES() {

	nes.clockNES();

	int16_t x = _currPixel->x - 1;
	int16_t y = _currPixel->y;
	if (x >= 0 && x < nes_w && y >= 0 && y < nes_h) {
		frameBuffer[y * nes_w + x] = _currPixel->pixelVal;
	}

	if (nes._ppu._frameComplete) {
		nes._ppu._frameComplete = false;

		// RENDER

		SDL_UpdateTexture(_texture, NULL, frameBuffer, nes_w * 4 /* Pitch: num of bytes that make up a single row */);
		SDL_RenderCopy(_renderer, _texture, NULL, NULL);
		SDL_RenderPresent(_renderer);

	}

}

// https://gamedev.stackexchange.com/questions/136055/why-doesnt-sdl-surface-rendering-work-in-sdl2
// https://forums.libsdl.org/viewtopic.php?p=22987
int main(int argc, char* argv[])
{

	SDL_Init(SDL_INIT_VIDEO);

	_window = SDL_CreateWindow("An SDL2 window", // window's title
		10, 25, // coordinates on the screen, in pixels, of the window's upper left corner
		nes_w * 3, nes_h * 3, // window's length and height in pixels
		SDL_WINDOW_OPENGL);

	SDL_SetWindowResizable(_window, SDL_TRUE);

	_renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED);

	_texture = SDL_CreateTexture(_renderer,
		SDL_PIXELFORMAT_RGBA8888,
		SDL_TEXTUREACCESS_STATIC,
		nes_w, nes_h);


	startNES();
    bool done = false;
    while (!done){
		runNES();
	}



	SDL_DestroyWindow(_window);
	SDL_Quit();
	return 0;

}