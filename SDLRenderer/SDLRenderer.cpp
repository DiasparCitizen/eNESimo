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
uint32_t nesPalette[64];

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

	nesPalette[0x00] = getColor(84, 84, 84);
	nesPalette[0x01] = getColor(0, 30, 116);
	nesPalette[0x02] = getColor(8, 16, 144);
	nesPalette[0x03] = getColor(48, 0, 136);
	nesPalette[0x04] = getColor(68, 0, 100);
	nesPalette[0x05] = getColor(92, 0, 48);
	nesPalette[0x06] = getColor(84, 4, 0);
	nesPalette[0x07] = getColor(60, 24, 0);
	nesPalette[0x08] = getColor(32, 42, 0);
	nesPalette[0x09] = getColor(8, 58, 0);
	nesPalette[0x0A] = getColor(0, 64, 0);
	nesPalette[0x0B] = getColor(0, 60, 0);
	nesPalette[0x0C] = getColor(0, 50, 60);
	nesPalette[0x0D] = getColor(0, 0, 0);
	nesPalette[0x0E] = getColor(0, 0, 0);
	nesPalette[0x0F] = getColor(0, 0, 0);

	nesPalette[0x10] = getColor(152, 150, 152);
	nesPalette[0x11] = getColor(8, 76, 196);
	nesPalette[0x12] = getColor(48, 50, 236);
	nesPalette[0x13] = getColor(92, 30, 228);
	nesPalette[0x14] = getColor(136, 20, 176);
	nesPalette[0x15] = getColor(160, 20, 100);
	nesPalette[0x16] = getColor(152, 34, 32);
	nesPalette[0x17] = getColor(120, 60, 0);
	nesPalette[0x18] = getColor(84, 90, 0);
	nesPalette[0x19] = getColor(40, 114, 0);
	nesPalette[0x1A] = getColor(8, 124, 0);
	nesPalette[0x1B] = getColor(0, 118, 40);
	nesPalette[0x1C] = getColor(0, 102, 120);
	nesPalette[0x1D] = getColor(0, 0, 0);
	nesPalette[0x1E] = getColor(0, 0, 0);
	nesPalette[0x1F] = getColor(0, 0, 0);

	nesPalette[0x20] = getColor(236, 238, 236);
	nesPalette[0x21] = getColor(76, 154, 236);
	nesPalette[0x22] = getColor(120, 124, 236);
	nesPalette[0x23] = getColor(176, 98, 236);
	nesPalette[0x24] = getColor(228, 84, 236);
	nesPalette[0x25] = getColor(236, 88, 180);
	nesPalette[0x26] = getColor(236, 106, 100);
	nesPalette[0x27] = getColor(212, 136, 32);
	nesPalette[0x28] = getColor(160, 170, 0);
	nesPalette[0x29] = getColor(116, 196, 0);
	nesPalette[0x2A] = getColor(76, 208, 32);
	nesPalette[0x2B] = getColor(56, 204, 108);
	nesPalette[0x2C] = getColor(56, 180, 204);
	nesPalette[0x2D] = getColor(60, 60, 60);
	nesPalette[0x2E] = getColor(0, 0, 0);
	nesPalette[0x2F] = getColor(0, 0, 0);

	nesPalette[0x30] = getColor(236, 238, 236);
	nesPalette[0x31] = getColor(168, 204, 236);
	nesPalette[0x32] = getColor(188, 188, 236);
	nesPalette[0x33] = getColor(212, 178, 236);
	nesPalette[0x34] = getColor(236, 174, 236);
	nesPalette[0x35] = getColor(236, 174, 212);
	nesPalette[0x36] = getColor(236, 180, 176);
	nesPalette[0x37] = getColor(228, 196, 144);
	nesPalette[0x38] = getColor(204, 210, 120);
	nesPalette[0x39] = getColor(180, 222, 120);
	nesPalette[0x3A] = getColor(168, 226, 144);
	nesPalette[0x3B] = getColor(152, 226, 180);
	nesPalette[0x3C] = getColor(160, 214, 228);
	nesPalette[0x3D] = getColor(160, 162, 160);
	nesPalette[0x3E] = getColor(0, 0, 0);
	nesPalette[0x3F] = getColor(0, 0, 0);

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