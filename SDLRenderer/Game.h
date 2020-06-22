/*
Copyright (C) 2020 Ismael García-Marlowe

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#pragma once

#include <SDL.h>
#include <SDL_audio.h>

#include "stdio.h"
#include <math.h>

#include "../MyNESEmulator/Cartridge.h"
#include "SDLRendererConstants.h"
#include "../MyNESEmulator/Bus.h"

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
	void queueNewSample();

private:
	Bus _nes;

	bool _isRunning;
	pixel_st* _currentPixel;
	uint32_t* frameBuffer;

	std::shared_ptr<Cartridge> _cart;

	SDL_Window* _window;
	SDL_Renderer* _renderer;
	SDL_Texture* _texture;

	double _currentTime;
	double _prevTime;
	double _remainderTime;

	bool _renderFrame;
	bool _handleEvents;

	// Sound
	int16_t _sampleBuffer[800];
	uint16_t _nextSampleBufferIdx = 0;
	int sample_nr = 0;
	SDL_AudioDeviceID _audioDevice;
	uint32_t samplesTakenCounter = 0;

#ifdef GAME_FILE_LOG
	// Log file
	std::ofstream gameLogFile;
#endif

};