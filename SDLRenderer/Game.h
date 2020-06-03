#pragma once

#include <SDL.h>
#include <SDL_audio.h>

#include "stdio.h"
#include <math.h>

#include "../MyNESEmulator/Cartridge.h"
#include "SDLRendererConstants.h"
#include "../MyNESEmulator/Bus.h"

constexpr double frameTime = 1000.0 / 60.1;

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