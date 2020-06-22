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
#include "Game.h"
#include "../MyNESEmulator/NesCoreApi.h"

#if defined(GAME_TERMINAL_LOG) && defined(GAME_FILE_LOG)
#define _LOG(txt) \
{ \
std::stringstream myStream; \
myStream << txt; \
std::cout << myStream.str(); \
gameLogFile << myStream.str(); \
} 
#elif defined(GAME_TERMINAL_LOG)
#define _LOG(txt) std::cout << txt
#elif defined(GAME_FILE_LOG)
#define _LOG(txt) gameLogFile << txt
#else
#define _LOG(txt) ;
#endif

Game::Game() {
    frameBuffer = new uint32_t[NES_RESOLUTION_WIDTH * NES_RESOLUTION_HEIGHT];
    _renderFrame = false;
    _handleEvents = false;

#ifdef GAME_FILE_LOG
    gameLogFile.open("game_log.txt");
#endif
}

Game::~Game() {
    delete frameBuffer;
#ifdef GAME_FILE_LOG
    gameLogFile.close();
#endif
}

void Game::init(const char* title, int xPos, int yPos, bool fullscreen) {

    // Initialize
    std::string gamePath;

    if (0 != SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
        SDL_Log("Failed to initialize VIDEO & EVENTS: %s", SDL_GetError());
        goto exit;
    }

    _putenv("SDL_AUDIODRIVER=DirectSound");
    if (0 != SDL_InitSubSystem(SDL_INIT_AUDIO)) {
        SDL_Log("Failed to initialize AUDIO: %s", SDL_GetError());
        goto exit;
    }

    // VIDEO

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


    // AUDIO

    SDL_AudioSpec want;
    want.freq = SAMPLE_RATE; // number of samples per second
    want.format = AUDIO_S16SYS; // sample type (here: signed short i.e. 16 bit)
    want.channels = 1; // only one channel
    want.samples = 512;
    want.callback = NULL;
    want.userdata = NULL;
    want.size = want.samples * 2;

    SDL_AudioSpec have;
    if (SDL_OpenAudio(&want, &have) != 0) {
        SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Failed to open audio: %s", SDL_GetError());
        goto exit;
    }
    if (want.format != have.format) {
        SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Failed to get the desired AudioSpec");
        goto exit;
    }

    _audioDevice = SDL_OpenAudioDevice(NULL, 0, &want, &have, SDL_AUDIO_ALLOW_ANY_CHANGE);
    if (_audioDevice == 0) {
        SDL_Log("Failed to open audio dev: %s", SDL_GetError());
        goto exit;
    }

    // Start NES

    gamePath = CARTRIDGE_NAME;
    gamePath = "../Games/" + gamePath;
    _cart = std::make_shared<Cartridge>(gamePath);
    _currentPixel = _nes.getPtrToLastPixelDrawn();
    _nes.insertCartridge(_cart);
    _nes.resetNES();
    _nes._controllers[0].setConnected(true);

    _isRunning = true;

    SDL_PauseAudioDevice(_audioDevice, 0); // Let audio run

    return;

exit:
    std::cout << "Error while constructing\n";

}

void Game::render() {

    if (!_renderFrame) {
        return;
    }

    _renderFrame = false;

    SDL_UpdateTexture(_texture, NULL, frameBuffer, NES_RESOLUTION_WIDTH * 4 /* Pitch: num of bytes that make up a single row */);
    SDL_RenderCopy(_renderer, _texture, NULL, NULL);
    SDL_RenderPresent(_renderer);

}

void Game::handleEvents() {

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
            case SDLK_DOWN:_nes._controllers[0].setDOWN(true); break;
            case SDLK_UP: _nes._controllers[0].setUP(true); break;
            case SDLK_LEFT: _nes._controllers[0].setLEFT(true); break;
            case SDLK_RIGHT: _nes._controllers[0].setRIGHT(true); break;
            case SDLK_a: _nes._controllers[0].setA(true); break;
            case SDLK_b:_nes._controllers[0].setB(true); break;
            case SDLK_s: _nes._controllers[0].setStart(true); break;
            case SDLK_p: _nes._controllers[0].setSelect(true); break;
            }
        }
        else if (event.type == SDL_KEYUP) {
            switch (event.key.keysym.sym) {
            case SDLK_DOWN:_nes._controllers[0].setDOWN(false); break;
            case SDLK_UP: _nes._controllers[0].setUP(false); break;
            case SDLK_LEFT: _nes._controllers[0].setLEFT(false); break;
            case SDLK_RIGHT: _nes._controllers[0].setRIGHT(false); break;
            case SDLK_a: _nes._controllers[0].setA(false); break;
            case SDLK_b:_nes._controllers[0].setB(false); break;
            case SDLK_s: _nes._controllers[0].setStart(false); break;
            case SDLK_p: _nes._controllers[0].setSelect(false); break;
            }
        }

    }

}

void Game::clean() {
    // Audio
    SDL_PauseAudioDevice(_audioDevice, 1);
    SDL_CloseAudio();
    SDL_CloseAudioDevice(_audioDevice);

    SDL_DestroyWindow(_window);
    SDL_DestroyRenderer(_renderer);
    SDL_Quit();
    std::cout << "Game cleaned!\n";
}

void Game::queueNewSample() {

    if (_nes.areNewSamplesAvailable()) {

        uint16_t numSamples;
        sample_st* newSamplesArray = _nes.getPtrToNewSamples(numSamples);

        sample_t sampleArray[10];
        sampleArray[0] = newSamplesArray[0].sample;

        if (numSamples) {
            _LOG(newSamplesArray[0].time << " " << newSamplesArray[0].sample << std::endl);
        }

        if (SDL_QueueAudio(_audioDevice, sampleArray, sizeof(sample_t)) == 0) {
        }
        else {
            SDL_Log("Device FAILED to queue %u more bytes: %s\n", (numSamples * sizeof(sample_t)));
        }
        samplesTakenCounter++;

    }
}


void Game::update() {

    _currentTime = (double)SDL_GetTicks();
    double deltaTime = _currentTime - _prevTime;

    if (deltaTime > NES_FRAME_PERIOD) {
        _prevTime = _currentTime;
        _remainderTime += (deltaTime - NES_FRAME_PERIOD);
    }
    else if (_remainderTime > NES_FRAME_PERIOD) {
        _remainderTime -= NES_FRAME_PERIOD;
    }
    else {
        return;
    }

    do {

        _nes.clockNES();
        queueNewSample();

        int16_t x = _currentPixel->x;
        int16_t y = _currentPixel->y;
        if (x >= 0 && x < NES_RESOLUTION_WIDTH && y >= 0 && y < NES_RESOLUTION_HEIGHT) {
            frameBuffer[y * NES_RESOLUTION_WIDTH + x] = _currentPixel->pixelVal;
        }

    } while (!_nes._ppu._frameComplete);

    samplesTakenCounter = 0;

    _nes._ppu._frameComplete = false;
    _renderFrame = true;
    _handleEvents = true;

}

bool Game::running() {
    return _isRunning;
}