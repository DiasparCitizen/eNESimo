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
#include <iostream>

// Guide: https://www.wikihow.com/Set-Up-SDL-with-Visual-Studio
#include "SDL.h"
#include "SDLRendererConstants.h"
#include "Game.h"

// After the include!
#define SDL_MAIN_HANDLED

// https://gamedev.stackexchange.com/questions/136055/why-doesnt-sdl-surface-rendering-work-in-sdl2
// https://forums.libsdl.org/viewtopic.php?p=22987
int main(int argc, char* argv[]) {

    Game* game = new Game();
    game->init("NES", 100, 100, false);
    while (game->running()) {
        game->handleEvents();
        game->update();
        game->render();
    }

    return 0;

}