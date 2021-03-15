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

#include "CommonLibs.h"

class EnvelopeUnit {

public:
    uint16_t getVolume() {
        return constantVolumeFlag ? volume : decayLevelCounter;
    }

    void clock() {

        if (!startFlag) { // Reviewed, fine

            if (envelopeDivider == 0) {

                envelopeDivider = volume;

                if (decayLevelCounter > 0) {
                    decayLevelCounter--;
                }
                else {
                    if (loopFlag) decayLevelCounter = 15;
                }

            }
            else {
                envelopeDivider--;
            }

        }
        else {
            startFlag = false;
            decayLevelCounter = 15;
            envelopeDivider = volume;
        }

    }

    bool startFlag;
    bool loopFlag;
    bool constantVolumeFlag;

    uint16_t volume; // == period
    uint16_t envelopeDivider;

    uint16_t decayLevelCounter;

};