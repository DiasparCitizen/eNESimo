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

constexpr uint16_t DECAY_COUNTER_MAX_VAL = 15;

// https://wiki.nesdev.com/w/index.php/APU_Envelope
class EnvelopeUnit {

public:
    EnvelopeUnit() :
        startFlag(false),
        loopFlag(false),
        constantVolumeFlag(false),
        volume(0),
        envelopeDivider(0),
        decayLevelCounter(0) {}

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
                else if (loopFlag) {
                     decayLevelCounter = DECAY_COUNTER_MAX_VAL;
                }

            }
            else {
                envelopeDivider--;
            }

        }
        else {
            startFlag = false;
            decayLevelCounter = DECAY_COUNTER_MAX_VAL;
            envelopeDivider = volume;
        }

    }

private:
    bool startFlag;
    bool loopFlag;
    bool constantVolumeFlag;

    uint16_t volume; // == period
    uint16_t envelopeDivider;
    uint16_t decayLevelCounter;

    friend class APU;

};