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

// https://wiki.nesdev.com/w/index.php/APU_Sweep
class SweepUnit {

public:
    SweepUnit() :
        pulseChId(0),
        sweepDivider(0),
        sweepPeriod(0),
        shiftCount(0),
        reloadFlag(false),
        enabledFlag(false),
        negateFlag(false),
        chTimer(nullptr),
        chConfiguredPeriod(nullptr),
        chTargetPeriod(0) {}

    void init(uint16_t* timer, uint16_t* configuredPeriod, uint8_t id) {
        // Set values from outside
        chTimer = timer;
        chConfiguredPeriod = configuredPeriod;
        pulseChId = id;
        // Initialize other values
        enabledFlag = true;
        negateFlag = false;
    }

    bool isMuted() {
        // If the current period is less than 8, the sweep unit mutes the channel.
        // This avoids sending harmonics in the hundreds of kHz through the audio path.
        // Muting based on a too-small current period cannot be overridden.
        return *chConfiguredPeriod < 8 || (!negateFlag && chTargetPeriod > 0x7FF);
    }

    void updateTargetPeriod() {

        uint16_t changeAmount = *chConfiguredPeriod >> shiftCount;

        if (negateFlag) {
            chTargetPeriod = *chConfiguredPeriod - changeAmount - pulseChId;
        }
        else {
            chTargetPeriod = *chConfiguredPeriod + changeAmount;
        }

    }

    void clock() {

        // If the divider's counter is zero, the sweep is enabled, and the sweep
        // unit is not muting the channel: The pulse's period is adjusted.
        if (sweepDivider == 0) {

            if (shiftCount && enabledFlag && !isMuted()) {

                *chConfiguredPeriod = chTargetPeriod;
                *chTimer = (*chConfiguredPeriod << 1) + 1;
                updateTargetPeriod();

                sweepDivider = sweepPeriod;

            }

        }
        else {
            sweepDivider--;
        }

        if (reloadFlag) {
            // Restart count for internal divider
            sweepDivider = sweepPeriod;
            reloadFlag = false;
        }

    }

private:
    uint8_t pulseChId;

    uint8_t sweepDivider;
    uint8_t sweepPeriod;
    uint8_t shiftCount; // Shift count (number of bits)

    bool reloadFlag;
    bool enabledFlag;
    // 0: add to period, sweeping toward lower frequencies
    // 1: subtract from period, sweeping toward higher frequencies
    bool negateFlag;

    uint16_t* chTimer;
    uint16_t* chConfiguredPeriod; // This is the value written by the CPU
    uint16_t chTargetPeriod;

    friend class APU;

};