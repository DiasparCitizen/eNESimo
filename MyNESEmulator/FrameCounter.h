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

enum class sequence_step {
    STEP_1,
    STEP_2,
    STEP_3,
    STEP_4,
    STEP_4P5,
    STEP_5,
    CYCLE_0,
    NONE
};

// AKA frame sequencer
class FrameCounter {

public:
    FrameCounter() :
        cpuCycleCount(0),
        mode(0) {}

    sequence_step clock() {

        sequence_step step = sequence_step::NONE;

        if (mode == 0) {
            if (cpuCycleCount == 0) {
                step = sequence_step::CYCLE_0;
            }
            else if (cpuCycleCount == 7457 /*3728.5*/) { // Step 1
                step = sequence_step::STEP_1;
            }
            else if (cpuCycleCount == 14913 /*7456.5*/) { // Step 2
                step = sequence_step::STEP_2;
            }
            else if (cpuCycleCount == 22371 /*11185.5*/) { // Step 3
                step = sequence_step::STEP_3;
            }
            else if (cpuCycleCount == 29828 /*14914.0*/) { // Step 4
                step = sequence_step::STEP_4;
            }
            else if (cpuCycleCount == 29829 /*14914.5*/) { // Step 4.5
                step = sequence_step::STEP_4P5;
                cpuCycleCount = 0;
            }
        }
        else { // Mode 1: 5 steps
            if (cpuCycleCount == 0) {
                step = sequence_step::CYCLE_0;
            }
            else if (cpuCycleCount == 7457 /*3728.5*/) { // Step 1
                step = sequence_step::STEP_1;
            }
            else if (cpuCycleCount == 14913 /*7456.5*/) { // Step 2
                step = sequence_step::STEP_2;
            }
            else if (cpuCycleCount == 22371 /*11185.5*/) { // Step 3
                step = sequence_step::STEP_3;
            }
            else if (cpuCycleCount == 29829 /*14914.5*/) { // Step 4
                step = sequence_step::STEP_4;
            }
            else if (cpuCycleCount == 37281 /*18640.5*/) { // Step 5
                step = sequence_step::STEP_5;
                cpuCycleCount = 0;
            }
        }

        cpuCycleCount++;

        return step;

    }

    uint16_t cpuCycleCount;
    uint8_t mode;

};