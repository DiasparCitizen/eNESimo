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
#include "LengthCounter.h"
#include "LinearCounter.h"

constexpr uint8_t triangleSequencerLut[] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

/*
----------------
Triangle Channel
----------------

                   +---------+    +---------+
                   |LinearCtr|    | Length  |
                   +---------+    +---------+
                        |              |
                        v              v
    +---------+        |\             |\         +---------+    +---------+
    |  Timer  |------->| >----------->| >------->|Sequencer|--->|   DAC   |
    +---------+        |/             |/         +---------+    +---------+
*/
class TriangleWaveEngine {

public:
    TriangleWaveEngine() :
        timer(0),
        configuredPeriod(0),
        sequencerOffset(0),
        output(0),
        linearCounterUnit(),
        lengthCounterUnit() {}

    void reloadTimer() {
        // NESDEV: "The sequencer is clocked by a timer whose period is the 11-bit
        // value (%HHH.LLLLLLLL) formed by timer high and timer low, plus one."
        // To always count one more tick than configured, we clock at -1.
        timer = configuredPeriod;
    }

    void configureTimerLo(uint8_t timerLo) {
        configuredPeriod &= 0x700;
        configuredPeriod |= timerLo;
        reloadTimer();
    }

    void configureTimerHi(uint8_t timerHi) {
        configuredPeriod &= 0xFF;
        configuredPeriod |= (timerHi << 8);
    }

    void clock() {

        if (timer < 0) {
            reloadTimer();

            if (configuredPeriod < 2)
                return;

            // The sequencer is clocked by the timer as long as both the
            // linear counter and the length counter are nonzero.
            if (linearCounterUnit.counter != 0 && lengthCounterUnit.divider != 0) {
                output = triangleSequencerLut[sequencerOffset];
                sequencerOffset = (sequencerOffset + 1) % 32;
            }
        }
        else {
            timer--;
        }

    }

    void clockLengthCounterUnit() {
        lengthCounterUnit.clock();
    }

    void clockLinearCounterUnit() {
        linearCounterUnit.clock();
    }

private:
    int16_t timer;
    int16_t configuredPeriod;

    uint8_t sequencerOffset = 0;
    uint8_t output;

    LinearCounterUnit linearCounterUnit;
    LengthCounterUnit lengthCounterUnit;

    friend class APU;

};