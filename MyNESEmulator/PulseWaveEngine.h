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
#include "Envelope.h"
#include "Sweep.h"

/*
--------------
Square Channel
--------------

                   +---------+    +---------+
                   |  Sweep  |--->|Timer / 2|
                   +---------+    +---------+
                        |              |
                        |              v
                        |         +---------+    +---------+
                        |         |Sequencer|    | Length  |
                        |         +---------+    +---------+
                        |              |              |
                        v              v              v
    +---------+        |\             |\             |\          +---------+
    |Envelope |------->| >----------->| >----------->| >-------->|   DAC   |
    +---------+        |/             |/             |/          +---------+
*/
class PulseWaveEngine {

public:
    PulseWaveEngine() :
        timer(0),
        configuredPeriod(0),
        waveForm(0),
        waveFormOffs(0),
        output(0),
        lengthCounterUnit(),
        sweepUnit(),
        envelopeUnit() {}

    void init(uint8_t id) {
        sweepUnit.init(&timer, &configuredPeriod, id);
    }

    void configureTimerLo(uint8_t timerLo) {
        configuredPeriod &= 0x700;
        configuredPeriod |= timerLo;
        timer = (configuredPeriod << 1) + 1;
        sweepUnit.updateTargetPeriod();
    }

    void configureTimerHi(uint8_t timerHi) {
        configuredPeriod &= 0xFF;
        configuredPeriod |= (timerHi << 8);
        timer = (configuredPeriod << 1) + 1;
        sweepUnit.updateTargetPeriod();
    }

    void restartSequencer() {
        waveFormOffs = 0;
    }

    void clock() {

        timer--;
        if (timer == 0xFFFF) { // When -1 is reached
            // Reload time
            timer = (configuredPeriod << 1) + 1;
            // Advance sequence step (really an index to the wave form sequence)
            waveFormOffs = (waveFormOffs + 1) & 0x7;
            output = (waveForm >> waveFormOffs) & 0x1;
        }

    }

    void clockLengthCounterUnit() {
        lengthCounterUnit.clock();
    }

    void clockSweepUnit() {
        sweepUnit.clock();
    }

    void clockEnvelopeUnit() {
        envelopeUnit.clock();
    }

private:
    // Timer unit
    uint16_t timer; // Active timer, derived from configuredTimer
    uint16_t configuredPeriod; // Value as directly set by CPU

    // Sequencer unit
    // Easier if not encapsulated in its own struct
    uint8_t waveForm;
    uint8_t waveFormOffs; // An offset to a bit in waveForm
    uint8_t output;

    // Remaining units
    LengthCounterUnit lengthCounterUnit;
    SweepUnit sweepUnit;
    EnvelopeUnit envelopeUnit;

    friend class APU;

};