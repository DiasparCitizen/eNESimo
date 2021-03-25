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
#include "Envelope.h"
#include "LengthCounter.h"

constexpr uint16_t noiseTimerPeriodLut[] = { 0x004, 0x008, 0x010, 0x020, 0x040, 0x060, 0x080, 0x0A0, 0x0CA, 0x0FE, 0x17C, 0x1FC, 0x2FA, 0x3F8, 0x7F2, 0xFE4 };

/*
-------------
Noise Channel
-------------

    +---------+    +---------+    +---------+
    |  Timer  |--->| Random  |    | Length  |
    +---------+    +---------+    +---------+
                        |              |
                        v              v
    +---------+        |\             |\         +---------+
    |Envelope |------->| >----------->| >------->|   DAC   |
    +---------+        |/             |/         +---------+
*/
// https://wiki.nesdev.com/w/index.php/APU_Noise
class NoiseWaveEngine {

public:
    NoiseWaveEngine() {
        shiftRegister = 0;
        modeFlag = false;
        // On power-up, the shift register is loaded with the value 1.
        shiftRegister = 0x1;
        timer = 0;
        configuredPeriod = 0;
        feedback = 0;
        envelopeUnit = {};
        lengthCounterUnit = {};
    }

    void configureTimer(uint8_t config) {
        configuredPeriod = noiseTimerPeriodLut[config];
    }

    void reloadTimer() {
        timer = configuredPeriod;
    }

    void clock() {

        if (timer == 0) {

            reloadTimer();

            // Feedback is calculated as the exclusive-OR of bit 0 and one other
            // bit: bit 6 if Mode flag is set, otherwise bit 1.

            feedback = shiftRegister & 0x1;

            uint8_t bit2 = modeFlag ? (shiftRegister >> 6) : (shiftRegister >> 1);
            bit2 &= 0x1;

            feedback = feedback ^ bit2; // XOR operation

            // The shift register is shifted right by one bit.

            shiftRegister >>= 1; // Shift right 1 bit

            // Bit 14, the leftmost bit, is set to the feedback calculated earlier.

            shiftRegister &= ~(0x1 << 14);
            shiftRegister |= (feedback << 14);

        }
        else {
            timer--;
        }

    }

    void clockLengthCounterUnit() {
        lengthCounterUnit.clock();
    }

    void clockEnvelopeUnit() {
        envelopeUnit.clock();
    }

private:
    uint16_t timer;
    uint16_t configuredPeriod;

    EnvelopeUnit envelopeUnit;
    LengthCounterUnit lengthCounterUnit;

    bool modeFlag;

    uint16_t shiftRegister;
    uint8_t feedback;

    friend class APU;
};