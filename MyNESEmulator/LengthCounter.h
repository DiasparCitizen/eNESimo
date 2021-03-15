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

// Already +1; the length counter mutes the channel when clocked *while 0*
constexpr uint16_t lengthCounterLut[] = { 10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
                                        12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30 };

class LengthCounterUnit {

public:
    LengthCounterUnit() {
        divider = 0;
        enabled = false;
        halt = false;
    }

    void configureDivider(uint8_t config) {
        // When the enabled bit is cleared (via $4015), the length counter is forced to 0
        // and cannot be changed until enabled is set again (the length counter's
        // previous value is lost). There is no immediate effect when enabled is set.
        if (enabled) {
            divider = lengthCounterLut[config];
        }
    }

    void setEnabled(bool en) {
        // Writing a zero to any of the channel enable bits will silence that channel
        // and immediately set its length counter to 0.
        enabled = en;
        if (!en) {
            divider = 0;
        }
    }

    void clock() {

        if (divider > 0 && !halt) {
            divider--;
        }

    }

    uint16_t divider;
    bool enabled;
    bool halt; // Halt flag

};