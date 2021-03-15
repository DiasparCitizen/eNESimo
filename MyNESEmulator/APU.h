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

#include <cstdint>
#include "CommonLibs.h"
#include "NESConstants.h"
#include "PulseWaveEngine.h"
#include "TriangleWaveEngine.h"
#include "NoiseWaveEngine.h"
#include "LengthCounter.h"
#include "LinearCounter.h"
#include "Sweep.h"
#include "Envelope.h"
#include "FrameCounter.h"

// http://www.slack.net/~ant/nes-emu/apu_ref.txt

// Audio emulation approaches - NESDEV forum
// https://forums.nesdev.com/viewtopic.php?t=10048
// https://forums.nesdev.com/viewtopic.php?t=18493

// Great thread:
// https://forums.nesdev.com/viewtopic.php?f=5&t=15383

// Blip-buf library
// https://code.google.com/archive/p/blip-buf/

// http://forums.nesdev.com/viewtopic.php?f=3&t=3808

constexpr uint8_t waveForms[] = { 0b01000000, 0b01100000, 0b01111000, 0b10011111 };

////////////////////////////////////////////
///// APU registers (CPU-APU interface)
////////////////////////////////////////////

struct pulse_wave_reg1_st { // 0x4000
    uint8_t volume : 4; // Volume
    uint8_t constantVolumeFlag : 1; // Constant volume flag (0: use volume from envelope; 1: use constant volume)
    uint8_t lengthCounterHalt : 1; // Length counter halt, loop flag for the envelope
    uint8_t duty : 2; // Duty
};

struct pulse_wave_reg2_st { // 0x4001, Sweep Unit
    uint8_t shiftCount : 3;
    uint8_t negate : 1; // 0: add to period, sweeping toward lower frequencies; 1: subtract from period, sweeping toward higher frequencies
    uint8_t period : 3; // Period of the wave
    uint8_t enabled : 1;
};

struct pulse_wave_reg3_st { // 0x4002
    uint8_t timerLo;
};

struct pulse_wave_reg4_st { // 0x4003
    uint8_t timerHi : 3; // 3 MSB of the timer
    uint8_t lengthCounterLoad : 5; // Value we get from the length table
};

struct triangle_wave_reg1_st { // 0x4008
    uint8_t linearCounterLoad : 7;
    uint8_t lengthCounterHaltAndLinearCounterControl : 1;
};

struct triangle_wave_reg3_st { // 0x400A
    uint8_t timerLo;
};

struct triangle_wave_reg4_st { // 0x400B
    uint8_t timerHi : 3;
    uint8_t lengthCounterLoad : 5;
};

struct noise_reg1_st { // 0x400C
    uint8_t volume : 4;
    uint8_t constantVolume : 1;
    uint8_t envelopeLoopAndLengthCounterHalt : 1;
    // 2 bits unused
};

struct noise_reg3_st { // 0x400E
    uint8_t noisePeriod : 4;
    uint8_t unused : 3;
    uint8_t mode : 1;
};

struct noise_reg4_st { // 0x400F
    uint8_t unused : 3;
    uint8_t lengthCounterLoad : 5;
};

struct status_wr_reg_st { // 0x4015
    uint8_t enablePulseCh1 : 1;
    uint8_t enablePulseCh2 : 1;
    uint8_t enableTriangleChl : 1;
    uint8_t enableNoiseChl : 1;
    uint8_t enableDMC : 1;
    uint8_t unused : 3;
};

struct status_rd_reg_st { // 0x4015
    uint8_t pulseCh1LenCntActive : 1;
    uint8_t pulseCh2LenCntActive : 1;
    uint8_t triangleChLenCntActive : 1;
    uint8_t noiseChLenCntActive : 1;
    uint8_t dmcActive : 1;
    uint8_t unused : 1;
    uint8_t frameInterrupt : 1;
    uint8_t dmcInterrupt : 1;
};

struct frame_counter_st { // 0x4017
    uint8_t unused : 6;
    uint8_t mode : 1; // (M, 0 = 4-step, 1 = 5-step)
    uint8_t irqInhibit : 1; // IRQ (if bit 6 is clear)
};




// Forward declare
class Bus;

class APU {

public:
    void reset();
    void clock();
    void connectConsole(Bus* bus);

    void writePulseWave1Reg1(uint8_t data);
    void writePulseWave1Reg2(uint8_t data);
    void writePulseWave1Reg3(uint8_t data);
    void writePulseWave1Reg4(uint8_t data);

    void writePulseWave2Reg1(uint8_t data);
    void writePulseWave2Reg2(uint8_t data);
    void writePulseWave2Reg3(uint8_t data);
    void writePulseWave2Reg4(uint8_t data);

    void setPulseWaveReg1Fields(uint8_t id, pulse_wave_reg1_st data);
    void setPulseWaveReg2Fields(uint8_t id, pulse_wave_reg2_st data);
    void setPulseWaveReg3Fields(uint8_t id, uint8_t data);
    void setPulseWaveReg4Fields(uint8_t id, pulse_wave_reg4_st data);

    void writeTriangleWaveReg1(uint8_t data);
    void writeTriangleWaveReg2(uint8_t data); // Unused
    void writeTriangleWaveReg3(uint8_t data);
    void writeTriangleWaveReg4(uint8_t data);

    void writeNoiseReg1(uint8_t data);
    void writeNoiseReg2(uint8_t data); // Unused
    void writeNoiseReg3(uint8_t data);
    void writeNoiseReg4(uint8_t data);


    void writeStatusReg(uint8_t data);
    void writeFrameCounterReg(uint8_t data);

    uint8_t readStatusReg();

    sample_t getOutput();

private:
    void setFrameInterruptFlag(bool set);

private:
    Bus* _nes;

    bool _oddCycle = true;

    // Pulse wave 1
    pulse_wave_reg1_st _pulseWave1Reg1;
    pulse_wave_reg2_st _pulseWave1Reg2;
    pulse_wave_reg3_st _pulseWave1Reg3;
    pulse_wave_reg4_st _pulseWave1Reg4;
    // Pulse wave 2
    pulse_wave_reg1_st _pulseWave2Reg1;
    pulse_wave_reg2_st _pulseWave2Reg2;
    pulse_wave_reg3_st _pulseWave2Reg3;
    pulse_wave_reg4_st _pulseWave2Reg4;
    // Triangle wave
    triangle_wave_reg1_st _triangleWaveReg1;
    triangle_wave_reg3_st _triangleWaveReg3;
    triangle_wave_reg4_st _triangleWaveReg4;
    // Noise wave
    noise_reg1_st _noiseReg1;
    noise_reg3_st _noiseReg3;
    noise_reg4_st _noiseReg4;

    status_wr_reg_st _statusWrReg;
    status_rd_reg_st _statusRdReg;
    frame_counter_st _frameCounterReg;

    PulseWaveEngine _pulseWaveEngines[2];
    TriangleWaveEngine _triangleWaveEngine;
    NoiseWaveEngine _noiseWaveEngine;

    FrameCounter _frameCounterEngine;

public:
    bool _frameInterruptFlag;

};