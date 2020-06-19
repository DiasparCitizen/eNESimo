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
#include "NESConstants.h"

#include "mostech6502.h"

#include <array>
#include "PPU.h"
#include "Cartridge.h"
#include "NESController.h"
#include "APU.h"
#include "NesCoreApi.h"

enum class dma_state {
    IDLE,
    TRANSFER_SCHEDULED,
    DUMMY_READ,
    TRANSFERRING
};

struct dma_control_st {
    dma_state dmaState;
    uint16_t dmaSrcAddr;
    uint16_t dmaDstAddr;
    uint8_t data;
};

struct sample_st {
    sample_t sample;
    double time;
};

// The BUS really represents the NES as a whole
class Bus {

public:
    Bus();
    ~Bus();

public: // System interface
    void insertCartridge(const std::shared_ptr<Cartridge>& cartridge);
    void resetNES();
    void clockNES();

public:
    void cpuWrite(uint16_t addr, uint8_t byte);
    uint8_t cpuRead(uint16_t addr, bool bReadOnly);

public: // Debug
    void printRamRange(uint16_t startAddr, uint16_t endAddr);
    void printPrgMemRange(uint16_t startAddr, uint16_t endAddr);
    void printBufferRange(uint16_t startAddr, uint16_t endAddr, uint8_t* buffer);
    friend std::string getNESStateAsStr(Bus* bus);

    uint8_t* getFrameBuffer();
    pixel_st* getPtrToLastPixelDrawn();
    bool areNewSamplesAvailable();
    sample_st* getPtrToNewSamples(uint16_t& numSamples);

public:
    // Devices connected to the Bus
    mostech6502 _cpu; // The MOS Technology 6502 CPU
    PPU _ppu; // The 2C02 Picture Processing Unit
    APU _apu;

    std::array<uint8_t, CPU_ADDR_SPACE_RAM_SIZE> _cpuRam;

    std::shared_ptr<Cartridge> _cartridge; // The inserted cartridge (or not)

    dma_control_st _dmaControl;

    double _accumulatedTime;
    double _globalTime;
    sample_st _smallSampleBuffer[10];
    uint16_t _nextSampleIdx;

    // Controllers
    // https://tresi.github.io/nes/
    NESController _controllers[2];

    // Other
    uint64_t _systemControlCounter;

    // DEBUG
    std::array<uint8_t, (128 * 1024)> cpuDebugPrgMem;

#ifdef BUS_FILE_LOG
    // Log file
    std::ofstream busLogFile;
#endif
};