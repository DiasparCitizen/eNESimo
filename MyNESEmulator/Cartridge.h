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

#include <vector>
#include <memory>

#include "IMapper.h"

struct ines_header_st {

    char name[4];
    uint8_t prgRom16KiBChunkCount;
    uint8_t chrRom8KiBChunkCount; //  If 0, the game has character RAM instead of ROM

    // Flags 6
    uint8_t mirroring : 1; // 0: horizontal, 1: vertical
    uint8_t bbPrgRam : 1; // Contains battery-backed PRG RAM
    uint8_t has512ByteTrainer : 1; // 512-byte trainer at $7000-$71FF (stored before PRG data)
    uint8_t ignoreMirroringCtrl : 1; // Ignore mirroring control or above mirroring bit; instead provide four-screen VRAM
    uint8_t mapperNumberLowerNibble : 4; // Lower nybble of mapper number

    // Flags 7
    uint8_t vsUnisystem : 1; // VS Unisystem
    uint8_t playChoice : 1; // PlayChoice-10 (8KB of Hint Screen data stored after CHR data)
    uint8_t new2p0Format : 2; // If equal to 2, flags 8-15 are in NES 2.0 format
    uint8_t mapperNumberUpperNibble : 4; // Upper nybble of mapper number

    // Flags 8
    uint8_t prgRam8KiBChunkCount; // Rarely used extension

    // Flags 9
    uint8_t tvSystem1 : 1; // TV system (0: NTSC; 1: PAL)
    uint8_t reserved0 : 7; // Reserved, set to 0

    // Flags 10
    uint8_t tvSystem2 : 2; // TV system (0: NTSC; 2: PAL; 1/3: dual compatible)
    uint8_t reserved1 : 2; // Unused
    uint8_t prgRamPresent : 1; // PRG RAM ($6000-$7FFF) (0: present; 1: not present)
    uint8_t doesBoardHaveBusConflicts : 1; // 0: Board has no bus conflicts; 1: Board has bus conflicts
    uint8_t reserved2 : 2;

    // Rest
    uint8_t unusedPadding[5];

};

class Cartridge {

public:
    Cartridge(const std::string& cartridgeFileName);
    ~Cartridge();

public:
    // Comms with CPU bus
    bool cpuRead(uint16_t addr, uint8_t& data);
    bool cpuWrite(uint16_t addr, uint8_t byte);

    // Comms with PPU bus
    bool ppuRead(uint16_t addr, uint8_t& data);
    bool ppuWrite(uint16_t addr, uint8_t byte);

public:

    // Helper vars, constants extracted from cartridge header
    uint8_t mapperId;
    uint8_t prgBankCount; // Can be more than addressable by the CPU
    uint8_t charBankCount; // Can be more than addressable by the PPU

    // Mapper
    std::shared_ptr<IMapper> _mapper;

    // Memory
    std::vector<uint8_t> _programRom;
    std::vector<uint8_t> _characterRom;

    // Cached
    bool _vertical;
    ines_header_st _cartridgeHeader;

};