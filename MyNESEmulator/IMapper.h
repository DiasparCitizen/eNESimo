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
#include "NESConstants.h"

class IMapper { // Interface to cartridge memory

public:
    IMapper(uint8_t prg_bank_count, uint8_t char_bank_count) {
        this->_prgBankMask = 0x0;
        this->_prgBankCount = 0x0;
        this->_charBankCount = 0x0;
    }
    ~IMapper() {}

public:
    virtual bool cpuMapRead(uint16_t addr, uint32_t& mappedAddr) = 0;
    virtual bool cpuMapWrite(uint16_t addr, uint32_t& mappedAddr) = 0;
    virtual bool ppuMapRead(uint16_t addr, uint32_t& mappedAddr) = 0;
    virtual bool ppuMapWrite(uint16_t addr, uint32_t& mappedAddr) = 0;

protected:
    uint8_t _prgBankCount;
    uint8_t _charBankCount;
    // Precalculated in constructor
    uint16_t _prgBankMask;

};