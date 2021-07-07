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

enum class MIRRORING_TYPE {
    H,
    V,
    ONE_LO,
    ONE_HI,
    STATIC, // Doesn't change, take from ines header
    UNSET
};

enum class MEM_MODULE {
    CHR_ROM,
    PRG_ROM,
    PRG_RAM,
    INVALID,
    OP_COMPLETE
};

constexpr uint16_t SWITCHABLE_16K_PRG_BANK_BYTE_SZ = (16 * 1024);
constexpr uint16_t SWITCHABLE_16K_PRG_BANK_MASK = (SWITCHABLE_16K_PRG_BANK_BYTE_SZ - 1);

constexpr uint16_t SWITCHABLE_32K_PRG_BANK_BYTE_SZ = (SWITCHABLE_16K_PRG_BANK_BYTE_SZ * 2);
constexpr uint16_t SWITCHABLE_32K_PRG_BANK_MASK = (SWITCHABLE_32K_PRG_BANK_BYTE_SZ - 1);

constexpr uint16_t SWITCHABLE_4K_CHR_BANK_BYTE_SZ = (4 * 1024);
constexpr uint16_t SWITCHABLE_4K_CHR_BANK_MASK = (SWITCHABLE_4K_CHR_BANK_BYTE_SZ - 1);

constexpr uint16_t SWITCHABLE_8K_CHR_BANK_BYTE_SZ = (SWITCHABLE_4K_CHR_BANK_BYTE_SZ * 2);
constexpr uint16_t SWITCHABLE_8K_CHR_BANK_MASK = (SWITCHABLE_8K_CHR_BANK_BYTE_SZ - 1);

class IMapper {

public:
    IMapper(uint8_t prg_bank_count, uint8_t char_bank_count) {
        this->_prgBankCount = prg_bank_count;
        this->_charBankCount = char_bank_count;
    }
    ~IMapper() {}

    virtual void reset() = 0;

    virtual MEM_MODULE mapCpuRead(uint16_t addr, uint32_t& mappedAddr) = 0;
    virtual MEM_MODULE mapCpuWrite(uint16_t addr, uint32_t& mappedAddr, uint8_t data) = 0;
    virtual MEM_MODULE mapPpuRead(uint16_t addr, uint32_t& mappedAddr) = 0;
    virtual MEM_MODULE mapPpuWrite(uint16_t addr, uint32_t& mappedAddr) = 0;

    virtual MIRRORING_TYPE getMirroringType() = 0;

protected:
    uint8_t _prgBankCount;
    uint8_t _charBankCount;

};