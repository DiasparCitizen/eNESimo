/*
Copyright (C) 2020 Ismael Garc�a-Marlowe

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

#include "IMapper.h"

#define SWITCHABLE_8K_CHR_BANK_BYTE_SZ (8 * 1024)
#define SWITCHABLE_8K_CHR_BANK_MASK (SWITCHABLE_8K_CHR_BANK_BYTE_SZ - 1)

class Mapper003 : public IMapper {

public:
	Mapper003(uint8_t prg_bank_count, uint8_t char_bank_count) : IMapper(prg_bank_count, char_bank_count) {
		_selectedChrBankId = 0;
		_prgBankMask = prg_bank_count > 1 ?
			MAPPER_000_2_PRG_BANKS_ADDR_MASK : MAPPER_000_1_PRG_BANKS_ADDR_MASK;
	}

	~Mapper003() {}

public:

	void reset() {
		_selectedChrBankId = 0;
	}

	MIRRORING_TYPE getMirroringType() {
		return MIRRORING_TYPE::STATIC;
	}

	MEM_MODULE mapCpuRead(uint16_t addr, uint32_t& mappedAddr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mappedAddr = addr & _prgBankMask;
			return MEM_MODULE::PRG_ROM;
		}
		return MEM_MODULE::INVALID;
	}

	MEM_MODULE mapCpuWrite(uint16_t addr, uint32_t& mappedAddr, uint8_t data) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			_selectedChrBankId = data & 0x3;
			return MEM_MODULE::OP_COMPLETE;
		}
		return MEM_MODULE::INVALID;
	}

	MEM_MODULE mapPpuRead(uint16_t addr, uint32_t& mappedAddr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {
			mappedAddr = (_selectedChrBankId * SWITCHABLE_8K_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_8K_CHR_BANK_MASK);
			return MEM_MODULE::CHR_ROM;
		}
		return MEM_MODULE::INVALID;
	}

	MEM_MODULE mapPpuWrite(uint16_t addr, uint32_t& mappedAddr) {
		return MEM_MODULE::INVALID;
	}

private:
	uint8_t _selectedChrBankId;
	// Precalculated in constructor
	uint16_t _prgBankMask;
};