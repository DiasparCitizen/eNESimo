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

	MIRRORING_TYPE getMirroringType() {
		return MIRRORING_TYPE::STATIC;
	}

	void selectBank(uint8_t bankId) {
		_selectedChrBankId = bankId;
	}

	void reset() {
		_selectedChrBankId = 0;
	}

	// The CPU tries to access the cartridge

	bool cpuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mapped_addr = addr & _prgBankMask;
			return true;
		}
		return false;
	}

	bool cpuMapWrite(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			return true;
		}
		return false;
	}

	// The PPU tries to access the cartridge

	bool ppuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {
			mapped_addr = (_selectedChrBankId * SWITCHABLE_8K_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_8K_CHR_BANK_MASK);
			return true;
		}
		return false;
	}

	bool ppuMapWrite(uint16_t addr, uint32_t& mapped_addr) {
		return false;
	}

	void serialWrite(uint16_t addr, uint8_t data) {}

private:
	uint8_t _selectedChrBankId;
	// Precalculated in constructor
	uint16_t _prgBankMask;
};