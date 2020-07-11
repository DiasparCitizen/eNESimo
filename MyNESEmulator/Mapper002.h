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

#define MAPPER002_SWITCHABLE_BANK_START_ADDR 0x8000
#define MAPPER002_SWITCHABLE_BANK_END_ADDR 0xBFFF

#define MAPPER002_FIXED_BANK_START_ADDR 0xC000
#define MAPPER002_FIXED_BANK_END_ADDR 0xFFFF

#define _IS_SWITCHABLE_BANK_ADDR(addr) (addr >= MAPPER002_SWITCHABLE_BANK_START_ADDR && addr <= MAPPER002_SWITCHABLE_BANK_END_ADDR)
#define _IS_FIXED_BANK_ADDR(addr) (addr >= MAPPER002_FIXED_BANK_START_ADDR && addr <= MAPPER002_FIXED_BANK_END_ADDR)

#define SWITCHABLE_BANK_BYTE_SZ (1024 * 16) // 16 KiB
#define SWITCHABLE_BANK_MASK (SWITCHABLE_BANK_BYTE_SZ - 1)

/*
From NESDEV: https://wiki.nesdev.com/w/index.php/UxROM
iNES Mapper 002 is the implementation of the most common usage of UxROM compatible boards, described in this article.
Example games:
Mega Man
Castlevania
Contra
Duck Tales
Metal Gear
*/
class Mapper002 : public IMapper {

public:

	Mapper002(uint8_t prg_bank_count, uint8_t char_bank_count) : IMapper(prg_bank_count, char_bank_count) {
		_selectedBankId = 0;
	}

	~Mapper002() {}

public:

	MIRRORING_TYPE getMirroringType() {
		return MIRRORING_TYPE::V; // Does not apply
	}

	void selectBank(uint8_t bankId) {
		_selectedBankId = bankId;
	}

	void reset() {
		_selectedBankId = 0;
	}

	// The CPU tries to access the cartridge

	bool cpuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		bool mapSuccess = false;
		if (_IS_FIXED_BANK_ADDR(addr)) {
			//mapped_addr = addr;
			mapped_addr = ((_prgBankCount - 1) * SWITCHABLE_BANK_BYTE_SZ) + (addr & SWITCHABLE_BANK_MASK);
			mapSuccess = true;
		}
		else if (_IS_SWITCHABLE_BANK_ADDR(addr)) {
			mapped_addr = (_selectedBankId * SWITCHABLE_BANK_BYTE_SZ) + (addr & SWITCHABLE_BANK_MASK);
			mapSuccess = true;
		}
		return mapSuccess;
	}

	bool cpuMapWrite(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mapped_addr = addr; // No use, though
			return true;
		}
		return false;
	}

	// The PPU tries to access the cartridge

	bool ppuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {
			mapped_addr = addr;
			return true;
		}
		return false;
	}

	bool ppuMapWrite(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {
			mapped_addr = addr;
			return true;
		}
		return false;
	}

private:
	uint8_t _selectedBankId;

};