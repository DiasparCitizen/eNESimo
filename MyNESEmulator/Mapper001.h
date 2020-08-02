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

// https://wiki.nesdev.com/w/index.php?title=MMC1&redirect=no
// Games
// http://www.godzil.net/nesstat/list.php

struct control_register_st {
	uint8_t mirroring : 2;      // (0: one-screen, lower bank; 1: one-screen, upper bank; 2: vertical; 3: horizontal)
	uint8_t prgRomBankMode : 2; // (0, 1: switch 32 KB at $8000, ignoring low bit of bank number;
                                // 2: fix first bank at $8000 and switch 16 KB bank at $C000;
                                // 3: fix last bank at $C000 and switch 16 KB bank at $8000)
	uint8_t chrRomBankMode : 1; // (0: switch 8 KB at a time; 1: switch two separate 4 KB banks)
};

struct prg_bank_register_st {
	uint8_t programBank : 4; // Select 16 KB PRG ROM bank (low bit ignored in 32 KB mode)
	uint8_t programBankChipEnable : 1; // PRG RAM chip enable (0: enabled; 1: disabled; ignored on MMC1A)
};


// Variants
struct chr_bank0_register_st {
	uint8_t C : 1; // Select 4 KB CHR RAM bank at PPU $0000 (ignored in 8 KB mode)
	uint8_t reserved : 3;
	uint8_t E : 1; // PRG RAM disable (0: enable, 1: open bus)
};

struct chr_bank1_register_st {
	uint8_t C : 1; // Select 4 KB CHR RAM bank at PPU $1000 (ignored in 8 KB mode)
	uint8_t reserved : 3;
	uint8_t E : 1; // PRG RAM disable (0: enable, 1: open bus)
};

#define CONTROL_REG_SELECT_BITS 0b00
#define CHR_BANK_0_SELECT_BITS 0b01
#define CHR_BANK_1_SELECT_BITS 0b10
#define PRG_BANK_SELECT_BITS 0b11

#define SWITCHABLE_PRG_BANK_BYTE_SZ (16 * 1024)
#define SWITCHABLE_PRG_BANK_MASK (SWITCHABLE_PRG_BANK_BYTE_SZ - 1)
#define SWITCHABLE_PRG_BANK_0_START_ADDR 0x8000
#define SWITCHABLE_PRG_BANK_1_START_ADDR (SWITCHABLE_PRG_BANK_0_START_ADDR + SWITCHABLE_PRG_BANK_BYTE_SZ)

#define DOUBLE_PRG_BANK_BYTE_SZ (SWITCHABLE_PRG_BANK_BYTE_SZ * 2)
#define DOUBLE_PRG_BANK_MASK (DOUBLE_PRG_BANK_BYTE_SZ - 1)

#define SWITCHABLE_CHR_BANK_BYTE_SZ (4 * 1024)
#define SWITCHABLE_CHR_BANK_MASK (SWITCHABLE_CHR_BANK_BYTE_SZ - 1)

#define DOUBLE_CHR_BANK_BYTE_SZ (SWITCHABLE_CHR_BANK_BYTE_SZ * 2)
#define DOUBLE_CHR_BANK_MASK (DOUBLE_CHR_BANK_BYTE_SZ - 1)

class Mapper001 : public IMapper {

public:
	Mapper001(uint8_t prg_bank_count, uint8_t char_bank_count) : IMapper(prg_bank_count, char_bank_count) {

		_controlRegister.mirroring = 0;
		_controlRegister.chrRomBankMode = 0x1;
		_controlRegister.prgRomBankMode = 0x3;
		_prgBankRegister.programBank = 0;
		_prgBankRegister.programBankChipEnable = 0;
		_chrBank0Register = 0;
		_chrBank1Register = 0;
		_loadRegister = 0x00;
		_serialWriteCnt = 0;

	}

	~Mapper001() {}

public:

	void reset() {}

	MIRRORING_TYPE getMirroringType() {
		MIRRORING_TYPE mType;
		switch (_controlRegister.mirroring) {
		case 0: mType = MIRRORING_TYPE::ONE_LO; break;
		case 1: mType = MIRRORING_TYPE::ONE_HI; break;
		case 2: mType = MIRRORING_TYPE::V; break;
		default: mType = MIRRORING_TYPE::H; break;
		}
		//std::cout << "mType : " << (int)mType << std::endl;
		return mType;
	}

	void selectBank(uint8_t bankId) {}

	void serialWrite(uint16_t addr, uint8_t data) {

		if (data & 0x80) {
			// Clear
			_serialWriteCnt = 0;
			_loadRegister = 0x00;
		}
		else {

			uint8_t bit = data & 0x1;
			_loadRegister |= (bit << _serialWriteCnt);

			// Increment counter
			_serialWriteCnt = (_serialWriteCnt + 1) % 5;

			// If we've already written 5 bits, load value onto
			// the selected register
			if (_serialWriteCnt == 0) {

				uint16_t selectorBits = (addr >> 13) & 0x3;
				switch (selectorBits) {
				case CONTROL_REG_SELECT_BITS: *((uint8_t*)&_controlRegister) = _loadRegister & 0x1F; break;
				case CHR_BANK_0_SELECT_BITS: _chrBank0Register = _loadRegister & 0x1F; break;
				case CHR_BANK_1_SELECT_BITS: _chrBank1Register = _loadRegister & 0x1F; break;
				case PRG_BANK_SELECT_BITS: *((uint8_t*)&_prgBankRegister) = _loadRegister & 0x1F; break;
				}

				// Clear load register
				_loadRegister = 0x00;

			}

		}

	}

	bool cpuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {

			if (_controlRegister.prgRomBankMode < 2) { // 32 KiB blocks

				uint8_t blockId = _prgBankRegister.programBank >> 1; // Ignore lsb

				// Switch 32 KiB
				mapped_addr = (blockId * DOUBLE_PRG_BANK_BYTE_SZ) + (addr & DOUBLE_PRG_BANK_MASK);

				return true;

			}
			else if (_controlRegister.prgRomBankMode == 2) {

				if (addr >= SWITCHABLE_PRG_BANK_1_START_ADDR /*0xC000*/) {
					// Switchable bank
					mapped_addr =
						(_prgBankRegister.programBank * SWITCHABLE_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_PRG_BANK_MASK);
				}
				else {
					// Fixed first bank
					mapped_addr = /* offset = 0 */ addr & SWITCHABLE_PRG_BANK_MASK;
				}
				return true;

			}
			else if (_controlRegister.prgRomBankMode == 3) {

				if (addr >= SWITCHABLE_PRG_BANK_1_START_ADDR /*0xC000*/) {
					// Last bank fixed
					mapped_addr =
						((_prgBankCount - 1) * SWITCHABLE_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_PRG_BANK_MASK);
				}
				else {
					// Switchable bank
					mapped_addr =
						(_prgBankRegister.programBank * SWITCHABLE_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_PRG_BANK_MASK);
				}
				return true;

			}

		}
		return false;
	}

	bool cpuMapWrite(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			// This will result in a write to the LOAD register
			return true;
		}
		return false;
	}

	// The PPU tries to access the cartridge

	bool ppuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {

			if (_charBankCount == 0) {
				mapped_addr = addr;
			}
			else if (addr <= PPU_ADDR_SPACE_PATTERN_TABLE_0_END) {

				if (_controlRegister.chrRomBankMode == 0) { // 8 KiB
					uint8_t blockId = _chrBank0Register >> 1;
					mapped_addr = (blockId * DOUBLE_CHR_BANK_BYTE_SZ) + (addr & DOUBLE_CHR_BANK_MASK);
				}
				else { // Switch 4 KiB
					mapped_addr = (_chrBank0Register * SWITCHABLE_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_CHR_BANK_MASK);
				}

			}
			else { // >= PPU_ADDR_SPACE_PATTERN_TABLE_1_START

				if (_controlRegister.chrRomBankMode == 0) { // 8 KiB
					return false;
				}
				else {
					mapped_addr = (_chrBank1Register * SWITCHABLE_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_CHR_BANK_MASK);
				}

			}

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
	uint8_t _loadRegister;
	control_register_st _controlRegister;
	uint8_t _chrBank0Register;
	uint8_t _chrBank1Register;
	prg_bank_register_st _prgBankRegister;

	uint8_t _serialWriteCnt;

};