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

constexpr uint16_t CONTROL_REG_SELECT_BITS = 0b00;
constexpr uint16_t CHR_BANK_0_SELECT_BITS = 0b01;
constexpr uint16_t CHR_BANK_1_SELECT_BITS = 0b10;
constexpr uint16_t PRG_BANK_SELECT_BITS = 0b11;

constexpr uint16_t SWITCHABLE_16K_PRG_BANK_0_START_ADDR = 0x8000;
constexpr uint16_t SWITCHABLE_16K_PRG_BANK_1_START_ADDR =
	(SWITCHABLE_16K_PRG_BANK_0_START_ADDR + SWITCHABLE_16K_PRG_BANK_BYTE_SZ);

class Mapper001 : public IMapper {

public:
	Mapper001(uint8_t prg_bank_count, uint8_t char_bank_count)
		: IMapper(prg_bank_count, char_bank_count),
		_controlRegister {0},
		_prgBankRegister {0},
		_chrBank0Register {0},
		_loadRegister(0x00),
		_serialWriteCnt(0)
	{
		// Only fill values != 0
		_controlRegister.chrRomBankMode = 0x1;
		_controlRegister.prgRomBankMode = 0x3;
	}

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

	MEM_MODULE mapCpuRead(uint16_t addr, uint32_t& mappedAddr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {

			if (_controlRegister.prgRomBankMode < 2) { // 32 KiB blocks

				uint8_t blockId = _prgBankRegister.programBank >> 1; // Ignore lsb

				// Switch 32 KiB
				mappedAddr = (blockId * SWITCHABLE_32K_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_32K_PRG_BANK_MASK);

			}
			else if (_controlRegister.prgRomBankMode == 2) {

				if (addr >= SWITCHABLE_16K_PRG_BANK_1_START_ADDR /*0xC000*/) {
					// Switchable bank
					mappedAddr =
						(_prgBankRegister.programBank * SWITCHABLE_16K_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_16K_PRG_BANK_MASK);
				}
				else {
					// Fixed first bank
					mappedAddr = /* offset = 0 */ addr & SWITCHABLE_16K_PRG_BANK_MASK;
				}

			}
			else if (_controlRegister.prgRomBankMode == 3) {

				if (addr >= SWITCHABLE_16K_PRG_BANK_1_START_ADDR /*0xC000*/) {
					// Last bank fixed
					mappedAddr =
						((_prgBankCount - 1) * SWITCHABLE_16K_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_16K_PRG_BANK_MASK);
				}
				else {
					// Switchable bank
					mappedAddr =
						(_prgBankRegister.programBank * SWITCHABLE_16K_PRG_BANK_BYTE_SZ) + (addr & SWITCHABLE_16K_PRG_BANK_MASK);
				}

			}
			return MEM_MODULE::PRG_ROM;

		}
		else if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_RAM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_RAM_END) {
			mappedAddr = addr & 0x1FFF;
			return MEM_MODULE::PRG_RAM;
		}
		return MEM_MODULE::INVALID;
	}

	MEM_MODULE mapCpuWrite(uint16_t addr, uint32_t& mappedAddr, uint8_t data) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			serialWrite(addr, data);
			return MEM_MODULE::PRG_ROM;
		}
		else if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_RAM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_RAM_END) {
			mappedAddr = addr & 0x1FFF;
			return MEM_MODULE::PRG_RAM;
		}
		return MEM_MODULE::INVALID;
	}

	MEM_MODULE mapPpuRead(uint16_t addr, uint32_t& mappedAddr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {

			if (_charBankCount == 0) {
				mappedAddr = addr;
			}
			else if (addr <= PPU_ADDR_SPACE_PATTERN_TABLE_0_END) {

				if (_controlRegister.chrRomBankMode == 0) { // 8 KiB
					uint8_t blockId = _chrBank0Register >> 1;
					mappedAddr = (blockId * SWITCHABLE_8K_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_8K_CHR_BANK_MASK);
				}
				else { // Switch 4 KiB
					mappedAddr = (_chrBank0Register * SWITCHABLE_4K_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_4K_CHR_BANK_MASK);
				}

			}
			else { // >= PPU_ADDR_SPACE_PATTERN_TABLE_1_START

				if (_controlRegister.chrRomBankMode == 0) { // 8 KiB
					//return MEM_MODULE::CHR_ROM;
				}
				else {
					mappedAddr = (_chrBank1Register * SWITCHABLE_4K_CHR_BANK_BYTE_SZ) + (addr & SWITCHABLE_4K_CHR_BANK_MASK);
				}

			}

			return MEM_MODULE::CHR_ROM;

		}
		return MEM_MODULE::INVALID;
	}

	MEM_MODULE mapPpuWrite(uint16_t addr, uint32_t& mappedAddr) {
		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {
			mappedAddr = addr;
			return MEM_MODULE::CHR_ROM;
		}
		return MEM_MODULE::INVALID;
	}

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

private:
	uint8_t _loadRegister;
	control_register_st _controlRegister;
	uint8_t _chrBank0Register;
	uint8_t _chrBank1Register;
	prg_bank_register_st _prgBankRegister;

	uint8_t _serialWriteCnt;

};