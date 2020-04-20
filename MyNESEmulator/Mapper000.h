#pragma once

#include "IMapper.h"

class Mapper000 : public IMapper {

public:

	Mapper000(uint8_t prg_bank_count, uint8_t char_bank_count) : IMapper(prg_bank_count, char_bank_count) {
		this->_prgBankCount = prg_bank_count;
		this->_charBankCount = char_bank_count;

		this->_prgBankMask = prg_bank_count > 1 ?
			MAPPER_000_2_PRG_BANKS_ADDR_MASK : MAPPER_000_1_PRG_BANKS_ADDR_MASK;
	}

	~Mapper000() {}

public:

	// The CPU tries to access the cartridge

	bool cpuMapRead(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mapped_addr = addr & this->_prgBankMask;
			return true;
		}
		return false;
	}

	bool cpuMapWrite(uint16_t addr, uint32_t& mapped_addr) {
		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mapped_addr = addr & this->_prgBankMask;
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

};