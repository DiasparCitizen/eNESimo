#pragma once

#include "IMapper.h"

class Mapper000 : public IMapper {

public:

	Mapper000(uint8_t prgBankCount, uint8_t charBankCount) : IMapper(prgBankCount, charBankCount) {
		this->prgBankCount = prgBankCount;
		this->charBankCount = charBankCount;

		this->prgBankMask = prgBankCount > 1 ?
			MAPPER_000_2_PRG_BANKS_ADDR_MASK : MAPPER_000_1_PRG_BANKS_ADDR_MASK;
	}

	~Mapper000() {}

public:

	// The CPU tries to access the cartridge

	bool cpuMapRead(uint16_t addr, uint32_t& mappedAddr) {

		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mappedAddr = addr & this->prgBankMask;
			return true;
		}
		return false;
	}

	bool cpuMapWrite(uint16_t addr, uint32_t& mappedAddr) {

		if (addr >= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START && addr <= CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END) {
			mappedAddr = addr & this->prgBankMask;
			return true;
		}
		return false;
	}

	// The PPU tries to access the cartridge

	bool ppuMapRead(uint16_t addr, uint32_t& mappedAddr) {

		if (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END) {
			//mappedAddr &= PPU_ADDR_SPACE_PATTERN_TABLES_MASK;
			mappedAddr = addr;
			return true;
		}
		return false;
	}

	bool ppuMapWrite(uint16_t addr, uint32_t& mappedAddr) {

		// Why would we write to the character ROM?
		return false;
	}

};