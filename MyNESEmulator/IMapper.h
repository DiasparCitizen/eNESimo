#pragma once
#include <cstdint>
#include "NESConstants.h"

class IMapper { // Interface to cartridge memory

public:
	IMapper(uint8_t prgBankCount, uint8_t charBankCount) {
		this->prgBankMask = 0x0;
		this->prgBankCount = 0x0;
		this->charBankCount = 0x0;
	}
	~IMapper() {}

public:
	virtual bool cpuMapRead(uint16_t addr, uint32_t& mappedAddr) = 0;
	virtual bool cpuMapWrite(uint16_t addr, uint32_t& mappedAddr) = 0;
	virtual bool ppuMapRead(uint16_t addr, uint32_t& mappedAddr) = 0;
	virtual bool ppuMapWrite(uint16_t addr, uint32_t& mappedAddr) = 0;

protected:
	uint8_t prgBankCount;
	uint8_t charBankCount;
	// Precalculated in constructor
	uint16_t prgBankMask;

};