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