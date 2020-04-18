#pragma once

#include "mostech6502.h"
#include <cstdint>
#include "NESConstants.h"
#include <array>
#include "PPU.h"
#include "Cartridge.h"

// The BUS really represents the NES as a whole
class Bus {

public:
	Bus();
	~Bus();
	
public: // System interface
	void insertCartridge(const std::shared_ptr<Cartridge>& cartridge);
	void resetNES();
	void clockNES();

public:
	void cpuWrite(uint16_t addr, uint8_t byte);
	uint8_t cpuRead(uint16_t addr, bool bReadOnly);

public: // Debug
	void printRamRange(uint16_t startAddr, uint16_t endAddr);
	void printPrgMemRange(uint16_t startAddr, uint16_t endAddr);
	friend std::string getNESStateAsStr(Bus* bus);

public:
	// Devices connected to the Bus
	mostech6502 _cpu; // The MOS Technology 6502 CPU
	PPU _ppu; // The 2C02 Picture Processing Unit

	std::array<uint8_t, CPU_ADDR_SPACE_RAM_SIZE> _cpuRam;

	std::shared_ptr<Cartridge> _cartridge; // The inserted cartridge (or not)

	// Controllers
	uint8_t controller[2];

	// Other
	uint64_t systemClockCounter;
	uint32_t turnCounter;

	// DEBUG
	std::array<uint8_t, (128 * 1024)> cpuDebugPrgMem;

#ifdef BUS_FILE_LOG
	// Log file
	std::ofstream busLogFile;
#endif
};