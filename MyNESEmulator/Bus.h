#pragma once

#include "CommonLibs.h"
#include "NESConstants.h"

#include "mostech6502.h"

#include <array>
#include "PPU.h"
#include "Cartridge.h"
#include "NESController.h"
#include "APU.h"
#include "NesCoreApi.h"

enum DMA_STATE {
	DMA_STATE_IDLE,
	DMA_STATE_TRANSFER_SCHEDULED,
	DMA_STATE_DUMMY_READ,
	DMA_STATE_TRANSFERRING
};

struct dma_control_st {
	DMA_STATE dmaState;
	uint16_t dmaSrcAddr;
	uint16_t dmaDstAddr;
	uint8_t data;
};

// The BUS really represents the NES as a whole
class Bus {

public:
	Bus();
	~Bus();

	void setPixelMode(pixel_st* pixelOutput);
	
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
	void printBufferRange(uint16_t startAddr, uint16_t endAddr, uint8_t* buffer);
	friend std::string getNESStateAsStr(Bus* bus);

public:
	// Devices connected to the Bus
	mostech6502 _cpu; // The MOS Technology 6502 CPU
	PPU _ppu; // The 2C02 Picture Processing Unit
	APU _apu;

	pixel_st* pixelOutput;

	std::array<uint8_t, CPU_ADDR_SPACE_RAM_SIZE> _cpuRam;

	std::shared_ptr<Cartridge> _cartridge; // The inserted cartridge (or not)

	dma_control_st _dmaControl;

	// Controllers
	// https://tresi.github.io/nes/
	NESController _controllers[2];

	// Other
	uint64_t _systemControlCounter;

	// DEBUG
	std::array<uint8_t, (128 * 1024)> cpuDebugPrgMem;

#ifdef BUS_FILE_LOG
	// Log file
	std::ofstream busLogFile;
#endif
};