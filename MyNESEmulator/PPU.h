#pragma once

#include <cstdint>
#include <iostream>

#include "Cartridge.h"
//#include "Bus.h"

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include "olcPixelGameEngine.h"

// Forward declare
class Bus;

struct ppuconfig_st {
	bool isNTSC;
	int32_t lastDrawableScanline;
	int32_t nmiScanline;
	int32_t totalScanlines;
	int32_t postRenderScanline;
};

union ppuctrl_un {
	struct {
		uint8_t baseNametableAddr : 2; // (0 = $2000; 1 = $2400; 2 = $2800; 3 = $2C00)
		uint8_t vramAddrIncrementPerCpuRw : 1; // (0: add 1, going across; 1: add 32, going down)
		uint8_t sprPatternTableAddrFor8x8Mode : 1; // (0: $0000; 1: $1000; ignored in 8x16 mode)
		uint8_t bgPatternTableAddr : 1; // (0: $0000; 1: $1000)
		uint8_t sprSize : 1; // (0: 8x8 pixels; 1: 8x16 pixels)
		uint8_t ppuMasterSlaveSelect : 1; // (0: read backdrop from EXT pins; 1: output color on EXT pins)
		uint8_t nmiAtVBlankIntervalStart : 1; // (0: off; 1: on)
	};
	uint8_t raw;
};

union ppumask_un {
	struct {
		uint8_t grayscale : 1; // (0: normal color, 1: produce a greyscale display)
		uint8_t showBgLeft : 1; // 0: hide
		uint8_t showSprLeft : 1; // 0: hide
		uint8_t showBg : 1;
		uint8_t showSpr : 1;
		uint8_t emphasizeRed : 1;
		uint8_t emphasizeGreen : 1;
		uint8_t emphasizeBlue : 1;
	};
	uint8_t raw;
};

union ppustatus_un {
	struct {
		uint8_t unused : 5;
		uint8_t sprOverflow : 1;
		uint8_t sprZeroHit : 1;
		uint8_t verticalBlank : 1;
	};
	uint8_t raw;
};

union loopy_un {
	struct {
		uint16_t coarseX : 5;
		uint16_t coarseY : 5;
		uint16_t nametableX : 1;
		uint16_t nametableY : 1;
		uint16_t fineY : 3;
		uint16_t unused : 1;
	};
	uint16_t raw = 0x0000;
};

struct debug_ppu_state_dsc_st {

	int16_t scanline;
	int16_t scanlineDot;
	uint64_t frameCounter;

	ppuctrl_un controlReg;
	ppumask_un maskReg;
	ppustatus_un statusReg;

	loopy_un tmpVramAddr;
	loopy_un vramAddr;
	uint8_t fineX;

};

#define IS_PRERENDER_SCANLINE() (_scanline == -1)
#define IS_DRAWABLE_SCANLINE() (_scanline >= -1 && _scanline <= 239)

#define TRANSFER_ADDR_X() \
	_vramAddr.coarseX = _tmpVramAddr.coarseX; \
	_vramAddr.nametableX = _tmpVramAddr.nametableX

#define TRANSFER_ADDR_Y() \
	_vramAddr.fineY = _tmpVramAddr.fineY; \
	_vramAddr.coarseY = _tmpVramAddr.coarseY; \
	_vramAddr.nametableY = _tmpVramAddr.nametableY

#define MOVE_PIPES() \
	_bg16pxPaletteIdLsbPipe <<= 1; \
	_bg16pxPaletteIdMsbPipe <<= 1; \
	_bg16pxColorIdLsbPipe <<= 1; \
	_bg16pxColorIdMsbPipe <<= 1

#define PUSH_PALETTE_ID_LSBS_TO_PIPE(paletteId) \
	_bg16pxPaletteIdLsbPipe = ((_bg16pxPaletteIdLsbPipe & 0xFF00) | (paletteId & 0b01 ? 0xFF : 0x00))

#define PUSH_PALETTE_ID_MSBS_TO_PIPE(paletteId) \
	_bg16pxPaletteIdMsbPipe = ((_bg16pxPaletteIdMsbPipe & 0xFF00) | (paletteId & 0b10 ? 0xFF : 0x00))

#define PUSH_COLOR_ID_LSBS_TO_PIPE(tileLsbs) \
	_bg16pxColorIdLsbPipe = ((_bg16pxColorIdLsbPipe & 0xFF00) | tileLsbs)

#define PUSH_COLOR_ID_MSBS_TO_PIPE(tileMsbs) \
	_bg16pxColorIdMsbPipe = ((_bg16pxColorIdMsbPipe & 0xFF00) | tileMsbs)

#define INCREMENT_Y() \
	_vramAddr.fineY = (_vramAddr.fineY + 1) % 8; \
	if (_vramAddr.fineY == 0){ \
		_vramAddr.coarseY = (_vramAddr.coarseY + 1) % PPU_NAME_TABLE_DRAWABLE_ROWS_PER_NAME_TABLE; \
		if (_vramAddr.coarseY == 0){ \
			_vramAddr.nametableY = ~_vramAddr.nametableY; \
		} \
	}

#define INCREMENT_X() \
	_vramAddr.coarseX = (_vramAddr.coarseX + 1) % PPU_NAME_TABLE_COLS_PER_ROW; \
	if (_vramAddr.coarseX == 0){ \
		_vramAddr.nametableX = ~_vramAddr.nametableX; \
	}

class PPU {

public:
	PPU();
	~PPU();
	void reset();

public:
	void connectConsole(Bus* bus);
	void connectCartridge(const std::shared_ptr<Cartridge>& cartridge);
	void clock();

public:
	// Comms with CPU bus
	uint8_t cpuRead(uint16_t addr, bool readOnly = false);
	void cpuWrite(uint16_t addr, uint8_t data);
	// Comms with PPU bus
	uint8_t ppuRead(uint16_t addr, bool readOnly = false);
	void ppuWrite(uint16_t addr, uint8_t data);

	// Debug
	void printPPURamRange(uint16_t startAddr, uint16_t endAddr);
	debug_ppu_state_dsc_st getDebugPPUstate();

private: // Components accessible from the PPU

	std::shared_ptr<Cartridge> _cartridge;
	// Pattern memory goes back to the bitmap times, would hold char maps.
	// In the times of the NES, pattern tables were holding sprites.
	uint8_t _patternTables[2][PPU_PATTERN_TABLE_SIZE];
	uint8_t _nameTables[2][PPU_NAME_TABLE_SIZE];
	uint8_t _paletteMem[PPU_PALETTE_SIZE * 2]; // Size 32

public:
	uint8_t _oamMem[PPU_OAM_SIZE];

	/************** VIDEO TUTORIAL #3 *********************/
private:
	olc::Pixel  palScreen[0x40];
	olc::Sprite sprScreen = olc::Sprite(256, 240);
	olc::Sprite sprNameTable[2] = { olc::Sprite(256, 240), olc::Sprite(256, 240) };
	olc::Sprite sprPatternTable[2] = { olc::Sprite(128, 128), olc::Sprite(128, 128) };

public:

	// Debugging Utilities
	olc::Sprite& GetScreen();
	olc::Sprite& GetNameTable(uint8_t i);
	olc::Sprite& GetPatternTable(uint8_t i, uint8_t palette);
	olc::Pixel& GetColourFromPaletteRam(uint8_t palette, uint8_t pixel);
	bool _frameComplete = false;

public:

	// Render
	int32_t _scanline;
	int16_t _scanlineDot;

	bool _oddFrameSwitch;
	uint8_t _addrScrollLatch;
	uint8_t _dataBuffer;

	uint8_t _bgTileIdNxt;
	uint8_t _bgTileAttrByteNxt;

	uint8_t _bgNxt8pxPaletteId;
	uint8_t _bgNxt8pxColorIdLsb;
	uint8_t _bgNxt8pxColorIdMsb;

	uint16_t _bg16pxPaletteIdLsbPipe;
	uint16_t _bg16pxPaletteIdMsbPipe;
	uint16_t _bg16pxColorIdLsbPipe;
	uint16_t _bg16pxColorIdMsbPipe;
	
	ppuctrl_un _controlReg;
	ppumask_un _maskReg;
	ppustatus_un _statusReg;
	loopy_un _tmpVramAddr;
	loopy_un _vramAddr;
	uint8_t _fineX;
	uint8_t _oamAddr;
	uint8_t _oamData;

	ppuconfig_st _ppuConfig;

	Bus* _nes;

	bool _8pxBatchReady;

	bool isNTSC;

	// DEBUG
	debug_ppu_state_dsc_st _debugPPUState;
	uint64_t _frameCounter;

#if defined(PPU_FILE_LOG) || defined(PPU_FILE_LOG2)
	// Log file
public:
	std::ofstream ppuLogFile;
#endif

};