#include "NESConstants.h"
#include "PPU.h"
#include "Bus.h"

#define _IS_PATTERN_TABLE_ADDR(addr) (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END)
#define _IS_PALETTE_ADDR(addr) (addr >= PPU_ADDR_SPACE_PALETTES_REGION_START && addr <= PPU_ADDR_SPACE_PALETTES_REGION_END)

#define _IS_NAMETABLE_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_3_END)

#define _IS_NAMETABLE_0_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_0_END)
#define _IS_NAMETABLE_1_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_1_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_1_END)
#define _IS_NAMETABLE_2_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_2_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_2_END)
#define _IS_NAMETABLE_3_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_3_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_3_END)

void printBuffer(uint16_t startAddr, uint16_t endAddr, uint8_t* buffer);


#if defined(PPU_TERMINAL_LOG) && defined(PPU_FILE_LOG)
#define _LOG(txt) \
{ \
std::stringstream myStream; \
myStream << txt; \
std::cout << myStream.str(); \
ppuLogFile << myStream.str(); \
} 
#elif defined(PPU_TERMINAL_LOG)
#define _LOG(txt) std::cout << txt
#elif defined(PPU_FILE_LOG)
#define _LOG(txt) ppuLogFile << txt
#else
#define _LOG(txt) ;
#endif

#if defined(PPU_TERMINAL_LOG2) && defined(PPU_FILE_LOG2)
#define _LOG2(txt) \
{ \
std::stringstream myStream; \
myStream << txt; \
std::cout << myStream.str(); \
ppuLogFile << myStream.str(); \
} 
#elif defined(PPU_TERMINAL_LOG2)
#define _LOG2(txt) std::cout << txt
#elif defined(PPU_FILE_LOG2)
#define _LOG2(txt) ppuLogFile << txt
#else
#define _LOG2(txt) ;
#endif


PPU::PPU()
{
	// Copied
	palScreen[0x00] = olc::Pixel(84, 84, 84);
	palScreen[0x01] = olc::Pixel(0, 30, 116);
	palScreen[0x02] = olc::Pixel(8, 16, 144);
	palScreen[0x03] = olc::Pixel(48, 0, 136);
	palScreen[0x04] = olc::Pixel(68, 0, 100);
	palScreen[0x05] = olc::Pixel(92, 0, 48);
	palScreen[0x06] = olc::Pixel(84, 4, 0);
	palScreen[0x07] = olc::Pixel(60, 24, 0);
	palScreen[0x08] = olc::Pixel(32, 42, 0);
	palScreen[0x09] = olc::Pixel(8, 58, 0);
	palScreen[0x0A] = olc::Pixel(0, 64, 0);
	palScreen[0x0B] = olc::Pixel(0, 60, 0);
	palScreen[0x0C] = olc::Pixel(0, 50, 60);
	palScreen[0x0D] = olc::Pixel(0, 0, 0);
	palScreen[0x0E] = olc::Pixel(0, 0, 0);
	palScreen[0x0F] = olc::Pixel(0, 0, 0);

	palScreen[0x10] = olc::Pixel(152, 150, 152);
	palScreen[0x11] = olc::Pixel(8, 76, 196);
	palScreen[0x12] = olc::Pixel(48, 50, 236);
	palScreen[0x13] = olc::Pixel(92, 30, 228);
	palScreen[0x14] = olc::Pixel(136, 20, 176);
	palScreen[0x15] = olc::Pixel(160, 20, 100);
	palScreen[0x16] = olc::Pixel(152, 34, 32);
	palScreen[0x17] = olc::Pixel(120, 60, 0);
	palScreen[0x18] = olc::Pixel(84, 90, 0);
	palScreen[0x19] = olc::Pixel(40, 114, 0);
	palScreen[0x1A] = olc::Pixel(8, 124, 0);
	palScreen[0x1B] = olc::Pixel(0, 118, 40);
	palScreen[0x1C] = olc::Pixel(0, 102, 120);
	palScreen[0x1D] = olc::Pixel(0, 0, 0);
	palScreen[0x1E] = olc::Pixel(0, 0, 0);
	palScreen[0x1F] = olc::Pixel(0, 0, 0);

	palScreen[0x20] = olc::Pixel(236, 238, 236);
	palScreen[0x21] = olc::Pixel(76, 154, 236);
	palScreen[0x22] = olc::Pixel(120, 124, 236);
	palScreen[0x23] = olc::Pixel(176, 98, 236);
	palScreen[0x24] = olc::Pixel(228, 84, 236);
	palScreen[0x25] = olc::Pixel(236, 88, 180);
	palScreen[0x26] = olc::Pixel(236, 106, 100);
	palScreen[0x27] = olc::Pixel(212, 136, 32);
	palScreen[0x28] = olc::Pixel(160, 170, 0);
	palScreen[0x29] = olc::Pixel(116, 196, 0);
	palScreen[0x2A] = olc::Pixel(76, 208, 32);
	palScreen[0x2B] = olc::Pixel(56, 204, 108);
	palScreen[0x2C] = olc::Pixel(56, 180, 204);
	palScreen[0x2D] = olc::Pixel(60, 60, 60);
	palScreen[0x2E] = olc::Pixel(0, 0, 0);
	palScreen[0x2F] = olc::Pixel(0, 0, 0);

	palScreen[0x30] = olc::Pixel(236, 238, 236);
	palScreen[0x31] = olc::Pixel(168, 204, 236);
	palScreen[0x32] = olc::Pixel(188, 188, 236);
	palScreen[0x33] = olc::Pixel(212, 178, 236);
	palScreen[0x34] = olc::Pixel(236, 174, 236);
	palScreen[0x35] = olc::Pixel(236, 174, 212);
	palScreen[0x36] = olc::Pixel(236, 180, 176);
	palScreen[0x37] = olc::Pixel(228, 196, 144);
	palScreen[0x38] = olc::Pixel(204, 210, 120);
	palScreen[0x39] = olc::Pixel(180, 222, 120);
	palScreen[0x3A] = olc::Pixel(168, 226, 144);
	palScreen[0x3B] = olc::Pixel(152, 226, 180);
	palScreen[0x3C] = olc::Pixel(160, 214, 228);
	palScreen[0x3D] = olc::Pixel(160, 162, 160);
	palScreen[0x3E] = olc::Pixel(0, 0, 0);
	palScreen[0x3F] = olc::Pixel(0, 0, 0);

	reset();

#if defined(PPU_FILE_LOG) || defined(PPU_FILE_LOG2)
	ppuLogFile.open("ppu_log.txt");
#endif

}

PPU::~PPU()
{
#ifdef defined(PPU_FILE_LOG) || defined(PPU_FILE_LOG2)
	ppuLogFile.close();
#endif
}

void PPU::reset() {

	// Initialize values

	_addrScrollLatch = PPU_HI_ADDR_WR_STATE;
	_dataBuffer = 0x00;

	_scanline = 0;
	_scanlineDot = 0;

	_bgTileIdNxt = 0x00;
	_bgTileAttrByteNxt = 0x00;

	_bgNxt8pxPaletteId = 0x00;
	_bgNxt8pxColorIdLsb = 0x00;
	_bgNxt8pxColorIdMsb = 0x00;

	_bg16pxPaletteIdLsbPipe = 0x00;
	_bg16pxPaletteIdMsbPipe = 0x00;
	_bg16pxColorIdLsbPipe = 0x00;
	_bg16pxColorIdMsbPipe = 0x00;

	_vramAddr.raw = 0x0000;
	_tmpVramAddr.raw = 0x0000;
	_fineX = 0;

	_controlReg.raw = 0x00;
	_maskReg.raw = 0x00;
	_statusReg.raw = 0x00;

	_oddFrameSwitch = true;

	_frameCounter = 1;

	_8pxBatchReady = false;

	_scanlineSpriteCnt = 0;

}

void PPU::connectConsole(Bus* bus)
{
	_nes = bus;
}

void PPU::connectCartridge(const std::shared_ptr<Cartridge>& cartridge)
{
	_cartridge = cartridge;

	if (cartridge->_cartridgeHeader.tvSystem1 == 0) {
		std::cout << "NTSC";
		// Is NTSC
		_ppuConfig.isNTSC = true;
		_ppuConfig.totalScanlines = 261;
		_ppuConfig.nmiScanline = 241;
		_ppuConfig.lastDrawableScanline = 239;
		_ppuConfig.postRenderScanline = 240;

	} else {
		std::cout << "PAL";
		_ppuConfig.isNTSC = false;
		_ppuConfig.totalScanlines = 261;
		_ppuConfig.nmiScanline = 241;
		_ppuConfig.lastDrawableScanline = 239;
		_ppuConfig.postRenderScanline = 240;

	}

}

uint8_t PPU::cpuRead(uint16_t addr, bool readOnly)
{
	uint8_t data = 0x00;

#ifdef PPU_DEBUG_MODE
	if (readOnly) {
		switch (addr) {
		case CPU_ADDR_SPACE_PPU_PPU_CTRL:
			data = control_reg.raw;
			break;
		case CPU_ADDR_SPACE_PPU_PPU_MASK:
			data = mask_reg.raw;
			break;
		case CPU_ADDR_SPACE_PPU_STATUS_REG:
			data = status_reg.raw;
			break;
		case CPU_ADDR_SPACE_PPU_SPRITE_MEM_ADDR:
			break;
		case CPU_ADDR_SPACE_PPU_SPRITE_MEM_DATA:
			break;
		case CPU_ADDR_SPACE_PPU_BG_SCROLL:
			break;
		case CPU_ADDR_SPACE_PPU_VRAM_ADDR:
			break;
		case CPU_ADDR_SPACE_PPU_VRAM_DATA:
			break;
		}
	}
	else {
#endif
		switch (addr) {
		case CPU_ADDR_SPACE_PPU_PPU_CTRL: // WRITE-ONLY
			break;
		case CPU_ADDR_SPACE_PPU_PPU_MASK: // WRITE-ONLY
			break;
		case CPU_ADDR_SPACE_PPU_STATUS_REG:
			data = (_statusReg.raw & 0b11100000) | (_dataBuffer & 0b00011111); // Return noise in lower 5 bits
			_LOG("cpuRead()/STATUS_REG: read 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
			// Clear bit 7
			_statusReg.verticalBlank = 0;
			_nes->_cpu._nmiOccurred = false;
			// Transition to initial state
			_addrScrollLatch = PPU_HI_ADDR_WR_STATE;
			break;
		case CPU_ADDR_SPACE_PPU_SPRITE_MEM_ADDR: // WRITE-ONLY
			break;
		case CPU_ADDR_SPACE_PPU_SPRITE_MEM_DATA: // WRITE-ONLY
			break;
		case CPU_ADDR_SPACE_PPU_BG_SCROLL:
			break;
		case CPU_ADDR_SPACE_PPU_VRAM_ADDR:
			break;
		case CPU_ADDR_SPACE_PPU_VRAM_DATA:
			// Delay 1 cycle
			data = _dataBuffer;
			// Read
			_dataBuffer = this->ppuRead(_vramAddr.raw, false);
			// If reading from the palettes, do not delay
			if (_IS_PALETTE_ADDR(_vramAddr.raw)) {
				data = _dataBuffer;
			}
			_LOG("cpuRead()/VRAM_DATA: read 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
			// Auto-increment nametable address (loopy address)
			if (_controlReg.vramAddrIncrementPerCpuRw) {
				// Go to the next row
				_vramAddr.raw += PPU_NAME_TABLE_COLS_PER_ROW;
			}
			else {
				// Go to the next row
				_vramAddr.raw += 1;
			}
			//vram_addr.raw &= 0x3FFF;
			_LOG("cpuRead():new vram_addr: " << std::hex << (uint32_t)_vramAddr.raw << std::endl);
			break;
		}
#ifdef PPU_DEBUG_MODE
	}
#endif


	return data;
}

void PPU::cpuWrite(uint16_t addr, uint8_t data)
{
	switch (addr) {
	case CPU_ADDR_SPACE_PPU_PPU_CTRL:
		_LOG("cpuWrite()/control_reg: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		_controlReg.raw = data;
		//
		_tmpVramAddr.nametableX = _controlReg.baseNametableAddr & 0x1;
		_tmpVramAddr.nametableY = (_controlReg.baseNametableAddr >> 1) & 0x1;
		break;
	case CPU_ADDR_SPACE_PPU_PPU_MASK:
		_LOG("cpuWrite()/mask_reg: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		_maskReg.raw = data;
		break;
	case CPU_ADDR_SPACE_PPU_STATUS_REG:
		break;
	case CPU_ADDR_SPACE_PPU_SPRITE_MEM_ADDR: // OAM
		_oamAddr = data;
		//_LOG2("New OAM addr: 0x" << std::hex << (uint16_t)_oamAddr << std::endl);
		break;
	case CPU_ADDR_SPACE_PPU_SPRITE_MEM_DATA: // OAM
		// See https://wiki.nesdev.com/w/index.php/PPU_registers#OAMDATA
		if ((_maskReg.showBg || _maskReg.showSpr) && _scanline >= -1 && _scanline <= _ppuConfig.lastDrawableScanline) {
			// Glitchy addr increment, only 6 msbs (sprite dsc address)
			uint8_t aux = (_oamAddr >> 2) + 1;
			_oamAddr = (_oamAddr & 0x00000011) | (aux << 2);
		}
		else {
			_oamMem.raw[_oamAddr] = data;
			_LOG2("OAM write 0x" << std::hex << (uint16_t)data << " @ " << std::hex << (uint16_t)_oamAddr << std::endl);
			_oamAddr++; // Carelessly increment, since it'll wrap around at 256
		}
		break;
	case CPU_ADDR_SPACE_PPU_BG_SCROLL: // Set top-left corner of the screen

		_LOG("cpuWrite()/scroll: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		if (_addrScrollLatch == PPU_HI_ADDR_WR_STATE) {
			// Transition to next state
			_addrScrollLatch = PPU_LO_ADDR_WR_STATE;

			_fineX = (uint16_t)data & 0x7;
			_tmpVramAddr.coarseX = ((uint16_t)data >> 3) & 0b00011111; // 0x1F
		} else {
			// Transition to initial state
			_addrScrollLatch = PPU_HI_ADDR_WR_STATE;

			_tmpVramAddr.fineY = (uint16_t)data & 0x7;
			_tmpVramAddr.coarseY = ((uint16_t)data >> 3) & 0b00011111; // 0x1F
		}
		break;
	case CPU_ADDR_SPACE_PPU_VRAM_ADDR: // Access the PPU's RAM via this address
		if (_addrScrollLatch == PPU_HI_ADDR_WR_STATE) {
			// Transition to next state
			_addrScrollLatch = PPU_LO_ADDR_WR_STATE;
			// Set hi part
			_tmpVramAddr.raw &= 0x00FF;
			_tmpVramAddr.raw |= ((uint16_t)data & 0x003F) << 8;
			_LOG("cpuWrite():PPU_VRAM_ADDR HI:data=" << std::hex << (uint16_t)data << ", new vram_addr.raw: " << std::hex << (uint32_t)_vramAddr.raw << std::endl);
		} else {
			// Go to initial state
			_addrScrollLatch = PPU_HI_ADDR_WR_STATE;
			// Set lo part
			_tmpVramAddr.raw &= 0xFF00;
			_tmpVramAddr.raw |= data;
			// Set active address
			_vramAddr.raw = _tmpVramAddr.raw;
			_LOG("cpuWrite():PPU_VRAM_ADDR LO:data=" << std::hex << (uint16_t)data << ", new vram_addr.raw: " << std::hex << (uint32_t)_vramAddr.raw << std::endl);
		}
		break;
	case CPU_ADDR_SPACE_PPU_VRAM_DATA: // NAMETABLE
		// Write
		this->ppuWrite(_vramAddr.raw, data);
		
		if (_controlReg.vramAddrIncrementPerCpuRw) {
			// Go to the next row
			_vramAddr.raw += PPU_NAME_TABLE_COLS_PER_ROW;
		} else {
			// Go to the next column
			_vramAddr.raw += 1;
		}
		//vram_addr.raw &= 0x3FFF;
		_LOG("cpuWrite():PPU_VRAM_DATA:new vram_addr.raw: " << std::hex << (uint32_t)_vramAddr.raw << std::endl);
		break;
	}
}

uint8_t PPU::ppuRead(uint16_t addr, bool readOnly)
{
	uint8_t readData = 0;
	addr &= PPU_ADDR_SPACE_MASK;

	if (this->_cartridge->ppuRead(addr, readData)) {

	}
	else if (_IS_PATTERN_TABLE_ADDR(addr)) {
		uint16_t patternTableId = (addr >> 12) & 0x1;
		readData = this->_patternTables[patternTableId][addr & PPU_PATTERN_TABLE_MASK];
	}
	else if (addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= (PPU_ADDR_SPACE_PALETTES_REGION_START - 1)) {
	//else if (_IS_NAMETABLE_ADDR(addr)) {
		
		addr = PPU_ADDR_SPACE_NAME_TABLE_0_START + (addr & PPU_NAME_TABLE_REGION_MASK);

		if (this->_cartridge->_vertical) { // Vertical mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				readData = this->_nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				readData = this->_nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				readData = this->_nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else {
				readData = this->_nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}
		}
		else { // Horizontal mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				readData = this->_nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				readData = this->_nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				readData = this->_nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}
			else {
				readData = this->_nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}

		}

	}
	else if (_IS_PALETTE_ADDR(addr)) {

		uint16_t aux_addr = addr & PPU_ADDR_SPACE_PALETTES_REGION_MASK;

		// Reflect mirroring in palette 2
		// If the address is a multiple of 4 (thus pointing to
		// the bg color), apply modulo 16
		if ((aux_addr % 4) == 0) {
			aux_addr %= PPU_PALETTE_SIZE;
		}

		readData = _paletteMem[aux_addr] & (_maskReg.grayscale ? 0x30 : 0x3F);
		
	}

	return readData;
}

void PPU::ppuWrite(uint16_t addr, uint8_t data)
{

	addr &= PPU_ADDR_SPACE_MASK;

	if (this->_cartridge->ppuWrite(addr & PPU_ADDR_SPACE_MASK, data)) {

	}
	else if (_IS_PATTERN_TABLE_ADDR(addr)) {
		uint16_t patternTableId = (addr >> 12) & 0x1;
		this->_patternTables[patternTableId][addr & PPU_PATTERN_TABLE_MASK];
	}
	else if ( addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= (PPU_ADDR_SPACE_PALETTES_REGION_START - 1) ) {
	//else if (_IS_NAMETABLE_ADDR(addr)) {

		addr = PPU_ADDR_SPACE_NAME_TABLE_0_START + (addr & PPU_NAME_TABLE_REGION_MASK);

		_LOG("ppuWrite()/nametable: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		if (this->_cartridge->_vertical) { // Vertical mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				this->_nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				this->_nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				this->_nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else {
				this->_nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
		}
		else { // Horizontal mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				this->_nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				this->_nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				this->_nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else {
				this->_nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
		}

	}
	else if (_IS_PALETTE_ADDR(addr)) {

		uint16_t aux_addr = addr & PPU_ADDR_SPACE_PALETTES_REGION_MASK;

		// Reflect mirroring in palette 2
		// If the address is a multiple of 4 (thus pointing to
		// the bg color), apply modulo 16
		if ((aux_addr % 4) == 0) {
			aux_addr %= PPU_PALETTE_SIZE;
		}
		
		_paletteMem[aux_addr] = data;
		_LOG("ppuWrite()/palette: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << " / 0x" << std::hex << aux_addr << ", NEW VALUE: " << std::hex << (uint16_t)_paletteMem[aux_addr] << std::endl);

	}

}


void PPU::clock() {

	uint16_t bgTileIdAddr;
	uint16_t supertileX;
	uint16_t supertileY;
	uint16_t supertileId;
	uint16_t supertileAttrByteAddr;
	uint16_t tileLsbAddr;
	uint16_t tileMsbAddr;


	// Is it a drawable scanline?
	if (_scanline >= -1 && _scanline <= _ppuConfig.lastDrawableScanline) {

		// Pre-render line (-1)
		if (_scanline == -1 && _scanlineDot == 1) {
			// Beginning of frame... Set vertical blank to 0.
			_statusReg.verticalBlank = 0;
			_nes->_cpu._nmiOccurred = false;
			_spriteZeroRendered = false;
			_statusReg.sprZeroHit = 0;
		}


		if ((_scanlineDot >= 1 && _scanlineDot <= 257) || (_scanlineDot >= 321 && _scanlineDot <= 336)) {

			////////////////////////////////////////////
			///// BACKGROUND RENDERING
			////////////////////////////////////////////

			if (_maskReg.showBg) {
				MOVE_BG_PIPES();
			}

			if (_maskReg.showSpr && _scanlineDot <= 256) {

				for (uint16_t spriteIdx = 0; spriteIdx < _scanlineSpriteCnt; spriteIdx++) {

					if (_secOamMem.sprites[spriteIdx].xPos > 0) {
						_secOamMem.sprites[spriteIdx].xPos--;
					}
					else {
						_spritePixelsLsbPipe[spriteIdx] <<= 1;
						_spritePixelsMsbPipe[spriteIdx] <<= 1;
					}

				}

			}

			uint16_t tilePixel = (_scanlineDot - 1) % 8;

			switch (tilePixel) {
			case 1:

				// Get the id of the next tile
				bgTileIdAddr =
					PPU_ADDR_SPACE_NAME_TABLE_REGION_START + (_vramAddr.raw & PPU_NAME_TABLE_REGION_MASK);
				_bgTileIdNxt = ppuRead(bgTileIdAddr); // Is a number in [0, 255]
				_LOG("\n--> FC: " << std::dec << _frameCounter << ",  scanline: " << std::dec << _scanline << ", scanline_dot: " << std::dec << (uint32_t)_scanlineDot << "*" << std::endl);
				_LOG("bgTileIdAddr: " << std::hex << (uint32_t)bgTileIdAddr << std::endl);
				_LOG("_bgTileIdNxt: " << std::dec << (uint32_t)_bgTileIdNxt << std::endl);

				break;

			case 3:

				// A supertile (my own nomenclature here) is 2x2 metatiles, and a metatile is 2x2 tiles.
				// One attribute table byte contains the palette ids of 4 metatiles, or 1 supertile.
				supertileX = _vramAddr.coarseX >> 2; // coarseX indicates a tile
				supertileY = _vramAddr.coarseY >> 2;
				// This id identifies a supertile with a number in [0, 63]
				supertileId = (supertileY << 3) | supertileX;
				// Get the attribute table byte defining the (4) palette ids for this supertile
				supertileAttrByteAddr = PPU_ADDR_SPACE_NAME_TABLE_REGION_START
					+ (_vramAddr.raw & 0x0C00) // Identifies the nametable
					+ (PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET
						+ supertileId); // Identifies a byte in a nametable

				_bgNxt8pxPaletteId = ppuRead(supertileAttrByteAddr);

				_LOG("_vramAddr.coarseX: " << std::dec << (uint32_t)_vramAddr.coarseX << std::endl);
				_LOG("_vramAddr.coarseY: " << std::dec << (uint32_t)_vramAddr.coarseY << std::endl);
				_LOG("supertileX: " << std::dec << (uint32_t)supertileX << std::endl);
				_LOG("supertileY: " << std::dec << (uint32_t)supertileY << std::endl);
				_LOG("supertileId: " << std::dec << (uint32_t)supertileId << std::endl);
				_LOG("(_vramAddr.raw): " << std::hex << (uint32_t)(_vramAddr.raw) << std::endl);
				_LOG("supertileAttrByteAddr: 0x" << std::hex << (uint32_t)supertileAttrByteAddr << std::endl);
				_LOG("_bgNxt8pxPaletteId: 0x" << std::hex << (uint32_t)_bgNxt8pxPaletteId << std::endl);

				// Now, get palette id for the 8 pixels which are currently being calculated
				// This is a function of which metatile said 8 pixels are a part of
				if (_vramAddr.coarseX & 0b10) _bgNxt8pxPaletteId >>= PPU_PALETTE_ID_BIT_LEN;
				if (_vramAddr.coarseY & 0b10) _bgNxt8pxPaletteId >>= (PPU_PALETTE_ID_BIT_LEN * 2);
				_bgNxt8pxPaletteId &= PPU_PALETTE_ID_MASK;

				break;

			case 5:

				tileLsbAddr =
					PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START + // Offset is 0... Useless calculation!
					(_controlReg.bgPatternTableAddr << 12) + // Select pattern table by multiplying by 4 KiB
					(_bgTileIdNxt << 4) + // Tile id * 16
					_vramAddr.fineY; // Select which 8 pixels
				_bgNxt8pxColorIdLsb = ppuRead(tileLsbAddr);

				break;

			case 7:

				tileMsbAddr =
					PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START + // Offset is 0... Useless calculation!
					(_controlReg.bgPatternTableAddr << 12) + // Select pattern table by multiplying by 4 KiB
					(_bgTileIdNxt << 4) + // Tile id * 16
					_vramAddr.fineY + // Select which 8 pixels
					8; // MSB plane
				_bgNxt8pxColorIdMsb = ppuRead(tileMsbAddr);

				if (_maskReg.showBg || _maskReg.showSpr) {

					if (_scanlineDot == 256) {
						INCREMENT_Y();
						_LOG("INCREMENT_Y: _vramAddr.raw: " << std::dec << (uint32_t)_vramAddr.raw << std::endl);
					}
					else {
						INCREMENT_X();
						_LOG("INCREMENT_X: _vramAddr.raw: " << std::dec << (uint32_t)_vramAddr.raw << std::endl);
					}

					// Ready!
					_8pxBatchReady = true;

				}

				break;

			}

		}

		if (_8pxBatchReady) {
			_8pxBatchReady = false;
			PUSH_PALETTE_ID_LSBS_TO_PIPE(_bgNxt8pxPaletteId);
			PUSH_PALETTE_ID_MSBS_TO_PIPE(_bgNxt8pxPaletteId);
			PUSH_COLOR_ID_LSBS_TO_PIPE(_bgNxt8pxColorIdLsb);
			PUSH_COLOR_ID_MSBS_TO_PIPE(_bgNxt8pxColorIdMsb);
		}

		if (_scanlineDot == 257) {

			if (_maskReg.showBg || _maskReg.showSpr) {
				TRANSFER_ADDR_X();
				_LOG("TRANSFER_ADDR_X: _vramAddr.raw: " << std::dec << (uint32_t)_vramAddr.raw << std::endl);
			}
			
		}

		if (IS_PRERENDER_SCANLINE() && _maskReg.showBg && _ppuConfig.isNTSC && _oddFrameSwitch && _scanlineDot == 339) {
			_scanlineDot = 340;
		}

		/*if (_scanlineDot == 338 || _scanlineDot == 340) {

			// Unused NT fetches
			uint16_t bg_tile_id_addr =
				PPU_ADDR_SPACE_NAME_TABLE_REGION_START + (_vramAddr.raw & PPU_NAME_TABLE_REGION_MASK);
			_bgTileIdNxt = ppuRead(bg_tile_id_addr); // Is a number in [0, 255]

		}*/

		if (_scanline == -1) {

			if (_scanlineDot >= 280 && _scanlineDot <= 304) {
				if (_maskReg.showBg || _maskReg.showSpr) {
					TRANSFER_ADDR_Y();
					//_LOG("TRANSFER_ADDR_Y: _vramAddr.raw: " << std::dec << (uint32_t)_vramAddr.raw << std::endl);
				}
			}

		}



		
	}

	////////////////////////////////////////////
	///// FOREGROUND RENDERING
	////////////////////////////////////////////

	if (_scanline >= -1 && _scanline <= _ppuConfig.lastDrawableScanline) {

		// Dots 1-64: Secondary OAM clear

		if (_scanline >= 0 && _scanlineDot == 64) {
			//memset(&_secOamMem.raw, 0xFF, 8);
		}

		// Sprite evaluation (cycle 65-240)

		if (_scanline >= 0 && _scanlineDot == 257) {

			memset(&_secOamMem.raw, 0xFF, 8);
			memset(_spritePixelsLsbPipe, 0x0, 8);
			memset(_spritePixelsMsbPipe, 0x0, 8);
			_scanlineSpriteCnt = 0;

			uint16_t spriteHeight = _controlReg.sprSize ? 16 : 8;

			for (uint16_t spriteIdx = 0; spriteIdx < 64; spriteIdx++) {

				if (_scanline >= _oamMem.sprites[spriteIdx].yPos
					&& _scanline < (_oamMem.sprites[spriteIdx].yPos + spriteHeight)) {

					if (_scanlineSpriteCnt < 8) {

						_secOamMem.sprites[_scanlineSpriteCnt++] = _oamMem.sprites[spriteIdx];
						_spriteZeroRendered = spriteIdx == 0; // Sprite 0 will be rendered

					}
					else {
						_statusReg.sprOverflow = 1;
						break;
					}

				}

			}

		}

		// Sprite fetches (cycle 257-320)

		if (_scanlineDot == 320) {

			uint16_t sprite_8px_lo_addr;

			for (uint16_t spriteIdx = 0; spriteIdx < _scanlineSpriteCnt; spriteIdx++) {

				int16_t spriteYPosScanlineDistance = (int16_t)_scanline - (int16_t)_secOamMem.sprites[spriteIdx].yPos;

				if (_controlReg.sprSize == 0) { // 8x8

					if (!_secOamMem.sprites[spriteIdx].verticalFlip) { // Default vertical orientation

						sprite_8px_lo_addr = (_controlReg.sprPatternTableAddrFor8x8Mode << 12)
							+ ((uint16_t)_secOamMem.sprites[spriteIdx].id << 4) // * 16 bytes
							+ (uint16_t)spriteYPosScanlineDistance;

					}
					else { // Vertical flip

						sprite_8px_lo_addr = (_controlReg.sprPatternTableAddrFor8x8Mode << 12)
							+ ((uint16_t)_secOamMem.sprites[spriteIdx].id << 4) // * 16 bytes
							+ (7 - (uint16_t)spriteYPosScanlineDistance);

					}

				}
				else { // 8x16

					// A 8x16 sprite is composed of two half sprites.
					// The distance between the Y pos of the LSB plane of any of the 2 half sprites,
					// and the current scanline, will be a value in: [0, 7], [15, 23]
					bool isTopHalfSprite = spriteYPosScanlineDistance > 7;

					// Distance from the Y pos of the half sprite being rendered, and the current scanline
					uint16_t halfSpriteYPosScanlineDistance = spriteYPosScanlineDistance & 0x7;

					if (!_secOamMem.sprites[spriteIdx].verticalFlip) { // Default vertical orientation

						if (isTopHalfSprite) {

							sprite_8px_lo_addr = ((_secOamMem.sprites[spriteIdx].id & 0x1) << 12)
								+ (((uint16_t)_secOamMem.sprites[spriteIdx].id & 0xFE) << 4) // * 16 bytes
								+ (uint16_t)halfSpriteYPosScanlineDistance;

						}
						else { // Bottom half sprite

							sprite_8px_lo_addr = ((_secOamMem.sprites[spriteIdx].id & 0x1) << 12)
								+ (((uint16_t)_secOamMem.sprites[spriteIdx].id & 0xFE) << 4) // * 16 bytes
								+ 16
								+ (uint16_t)halfSpriteYPosScanlineDistance;

						}

					}
					else { // Vertical flip

						if (isTopHalfSprite) {

							sprite_8px_lo_addr = ((_secOamMem.sprites[spriteIdx].id & 0x1) << 12)
								+ (((uint16_t)_secOamMem.sprites[spriteIdx].id & 0xFE) << 4) // * 16 bytes
								+ (7 - (uint16_t)halfSpriteYPosScanlineDistance);

						}
						else { // Bottom half sprite

							sprite_8px_lo_addr = ((_secOamMem.sprites[spriteIdx].id & 0x1) << 12)
								+ (((uint16_t)_secOamMem.sprites[spriteIdx].id & 0xFE) << 4) // * 16 bytes
								+ 16
								+ (uint16_t)halfSpriteYPosScanlineDistance;

						}

					}

				}

				uint8_t spritePatternBitsLo = ppuRead(sprite_8px_lo_addr);
				uint8_t spritePatternBitsHi = ppuRead(sprite_8px_lo_addr + 8);

				if (_secOamMem.sprites[spriteIdx].horizontalFlip) {
					REVERSE(spritePatternBitsLo);
					REVERSE(spritePatternBitsHi);
				}

				_spritePixelsLsbPipe[spriteIdx] = spritePatternBitsLo;
				_spritePixelsMsbPipe[spriteIdx] = spritePatternBitsHi;

			}

		}

	}

	if (_scanline == _ppuConfig.postRenderScanline && _scanlineDot == 0) { // Post-render scanline
		_frameCounter++; // This has no emulation function
		//std::cout << "New frame: " << _frameCounter << std::endl;
		_oddFrameSwitch = !_oddFrameSwitch; // Reverse
	}

	if (_scanline == _ppuConfig.nmiScanline && _scanlineDot == 1) {
		_statusReg.verticalBlank = 1;
		_nes->_cpu._nmiOccurred = _controlReg.nmiAtVBlankIntervalStart ? true : false;
	}

	/************ Compose image *************/

	uint8_t pixel = 0x00;
	uint8_t palette = 0x00;

	uint8_t bg_pixel = 0x00;
	uint8_t bg_palette = 0x00;
	uint8_t fg_pixel = 0x00;
	uint8_t fg_palette = 0x00;
	uint8_t fg_priority = 0x00;

	if (_maskReg.showBg) {

		uint16_t pixel_selector = 0x8000 >> _fineX;

		uint8_t bg_pixel_lsb = (_bg16pxColorIdLsbPipe & pixel_selector) > 0;
		uint8_t bg_pixel_msb = (_bg16pxColorIdMsbPipe & pixel_selector) > 0;
		bg_pixel = (bg_pixel_msb << 1) | bg_pixel_lsb;

		uint8_t bg_palette_lsb = (_bg16pxPaletteIdLsbPipe & pixel_selector) > 0;
		uint8_t bg_palette_msb = (_bg16pxPaletteIdMsbPipe & pixel_selector) > 0;
		bg_palette = (bg_palette_msb << 1) | bg_palette_lsb;

		//MOVE_BG_PIPES();

	} else {
		// https://wiki.nesdev.com/w/index.php/PPU_palettes
		if ((_vramAddr.raw & 0x3F00) == 0x3F00) {
			bg_pixel = 0x1F & _vramAddr.raw;
		}
	}

	if (_maskReg.showSpr) {

		for (uint16_t spriteIdx = 0; spriteIdx < _scanlineSpriteCnt; spriteIdx++) {

			if (_secOamMem.sprites[spriteIdx].xPos == 0) {

				if (_spritePixelsLsbPipe[spriteIdx] == 0x7c) {
					_spritePixelsLsbPipe[spriteIdx] = 0x7c;
				}

				uint8_t fg_pixel_lo = (_spritePixelsLsbPipe[spriteIdx] & 0x80) > 0;
				uint8_t fg_pixel_hi = (_spritePixelsMsbPipe[spriteIdx] & 0x80) > 0;

				fg_pixel = (fg_pixel_hi << 1) | fg_pixel_lo;
				fg_palette = _secOamMem.sprites[spriteIdx].palette + 4; // Palettes 0-3: bg, palettes 4-7: sprites
				fg_priority = _secOamMem.sprites[spriteIdx].priority;

				if (fg_pixel != 0) {

					// Sprite 0 hit does not happen:
					// At x=0 to x=7 if the left-side clipping window is enabled (if bit 2 or bit 1 of PPUMASK is 0).
					bool cond1 = (_maskReg.showSprLeft == 0 || _maskReg.showBgLeft == 0) && _scanlineDot >= 0 && _scanlineDot < 8;
					// At x=255, for an obscure reason related to the pixel pipeline.
					bool cond2 = _scanlineDot == 255;

					if (bg_pixel != 0 && spriteIdx == 0 && _spriteZeroRendered && !cond1 && !cond2) {
						_statusReg.sprZeroHit = 1;
						_spriteZeroRendered = false;
					}

					break;
				}

			}

		}

	}

	if (bg_pixel > 0 && fg_pixel == 0) {
		pixel = bg_pixel;
		palette = bg_palette;
	}else if (bg_pixel == 0 && fg_pixel > 0) {
		pixel = fg_pixel;
		palette = fg_palette;
	}
	else if (bg_pixel > 0 && fg_pixel > 0) {

		if (!fg_priority) {
			pixel = fg_pixel;
			palette = fg_palette;
		}
		else {
			pixel = bg_pixel;
			palette = bg_palette;
		}

	}

	sprScreen.SetPixel(_scanlineDot - 1, _scanline, GetColourFromPaletteRam(palette, pixel));

	_scanlineDot++;
	if (_scanlineDot > 340)
	{
		_scanlineDot = 0;
		_scanline++;
		if (_scanline >= 261)
		{
			_scanline = -1;
			_frameComplete = true;
		}
	}

	// Debug
	_debugPPUState.scanline = _scanline;
	_debugPPUState.scanlineDot = _scanlineDot;
	_debugPPUState.frameCounter = _frameCounter;
	_debugPPUState.maskReg = _maskReg;
	_debugPPUState.controlReg = _controlReg;
	_debugPPUState.statusReg = _statusReg;
	_debugPPUState.fineX = _fineX;
	_debugPPUState.vramAddr = _vramAddr;
	_debugPPUState.tmpVramAddr = _tmpVramAddr;

}

olc::Pixel& PPU::GetColourFromPaletteRam(uint8_t palette, uint8_t pixel)
{
	// This is a convenience function that takes a specified palette and pixel
	// index and returns the appropriate screen colour.
	// "0x3F00"       - Offset into PPU addressable range where palettes are stored
	// "palette << 2" - Each palette is 4 bytes in size
	// "pixel"        - Each pixel index is either 0, 1, 2 or 3
	// "& 0x3F"       - Stops us reading beyond the bounds of the palScreen array
	return palScreen[ppuRead(0x3F00 + (palette << 2) + pixel) & 0x3F];

	// Note: We dont access tblPalette directly here, instead we know that ppuRead()
	// will map the address onto the seperate small RAM attached to the PPU bus.
}

olc::Sprite& PPU::GetScreen()
{
	return sprScreen;
}

void printBuffer(uint16_t startAddr, uint16_t endAddr, uint8_t* buffer) {
	uint8_t datum = 0;
	_LOG2("***** BUFFER *****" << std::endl);
	for (uint16_t currAddr = startAddr; currAddr < endAddr; currAddr++) {
		datum = buffer[currAddr];
		_LOG2("@0x" << std::hex << currAddr << ": 0x" << std::hex << (uint16_t)datum << std::endl);
	}
}

void PPU::printPPURamRange(uint16_t startAddr, uint16_t endAddr) {
	uint8_t datum = 0;
	_LOG("***** PPU MEM SPACE *****" << std::endl);
	for (uint16_t currAddr = startAddr; currAddr < endAddr; currAddr++) {
		datum = ppuRead(currAddr);
		_LOG("@0x" << std::hex << currAddr << ": 0x" << std::hex << (uint16_t)datum << std::endl);
	}
}

debug_ppu_state_dsc_st PPU::getDebugPPUstate()
{
	return this->_debugPPUState;
}
