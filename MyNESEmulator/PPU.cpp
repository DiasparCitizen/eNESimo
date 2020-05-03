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
	_scanlineCycle = 0;

	_bgTileIdNxt = 0x00;
	_bgTileAttrByteNxt = 0x00;

	_bgNxt8pxPaletteId = 0x00;
	_bgNxt8pxColorIdLsb = 0x00;
	_bgNxt8pxColorIdMsb = 0x00;

	_bg16pxPaletteIdLsbPipe = 0x0000;
	_bg16pxPaletteIdMsbPipe = 0x0000;
	_bg16pxColorIdLsbPipe = 0x0000;
	_bg16pxColorIdMsbPipe = 0x0000;

	_vramAddr.raw = 0x0000;
	_tmpVramAddr.raw = 0x0000;
	_fineX = 0;

	_controlReg.raw = 0x00;
	_maskReg.raw = 0x00;
	_statusReg.raw = 0x00;

	_oddFrameSwitch = true;

	_frameCounter = 1;

	_8pxBatchReady = false;

	_foundSpritesCount = 0;

	// Sprite evaluation
	_sprEvalState.state = SpriteEvalState::NORMAL_SEARCH;
	_sprEvalState.oamSpriteIdx = 0;
	_sprEvalState.secOamSpriteIdx = 0;
	_sprEvalState.spriteByteIdx = 0;

	_scanlineSpritesCnt = 0;

}

void PPU::connectConsole(Bus* bus)
{
	_nes = bus;
}

void PPU::connectCartridge(const std::shared_ptr<Cartridge>& cartridge)
{
	_cartridge = cartridge;

	if (cartridge->_cartridgeHeader.tvSystem1 == 0) {
		std::cout << "NTSC\n";
		// Is NTSC
		_ppuConfig.isNTSC = true;
		_ppuConfig.totalScanlines = 261;
		_ppuConfig.nmiScanline = 241;
		_ppuConfig.lastDrawableScanline = 239;
		_ppuConfig.postRenderScanline = 240;

	} else {
		std::cout << "PAL\n";
		_ppuConfig.isNTSC = false;
		_ppuConfig.totalScanlines = 261;
		_ppuConfig.nmiScanline = 241;
		_ppuConfig.lastDrawableScanline = 239;
		_ppuConfig.postRenderScanline = 240;

	}

}

/* CPU INTERFACE */

void PPU::writeControlReg(uint8_t data)
{
	_controlReg.raw = data;
	//
	_tmpVramAddr.nametableX = _controlReg.baseNametableAddr & 0x1;
	_tmpVramAddr.nametableY = (_controlReg.baseNametableAddr >> 1) & 0x1;
}

void PPU::writeMaskReg(uint8_t data)
{
	_maskReg.raw = data;
}

void PPU::writeStatusReg(uint8_t data) {}

void PPU::writeSpriteMemAddr(uint8_t data)
{
	_oamAddr = data;
	//_LOG2("New OAM addr: 0x" << std::hex << (uint16_t)_oamAddr << std::endl);
}

void PPU::writeSpriteMemData(uint8_t data)
{
	// See https://wiki.nesdev.com/w/index.php/PPU_registers#OAMDATA
	if ((_maskReg.showBg || _maskReg.showSpr) && _scanline >= -1 && _scanline <= _ppuConfig.lastDrawableScanline) {
		// Glitchy addr increment, only 6 msbs (sprite dsc address)
		uint8_t aux = (_oamAddr >> 2) + 1;
		_oamAddr = (_oamAddr & 0x00000011) | (aux << 2);
	}
	else {
		_oamMem.raw[_oamAddr] = data;
		_oamAddr++; // Carelessly increment, since it'll wrap around at 256
	}
}

void PPU::writeBgScroll(uint8_t data)
{
	if (_addrScrollLatch == PPU_HI_ADDR_WR_STATE) {
		// Transition to next state
		_addrScrollLatch = PPU_LO_ADDR_WR_STATE;

		_fineX = (uint16_t)data & 0x7;
		_tmpVramAddr.coarseX = ((uint16_t)data >> 3) & 0b00011111; // 0x1F
	}
	else {
		// Transition to initial state
		_addrScrollLatch = PPU_HI_ADDR_WR_STATE;

		_tmpVramAddr.fineY = (uint16_t)data & 0x7;
		_tmpVramAddr.coarseY = ((uint16_t)data >> 3) & 0b00011111; // 0x1F
	}
}

void PPU::writeVramAddr(uint8_t data)
{
	if (_addrScrollLatch == PPU_HI_ADDR_WR_STATE) {
		// Transition to next state
		_addrScrollLatch = PPU_LO_ADDR_WR_STATE;
		// Set hi part
		_tmpVramAddr.raw &= 0x00FF;
		_tmpVramAddr.raw |= ((uint16_t)data & 0x003F) << 8;
	}
	else {
		// Go to initial state
		_addrScrollLatch = PPU_HI_ADDR_WR_STATE;
		// Set lo part
		_tmpVramAddr.raw &= 0xFF00;
		_tmpVramAddr.raw |= data;
		// Set active address
		_vramAddr.raw = _tmpVramAddr.raw;
	}
}

void PPU::writeVramData(uint8_t data)
{
	this->ppuWrite(_vramAddr.raw, data);

	if (_controlReg.vramAddrIncrementPerCpuRw) {
		// Go to the next row
		_vramAddr.raw += PPU_NAME_TABLE_COLS_PER_ROW;
	}
	else {
		// Go to the next column
		_vramAddr.raw += 1;
	}
	//vram_addr.raw &= 0x3FFF;
}

uint8_t PPU::readStatusReg()
{
	uint8_t data = (_statusReg.raw & 0b11100000) | (_dataBuffer & 0b00011111); // Return noise in lower 5 bits
	// Clear bit 7
	_statusReg.verticalBlank = 0;
	_nes->_cpu._nmiOccurred = false;
	// Transition to initial state
	_addrScrollLatch = PPU_HI_ADDR_WR_STATE;
	return data;
}

uint8_t PPU::readControlReg() { return uint8_t(); }
uint8_t PPU::readMaskReg() { return uint8_t(); }
uint8_t PPU::readSpriteMemAddr() { return uint8_t(); }
uint8_t PPU::readSpriteMemData() { return uint8_t(); }
uint8_t PPU::readBgScroll() { return uint8_t(); }
uint8_t PPU::readVramAddr() { return uint8_t(); }

uint8_t PPU::readVramData()
{
	// Delay 1 cycle
	uint8_t data = _dataBuffer;
	// Read
	_dataBuffer = this->ppuRead(_vramAddr.raw, false);
	// If reading from the palettes, do not delay
	if (_IS_PALETTE_ADDR(_vramAddr.raw)) {
		data = _dataBuffer;
	}

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
	return data;
}

/* INTERNAL PPU COMMUNICATIONS */

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

	}

}


void PPU::clock() {

	uint16_t bgTileIdAddr;
	uint16_t bgSupertileX;
	uint16_t bgSupertileY;
	uint16_t bgSupertileId;
	uint16_t bgSupertileAttrByteAddr;
	uint16_t bgTileLsbAddr;
	uint16_t bgTileMsbAddr;

	// Is it a drawable scanline?
	if (_scanline >= -1 && _scanline <= _ppuConfig.lastDrawableScanline) {

		// Pre-render line (-1)
		if (_scanline == -1 && _scanlineCycle == 1) {
			// Beginning of frame
			_statusReg.verticalBlank = 0;
			_nes->_cpu._nmiOccurred = false;
			_statusReg.sprZeroHit = 0;
			_statusReg.sprOverflow = 0;
			_spriteZeroRenderedThisFrame = false;
		}

		////////////////////////////////////////////
		///// BACKGROUND RENDERING
		////////////////////////////////////////////

		if ((_scanlineCycle >= 1 && _scanlineCycle <= 257) || (_scanlineCycle >= 321 && _scanlineCycle <= 336)) {

			if (_maskReg.showBg) {
				MOVE_BG_PIPES();
			}

			uint16_t tilePixel = (_scanlineCycle - 1) & 0x7; // % 8

			switch (tilePixel) {
			case 1:

				// Get the id of the next tile
				bgTileIdAddr =
					PPU_ADDR_SPACE_NAME_TABLE_REGION_START + (_vramAddr.raw & PPU_NAME_TABLE_REGION_MASK);
				_bgTileIdNxt = ppuRead(bgTileIdAddr); // Is a number in [0, 255]

				break;

			case 3:

				// A supertile (my own nomenclature here) is 2x2 metatiles, and a metatile is 2x2 tiles.
				// One attribute table byte contains the palette ids of 4 metatiles, or 1 supertile.
				bgSupertileX = _vramAddr.coarseX >> 2; // coarseX indicates a tile
				bgSupertileY = _vramAddr.coarseY >> 2;
				// This id identifies a supertile with a number in [0, 63]
				bgSupertileId = (bgSupertileY << 3) | bgSupertileX;
				// Get the attribute table byte defining the (4) palette ids for this supertile
				bgSupertileAttrByteAddr = PPU_ADDR_SPACE_NAME_TABLE_REGION_START
					+ (_vramAddr.raw & 0x0C00) // Identifies the nametable
					+ (PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET
						+ bgSupertileId); // Identifies a byte in a nametable

				_bgNxt8pxPaletteId = ppuRead(bgSupertileAttrByteAddr);

				// Now, get palette id for the 8 pixels which are currently being calculated
				// This is a function of which metatile said 8 pixels are a part of
				if (_vramAddr.coarseX & 0b10) _bgNxt8pxPaletteId >>= PPU_PALETTE_ID_BIT_LEN;
				if (_vramAddr.coarseY & 0b10) _bgNxt8pxPaletteId >>= (PPU_PALETTE_ID_BIT_LEN * 2);
				_bgNxt8pxPaletteId &= PPU_PALETTE_ID_MASK;

				break;

			case 5:

				bgTileLsbAddr =
					PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START + // Offset is 0... Useless calculation!
					(_controlReg.bgPatternTableAddr << 12) + // Select pattern table by multiplying by 4 KiB
					(_bgTileIdNxt << 4) + // Tile id * 16
					_vramAddr.fineY; // Select which 8 pixels
				_bgNxt8pxColorIdLsb = ppuRead(bgTileLsbAddr);

				break;

			case 7:

				bgTileMsbAddr =
					PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START + // Offset is 0... Useless calculation!
					(_controlReg.bgPatternTableAddr << 12) + // Select pattern table by multiplying by 4 KiB
					(_bgTileIdNxt << 4) + // Tile id * 16
					_vramAddr.fineY + // Select which 8 pixels
					8; // MSB plane
				_bgNxt8pxColorIdMsb = ppuRead(bgTileMsbAddr);

				if (_maskReg.showBg || _maskReg.showSpr) {

					if (_scanlineCycle == 256) {
						INCREMENT_Y();
					}
					else {
						INCREMENT_X();
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

		if (_scanline == -1) {

			if (_scanlineCycle >= 280 && _scanlineCycle <= 304) {
				if (_maskReg.showBg || _maskReg.showSpr) {
					TRANSFER_ADDR_Y();
				}
			}

			if (_scanlineCycle == 339 && _maskReg.showBg && _ppuConfig.isNTSC && _oddFrameSwitch) {
				// Skip cycle
				_scanlineCycle = 340;
			}

		}

		if (_scanlineCycle == 257) {

			if (_maskReg.showBg || _maskReg.showSpr) {
				TRANSFER_ADDR_X();
			}

		}

		if (_scanlineCycle == 338 || _scanlineCycle == 340) {

			// Unused NT fetches
			uint16_t bg_tile_id_addr =
				PPU_ADDR_SPACE_NAME_TABLE_REGION_START + (_vramAddr.raw & PPU_NAME_TABLE_REGION_MASK);
			_bgTileIdNxt = ppuRead(bg_tile_id_addr); // Is a number in [0, 255]

		}


		////////////////////////////////////////////
		///// FOREGROUND RENDERING
		////////////////////////////////////////////

		if (_maskReg.showSpr && _scanlineCycle >= 1 && _scanlineCycle <= 257) {

			for (uint16_t spriteIdx = 0; spriteIdx < _scanlineSpritesCnt; spriteIdx++) {

				if (_scanlineSpritesBuffer_xPos[spriteIdx] > 0) {
					_scanlineSpritesBuffer_xPos[spriteIdx]--;
				}
				else {
					_scanlineSpritesBuffer_pixelLsb[spriteIdx] <<= 1;
					_scanlineSpritesBuffer_pixelMsb[spriteIdx] <<= 1;
				}

			}

		}

		// Cycles 1-64: Secondary OAM clear
		if (_scanline >= 0 && _scanlineCycle <= 64) {
			secondaryOAMClear();
		}

		// Cycles 65-256: Sprite Evaluation phase
		if (_scanline >= 0 && _scanlineCycle >= 65 && _scanlineCycle <= 256) {
			spriteEvaluation();
		}

		// Cycles 257-320: Sprite Fetch phase
		if (_scanlineCycle >= 257 && _scanlineCycle <= 320) {
			spriteFetch();
		}

	}

	if (_scanline == _ppuConfig.postRenderScanline && _scanlineCycle == 0) { // Post-render scanline
		_frameCounter++; // This has no emulation function
		_oddFrameSwitch = !_oddFrameSwitch; // Reverse
	}

	if (_scanline == _ppuConfig.nmiScanline && _scanlineCycle == 1) {
		_statusReg.verticalBlank = 1;
		_nes->_cpu._nmiOccurred = _controlReg.nmiAtVBlankIntervalStart ? true : false;
	}

	////////////////////////////////////////////
	///// PIXEL CALCULATION
	////////////////////////////////////////////

	pixel_info_st bgPixelInfo;
	fg_pixel_info_st fgPixelInfo;
	pixel_info_st finalPixelInfo;

	if (_maskReg.showBg) {
		if (_scanlineCycle >= 0 && _scanlineCycle <= 256) {
			bgPixelInfo = getBgPixel();
		}
	}
	else {
		// https://wiki.nesdev.com/w/index.php/PPU_palettes
		if ((_vramAddr.raw & 0x3F00) == 0x3F00) {
			bgPixelInfo.pixel = 0x1F & _vramAddr.raw;
		}
	}

	if (_maskReg.showSpr && _scanlineCycle >= 0 && _scanlineCycle <= 256) {
		fgPixelInfo = getFgPixel();
	}

	finalPixelInfo = getPixel(bgPixelInfo, fgPixelInfo);

	sprScreen.SetPixel(_scanlineCycle - 1, _scanline, GetColourFromPaletteRam(finalPixelInfo.palette, finalPixelInfo.pixel));

	_scanlineCycle++;
	if (_scanlineCycle > 340)
	{
		_scanlineCycle = 0;
		_scanline++;
		if (_scanline >= 261)
		{
			_scanline = -1;
			_frameComplete = true;
		}
	}

	// Debug
	_debugPPUState.scanline = _scanline;
	_debugPPUState.scanlineCycle = _scanlineCycle;
	_debugPPUState.frameCounter = _frameCounter;
	_debugPPUState.maskReg = _maskReg;
	_debugPPUState.controlReg = _controlReg;
	_debugPPUState.statusReg = _statusReg;
	_debugPPUState.fineX = _fineX;
	_debugPPUState.vramAddr = _vramAddr;
	_debugPPUState.tmpVramAddr = _tmpVramAddr;

}

void PPU::secondaryOAMClear() {

#ifdef ACCURATE_PPU_SPRITE_RENDER_EMU

	switch (_scanlineCycle) {

	case 2: _secOamMem.raw[0] = 0xFF; break;
	case 4: _secOamMem.raw[1] = 0xFF; break;
	case 6: _secOamMem.raw[2] = 0xFF; break;
	case 8: _secOamMem.raw[3] = 0xFF; break;
	case 10: _secOamMem.raw[4] = 0xFF; break;

	case 12: _secOamMem.raw[5] = 0xFF; break;
	case 14: _secOamMem.raw[6] = 0xFF; break;
	case 16: _secOamMem.raw[7] = 0xFF; break;
	case 18: _secOamMem.raw[8] = 0xFF; break;
	case 20: _secOamMem.raw[9] = 0xFF; break;

	case 22: _secOamMem.raw[10] = 0xFF; break;
	case 24: _secOamMem.raw[11] = 0xFF; break;
	case 26: _secOamMem.raw[12] = 0xFF; break;
	case 28: _secOamMem.raw[13] = 0xFF; break;
	case 30: _secOamMem.raw[14] = 0xFF; break;

	case 32: _secOamMem.raw[15] = 0xFF; break;
	case 34: _secOamMem.raw[16] = 0xFF; break;
	case 36: _secOamMem.raw[17] = 0xFF; break;
	case 38: _secOamMem.raw[18] = 0xFF; break;
	case 40: _secOamMem.raw[19] = 0xFF; break;

	case 42: _secOamMem.raw[20] = 0xFF; break;
	case 44: _secOamMem.raw[21] = 0xFF; break;
	case 46: _secOamMem.raw[22] = 0xFF; break;
	case 48: _secOamMem.raw[23] = 0xFF; break;
	case 50: _secOamMem.raw[24] = 0xFF; break;

	case 52: _secOamMem.raw[25] = 0xFF; break;
	case 54: _secOamMem.raw[26] = 0xFF; break;
	case 56: _secOamMem.raw[27] = 0xFF; break;
	case 58: _secOamMem.raw[28] = 0xFF; break;
	case 60: _secOamMem.raw[29] = 0xFF; break;

	case 62: _secOamMem.raw[30] = 0xFF; break;
	case 64:

		_secOamMem.raw[31] = 0xFF;

		_foundSpritesCount = 0;

		_sprEvalState.state = SpriteEvalState::NORMAL_SEARCH;
		_sprEvalState.oamSpriteIdx = 0;
		_sprEvalState.secOamSpriteIdx = 0;
		_sprEvalState.sprite0Hit = false;
		_sprEvalState.readByte = 0x00;
		_sprEvalState.spriteByteIdx = 0;

		break;

	}

#else

	// Strengthen condition to limit to one execution
	if (_scanlineCycle == 64) {
		memset(_secOamMem.raw, 0xFF, 32);
	}

#endif

}

void PPU::spriteEvaluation() {

#ifdef ACCURATE_PPU_SPRITE_RENDER_EMU

	if (_scanlineCycle & 0x1) { // ODD: READ

		if (_sprEvalState.state == SpriteEvalState::NORMAL_SEARCH) {

			_sprEvalState.readByte = _oamMem.sprites[_sprEvalState.oamSpriteIdx].yPos;

			uint16_t spriteHeight = _controlReg.sprSize ? 16 : 8;
			bool inRange = (uint16_t)_scanline >= (uint16_t)_sprEvalState.readByte && (uint16_t)_scanline < ((uint16_t)_sprEvalState.readByte + spriteHeight);

			if (inRange) {
				_sprEvalState.state = SpriteEvalState::COPY;
				// Set sprite 0 hit
				if (_sprEvalState.oamSpriteIdx == 0 && !_sprEvalState.sprite0Hit)
					_sprEvalState.sprite0Hit = true;
			}
			else {
				// Increase pointer to OAM sprite
				_sprEvalState.oamSpriteIdx = (_sprEvalState.oamSpriteIdx + 1) % 64;
				if (_sprEvalState.oamSpriteIdx == 0) _sprEvalState.state = SpriteEvalState::FULL_OAM_READ;
			}

		}
		else if (_sprEvalState.state == SpriteEvalState::COPY) {

			_sprEvalState.readByte = _oamMem.raw[_sprEvalState.oamSpriteIdx * 4 + _sprEvalState.spriteByteIdx];

		}
		else if (_sprEvalState.state == SpriteEvalState::BUGGY_SEARCH) {

			_sprEvalState.readByte = _oamMem.raw[_sprEvalState.oamSpriteIdx * 4 + _sprEvalState.spriteByteIdx];

			uint16_t spriteHeight = _controlReg.sprSize ? 16 : 8;
			bool inRange = (uint16_t)_scanline >= (uint16_t)_sprEvalState.readByte && (uint16_t)_scanline < ((uint16_t)_sprEvalState.readByte + spriteHeight);

			if (inRange) {
				_sprEvalState.state = SpriteEvalState::BUGGY_COPY;
				// Set overflow bit
				_statusReg.sprOverflow = 1;
			}
			else {
				// Buggy behavior
				_sprEvalState.oamSpriteIdx = (_sprEvalState.oamSpriteIdx + 1) & 0x3F;
				_sprEvalState.spriteByteIdx = (_sprEvalState.spriteByteIdx + 1) & 0x3;
			}

		}

	}
	else { // EVEN: WRITE


		if (_sprEvalState.state == SpriteEvalState::COPY) {

			_secOamMem.raw[_sprEvalState.secOamSpriteIdx * 4 + _sprEvalState.spriteByteIdx]
				= _sprEvalState.readByte; // Write into sec OAM

			// Increase indexes

			// Increase pointer to sprite byte
			_sprEvalState.spriteByteIdx = (_sprEvalState.spriteByteIdx + 1) & 0x3;

			// If we've written the whole sprite to sec OAM...
			if (_sprEvalState.spriteByteIdx == 0) {

				// Increase pointer to OAM sprite
				_sprEvalState.oamSpriteIdx = (_sprEvalState.oamSpriteIdx + 1) & 0x3F;

				// Increase the pointer to secondary OAM sprite
				_sprEvalState.secOamSpriteIdx = (_sprEvalState.secOamSpriteIdx + 1) & 0x7;
				if (_foundSpritesCount < 8) _foundSpritesCount++;

				if (_sprEvalState.oamSpriteIdx == 0) {
					// 2a. If n has overflowed back to zero (all 64 sprites evaluated), go to 4
					_sprEvalState.state = SpriteEvalState::FULL_OAM_READ;
				}
				else if (_sprEvalState.secOamSpriteIdx == 0) {
					// 2c. If exactly 8 sprites have been found, disable writes to secondary OAM because it is full.
					// This causes sprites in back to drop out.
					_sprEvalState.state = SpriteEvalState::BUGGY_SEARCH;
				}
				else {
					// 2b. If less than 8 sprites have been found, go to 1
					_sprEvalState.state = SpriteEvalState::NORMAL_SEARCH;
				}

			}

		}
		else if (_sprEvalState.state == SpriteEvalState::BUGGY_COPY) {

			// Don't copy anything to sec OAM

			_sprEvalState.spriteByteIdx = (_sprEvalState.spriteByteIdx + 1) & 0x3;

			// If we've written the whole sprite to sec OAM...
			if (_sprEvalState.spriteByteIdx == 0) {

				// Increase pointer to OAM sprite
				_sprEvalState.oamSpriteIdx = (_sprEvalState.oamSpriteIdx + 1) & 0x3F;

				// "If n overflows to 0, go to 4; otherwise go to 3"
				if (_sprEvalState.oamSpriteIdx == 0) {
					_sprEvalState.state = SpriteEvalState::FULL_OAM_READ;
				}
				else {
					_sprEvalState.state = SpriteEvalState::BUGGY_SEARCH;
				}

			}
			else {
				// Do nothing
			}

		}

	}

#else

	// Strengthen condition to limit to one execution
	if (_scanlineCycle == 256) {

		_spriteZeroRenderedNextScanline = false;

		_foundSpritesCount = 0;

		int16_t spriteHeight = _controlReg.sprSize ? 16 : 8;

		for (uint16_t spriteIdx = 0; spriteIdx < 64; spriteIdx++) {

			int16_t yPos = _oamMem.sprites[spriteIdx].yPos;
			int16_t dist = _scanline - yPos;
			bool inRange = dist >= 0 && dist < spriteHeight;

			if (inRange) {

				if (_foundSpritesCount < 8) {

					_secOamMem.sprites[_foundSpritesCount] = _oamMem.sprites[spriteIdx];

					_foundSpritesCount++;

					if (!_spriteZeroRenderedNextScanline) {
						_spriteZeroRenderedNextScanline = spriteIdx == 0; // Sprite 0 will be rendered
					}

				}
				else {
					if (_foundSpritesCount == 8 && _maskReg.raw > 0) {
						_statusReg.sprOverflow = 1;
					}
					break;
				}

			}

		}

	}

#endif

}

void PPU::spriteFetch() {

#ifdef ACCURATE_PPU_SPRITE_RENDER_EMU

	uint16_t fetchCyle = _scanlineCycle - 257;

	// At sprite fetch cycle == 5, fetch LO byte;
	// at sprite fetch cycle == 7, fetch HI byte
	uint16_t spriteFetchCycle = fetchCyle & 0x7;

	uint16_t spriteId = fetchCyle / 8;

	if (spriteId < _foundSpritesCount && (spriteFetchCycle == 5 || spriteFetchCycle == 7)) {

		uint16_t planeOffset = spriteFetchCycle == 7 ? 8 : 0;

		uint16_t spriteByteAddr;

		int16_t scanline = _scanline;
		if (_scanline < 0) scanline = 261;

		int16_t spriteLine = (int16_t)scanline - (int16_t)_secOamMem.sprites[spriteId].yPos;
		int16_t tileLine;

		if (_controlReg.sprSize == 0) { // 8x8

			if (_secOamMem.sprites[spriteId].attr.verticalFlip) {
				spriteLine = 7 - spriteLine;
			}

			spriteByteAddr = (_controlReg.sprPatternTableAddrFor8x8Mode << 12)
				+ ((uint16_t)_secOamMem.sprites[spriteId].id << 4) // * 16 bytes
				+ (uint16_t)spriteLine;

		}
		else { // 8x16

			// A 8x16 sprite is composed of two tiles.
			// The distance between the Y pos of the LSB plane of any of the 2 tiles,
			// and the current scanline, will be a value in: [0, 7], [15, 23]
			bool isBottomTile = spriteLine > 7;

			// Distance from the Y pos of the tile being rendered, and the current scanline
			tileLine = spriteLine & 0x7;
			if (_secOamMem.sprites[spriteId].attr.verticalFlip) {
				tileLine = 7 - tileLine;
			}

			spriteByteAddr = ((_secOamMem.sprites[spriteId].id & 0x1) << 12)
				+ (((uint16_t)_secOamMem.sprites[spriteId].id & 0xFE) << 4) // * 16 bytes
				+ (uint16_t)tileLine;

			if (isBottomTile) spriteByteAddr += 16;

		}

		uint8_t spritePatternBits = ppuRead(spriteByteAddr + planeOffset);

		if (_secOamMem.sprites[spriteId].attr.horizontalFlip) {
			REVERSE(spritePatternBits);
		}

		if (spriteFetchCycle == 5) {
			_scanlineSpritesBuffer_pixelLsb[spriteId] = spritePatternBits;
		}
		else {
			_scanlineSpritesBuffer_pixelMsb[spriteId] = spritePatternBits;
			// Copy just once
			_scanlineSpritesBuffer_attribute[spriteId] = _secOamMem.sprites[spriteId].attr;
			_scanlineSpritesBuffer_xPos[spriteId] = _secOamMem.sprites[spriteId].xPos;
		}

	}

	if (_scanlineCycle == 320) {
		// Set count of sprites for the next scanline
		_scanlineSpritesCnt = _foundSpritesCount;
		_spriteZeroRenderedNextScanline = _sprEvalState.sprite0Hit;
	}

#else

	// Strengthen condition to limit to one execution
	if (_scanlineCycle == 320) {

		memset(_scanlineSpritesBuffer_pixelLsb, 0x0, 8);
		memset(_scanlineSpritesBuffer_pixelMsb, 0x0, 8);
		memset(_scanlineSpritesBuffer_attribute, 0x0, 8);
		memset(_scanlineSpritesBuffer_xPos, 0x0, 8);

		uint16_t spriteByteAddr;

		for (uint16_t spriteIdx = 0; spriteIdx < _foundSpritesCount; spriteIdx++) {

			int16_t scanline = _scanline;
			if (_scanline < 0) scanline = 261;

			int16_t spriteLine = (int16_t)scanline - (int16_t)_secOamMem.sprites[spriteIdx].yPos;
			int16_t tileLine;

			if (_controlReg.sprSize == 0) { // 8x8

				if (_secOamMem.sprites[spriteIdx].attr.verticalFlip) {
					spriteLine = 7 - spriteLine;
				}

				spriteByteAddr = (_controlReg.sprPatternTableAddrFor8x8Mode << 12)
					+ ((uint16_t)_secOamMem.sprites[spriteIdx].id << 4) // * 16 bytes
					+ (uint16_t)spriteLine;

			}
			else { // 8x16

				// A 8x16 sprite is composed of two tiles.
				// The distance between the Y pos of the LSB plane of any of the 2 tiles,
				// and the current scanline, will be a value in: [0, 7], [15, 23]
				bool isBottomTile = spriteLine > 7;

				// Distance from the Y pos of the tile being rendered, and the current scanline
				tileLine = spriteLine & 0x7;
				if (_secOamMem.sprites[spriteIdx].attr.verticalFlip) {
					tileLine = 7 - tileLine;
				}

				spriteByteAddr = ((_secOamMem.sprites[spriteIdx].id & 0x1) << 12)
					+ (((uint16_t)_secOamMem.sprites[spriteIdx].id & 0xFE) << 4) // * 16 bytes
					+ (uint16_t)tileLine;


				if (isBottomTile) spriteByteAddr += 16;

			}

			uint8_t spritePatternBitsLo = ppuRead(spriteByteAddr);
			uint8_t spritePatternBitsHi = ppuRead(spriteByteAddr + 8);

			if (_secOamMem.sprites[spriteIdx].attr.horizontalFlip) {
				REVERSE(spritePatternBitsLo);
				REVERSE(spritePatternBitsHi);
			}

			_scanlineSpritesBuffer_pixelLsb[spriteIdx] = spritePatternBitsLo;
			_scanlineSpritesBuffer_pixelMsb[spriteIdx] = spritePatternBitsHi;
			_scanlineSpritesBuffer_attribute[spriteIdx] = _secOamMem.sprites[spriteIdx].attr;
			_scanlineSpritesBuffer_xPos[spriteIdx] = _secOamMem.sprites[spriteIdx].xPos;

		}

		// Set count of sprites for the next scanline
		_scanlineSpritesCnt = _foundSpritesCount;

	}

#endif

}

fg_pixel_info_st PPU::getFgPixel() {

	fg_pixel_info_st info;
	uint8_t pixel = 0x00;
	uint8_t palette = 0x00;
	uint8_t priority = 0x00;

	for (uint16_t spriteIdx = 0; spriteIdx < _scanlineSpritesCnt; spriteIdx++) {

		if (_scanlineSpritesBuffer_xPos[spriteIdx] == 0) {

			uint8_t fg_pixel_lo = (_scanlineSpritesBuffer_pixelLsb[spriteIdx] & 0x80) > 0;
			uint8_t fg_pixel_hi = (_scanlineSpritesBuffer_pixelMsb[spriteIdx] & 0x80) > 0;

			pixel = (fg_pixel_hi << 1) | fg_pixel_lo;
			palette = _scanlineSpritesBuffer_attribute[spriteIdx].palette + 4; // Palettes 0-3: bg, palettes 4-7: sprites
			priority = _scanlineSpritesBuffer_attribute[spriteIdx].priority;

			if (pixel != 0) {
				if (spriteIdx == 0) {
					info.isSprite0 = true;
				}
				break;
			}

		}

	}

	info.pixel = pixel;
	info.palette = palette;
	info.priority = priority;

	return info;

}

pixel_info_st PPU::getPixel(pixel_info_st& bgPixelInfo, fg_pixel_info_st& fgPixelInfo) {

	pixel_info_st info;

	if (bgPixelInfo.pixel > 0 && fgPixelInfo.pixel == 0) {

		info.pixel = bgPixelInfo.pixel;
		info.palette = bgPixelInfo.palette;

	}
	else if (bgPixelInfo.pixel == 0 && fgPixelInfo.pixel > 0) {

		info.pixel = fgPixelInfo.pixel;
		info.palette = fgPixelInfo.palette;

	}
	else if (bgPixelInfo.pixel > 0 && fgPixelInfo.pixel > 0) {

		if (!fgPixelInfo.priority) {
			info.pixel = fgPixelInfo.pixel;
			info.palette = fgPixelInfo.palette;
		}
		else {
			info.pixel = bgPixelInfo.pixel;
			info.palette = bgPixelInfo.palette;
		}

		// Sprite 0 hit does not happen:
		// At x=0 to x=7 if the left-side clipping window is enabled (if bit 2 or bit 1 of PPUMASK is 0).
		bool cond1 = (_maskReg.showSprLeft == 0 || _maskReg.showBgLeft == 0) && _scanlineCycle < 8;
		// At x=255, for an obscure reason related to the pixel pipeline.
		bool cond2 = _scanlineCycle == 255;

		if (_maskReg.showBg
			&& !_spriteZeroRenderedThisFrame
			&& fgPixelInfo.isSprite0
			&& _spriteZeroRenderedNextScanline
			&& !cond1
			&& !cond2)
		{
			_statusReg.sprZeroHit = 1;
			_spriteZeroRenderedThisFrame = true;
			_spriteZeroRenderedNextScanline = false;
		}

	}

	return info;

}

pixel_info_st PPU::getBgPixel() {

	pixel_info_st info;

	uint16_t pixel_selector = 0x8000 >> _fineX;

	uint8_t bgPixelLsb = (_bg16pxColorIdLsbPipe & pixel_selector) > 0;
	uint8_t bgPixelMsb = (_bg16pxColorIdMsbPipe & pixel_selector) > 0;
	info.pixel = (bgPixelMsb << 1) | bgPixelLsb;

	uint8_t bgPaletteLsb = (_bg16pxPaletteIdLsbPipe & pixel_selector) > 0;
	uint8_t bgPaletteMsb = (_bg16pxPaletteIdMsbPipe & pixel_selector) > 0;
	info.palette = (bgPaletteMsb << 1) | bgPaletteLsb;

	return info;

}

olc::Pixel& PPU::GetColourFromPaletteRam(uint8_t palette, uint8_t pixel) {
	return palScreen[ppuRead(0x3F00 + (palette << 2) + pixel) & 0x3F];
}

olc::Sprite& PPU::GetScreen() {
	return sprScreen;
}

/* DEBUG */

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
