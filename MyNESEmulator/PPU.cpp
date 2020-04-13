#include "PPU.h"
#include "NESConstants.h"

#define _IS_PATTERN_TABLE_ADDR(addr) (addr >= PPU_ADDR_SPACE_PATTERN_TABLE_0_START && addr <= PPU_ADDR_SPACE_PATTERN_TABLE_1_END)
#define _IS_PALETTE_ADDR(addr) (addr >= PPU_ADDR_SPACE_PALETTES_REGION_START && addr <= PPU_ADDR_SPACE_PALETTES_REGION_END)

#define _IS_NAMETABLE_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_3_END)

#define _IS_NAMETABLE_0_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_0_END)
#define _IS_NAMETABLE_1_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_1_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_1_END)
#define _IS_NAMETABLE_2_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_2_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_2_END)
#define _IS_NAMETABLE_3_ADDR(addr) (addr >= PPU_ADDR_SPACE_NAME_TABLE_3_START && addr <= PPU_ADDR_SPACE_NAME_TABLE_3_END)


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

	ppu_addr_scroll_latch = PPU_HI_ADDR_WR_STATE;
	ppu_data_buffer = 0x00;

	scanline = 0;
	scanline_dot = 0;
	tile_pixel = 0; // No need to initialize, though...

	bg_tile_id_nxt = 0x00;
	bg_tile_attr_byte_nxt = 0x00;

	bg_nxt_8px_palette_id = 0x00;
	bg_nxt_8px_color_id_lsb = 0x00;
	bg_nxt_8px_color_id_msb = 0x00;

	bg_16px_pipe_palette_id_lsb = 0x00;
	bg_16px_pipe_palette_id_msb = 0x00;
	bg_16px_pipe_color_id_lsb = 0x00;
	bg_16px_pipe_color_id_msb = 0x00;

	vram_addr.raw = 0x0000;
	tmp_vram_addr.raw = 0x0000;
	fine_x = 0;

	control_reg.raw = 0x00;
	mask_reg.raw = 0x00;
	status_reg.raw = 0x00;

	odd_frame_switch = true;

}

void PPU::connectCartridge(const std::shared_ptr<Cartridge>& cartridge)
{
	this->cartridge = cartridge;
	this->isNTSC = cartridge->cartridgeHeader.tvSystem1 == 0;
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
			data = (status_reg.raw & 0b11100000) | (ppu_data_buffer & 0b00011111); // Return noise in lower 5 bits
			_LOG("cpuRead()/STATUS_REG: read 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
			// Clear bit 7
			status_reg.vertical_blank = 0;
			// Transition to initial state
			ppu_addr_scroll_latch = PPU_HI_ADDR_WR_STATE;
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
			data = ppu_data_buffer;
			// Read
			ppu_data_buffer = this->ppuRead(vram_addr.raw, false);
			// If reading from the palettes, do not delay
			if (_IS_PALETTE_ADDR(vram_addr.raw)) {
				data = ppu_data_buffer;
			}
			_LOG("cpuRead()/VRAM_DATA: read 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
			// Auto-increment nametable address (loopy address)
			if (control_reg.vram_addr_increment_per_cpu_rw) {
				// Go to the next row
				vram_addr.raw += PPU_NAME_TABLE_COLS_PER_ROW;
			}
			else {
				// Go to the next row
				vram_addr.raw += 1;
			}
			//vram_addr.raw &= 0x3FFF;
			_LOG("cpuRead():new vram_addr: " << std::hex << (uint32_t)vram_addr.raw << std::endl);
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
		control_reg.raw = data;
		//
		tmp_vram_addr.nametable_x = control_reg.base_nametable_addr & 0x1;
		tmp_vram_addr.nametable_y = (control_reg.base_nametable_addr >> 1) & 0x1;
		break;
	case CPU_ADDR_SPACE_PPU_PPU_MASK:
		_LOG("cpuWrite()/mask_reg: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		mask_reg.raw = data;
		break;
	case CPU_ADDR_SPACE_PPU_STATUS_REG:
		break;
	case CPU_ADDR_SPACE_PPU_SPRITE_MEM_ADDR: // OAM
		oam_addr = data;
		break;
	case CPU_ADDR_SPACE_PPU_SPRITE_MEM_DATA: // OAM
		oam_data = data;
		break;
	case CPU_ADDR_SPACE_PPU_BG_SCROLL: // Set top-left corner of the screen

		_LOG("cpuWrite()/scroll: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		if (ppu_addr_scroll_latch == PPU_HI_ADDR_WR_STATE) {
			// Transition to next state
			ppu_addr_scroll_latch = PPU_LO_ADDR_WR_STATE;

			fine_x = (uint16_t)data & 0x7;
			tmp_vram_addr.coarse_x = ((uint16_t)data >> 3) & 0b00011111; // 0x1F
		} else {
			// Transition to initial state
			ppu_addr_scroll_latch = PPU_HI_ADDR_WR_STATE;

			tmp_vram_addr.fine_y = (uint16_t)data & 0x7;
			tmp_vram_addr.coarse_y = ((uint16_t)data >> 3) & 0b00011111; // 0x1F
		}
		break;
	case CPU_ADDR_SPACE_PPU_VRAM_ADDR: // Access the PPU's RAM via this address
		if (ppu_addr_scroll_latch == PPU_HI_ADDR_WR_STATE) {

			// Transition to next state
			ppu_addr_scroll_latch = PPU_LO_ADDR_WR_STATE;
			// Set hi part
			tmp_vram_addr.raw &= 0x00FF;
			tmp_vram_addr.raw |= ((uint16_t)data & 0x003F) << 8;
			_LOG("cpuWrite():PPU_VRAM_ADDR HI:new tmp_vram_addr.raw: " << std::hex << (uint32_t)tmp_vram_addr.raw << std::endl);
		} else {

			// Go to initial state
			ppu_addr_scroll_latch = PPU_HI_ADDR_WR_STATE;
			// Set lo part
			tmp_vram_addr.raw &= 0xFF00;
			tmp_vram_addr.raw |= data;
			// Set active address
			vram_addr.raw = tmp_vram_addr.raw;
			_LOG("cpuWrite():PPU_VRAM_ADDR LO:new vram_addr.raw: " << std::hex << (uint32_t)vram_addr.raw << std::endl);
		}
		break;
	case CPU_ADDR_SPACE_PPU_VRAM_DATA: // NAMETABLE
		// Write
		this->ppuWrite(vram_addr.raw, data);
		
		if (control_reg.vram_addr_increment_per_cpu_rw) {
			// Go to the next row
			vram_addr.raw += PPU_NAME_TABLE_COLS_PER_ROW;
		} else {
			// Go to the next column
			vram_addr.raw += 1;
		}
		//vram_addr.raw &= 0x3FFF;
		_LOG("cpuWrite():PPU_VRAM_DATA:new vram_addr.raw: " << std::hex << (uint32_t)vram_addr.raw << std::endl);
		break;
	}
}

uint8_t PPU::ppuRead(uint16_t addr, bool readOnly)
{
	uint8_t readData = 0;
	addr &= PPU_ADDR_SPACE_MASK;

	if (this->cartridge->ppuRead(addr, readData)) {

	}
	else if (_IS_PATTERN_TABLE_ADDR(addr)) {
		uint16_t patternTableId = (addr >> 12) & 0x1;
		readData = this->patternTables[patternTableId][addr & PPU_PATTERN_TABLE_MASK];
	}
	else if (addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= (PPU_ADDR_SPACE_PALETTES_REGION_START - 1)) {
	//else if (_IS_NAMETABLE_ADDR(addr)) {
		
		addr = PPU_ADDR_SPACE_NAME_TABLE_0_START + (addr & 0xFFF);

		if (this->cartridge->vertical) { // Vertical mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				readData = this->nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				readData = this->nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				readData = this->nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else {
				readData = this->nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}
		}
		else { // Horizontal mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				readData = this->nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				readData = this->nameTables[0][addr & PPU_NAME_TABLE_MASK];
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				readData = this->nameTables[1][addr & PPU_NAME_TABLE_MASK];
			}
			else {
				readData = this->nameTables[1][addr & PPU_NAME_TABLE_MASK];
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

		//_LOG("ppuRead()/palette: read 0x" << std::hex << (uint16_t)palette_mem[aux_addr] << " @ 0x" << std::hex << addr << " / 0x" << std::hex << aux_addr << std::endl);
		readData = palette_mem[aux_addr];// &(mask_reg.grayscale ? 0x30 : 0x3F);
		
	}
	else {
		std::cout << "rd some addr: 0x" << std::hex << addr;
	}

	return readData;
}

void PPU::ppuWrite(uint16_t addr, uint8_t data)
{
	if (data == 0x2f)
		_LOG2("ppuWrite()/RAW: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
	addr &= PPU_ADDR_SPACE_MASK;

	if (this->cartridge->ppuWrite(addr & PPU_ADDR_SPACE_MASK, data)) {

	}
	else if (_IS_PATTERN_TABLE_ADDR(addr)) {
		uint16_t patternTableId = (addr >> 12) & 0x1;
		this->patternTables[patternTableId][addr & PPU_PATTERN_TABLE_MASK];
	}
	else if ( addr >= PPU_ADDR_SPACE_NAME_TABLE_0_START && addr <= (PPU_ADDR_SPACE_PALETTES_REGION_START - 1) ) {
	//else if (_IS_NAMETABLE_ADDR(addr)) {

		addr = PPU_ADDR_SPACE_NAME_TABLE_0_START + (addr & 0xFFF);

		_LOG("ppuWrite()/nametable: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << std::endl);
		if (this->cartridge->vertical) { // Vertical mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				this->nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				this->nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				this->nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else {
				this->nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
		}
		else { // Horizontal mirroring
			if (_IS_NAMETABLE_0_ADDR(addr)) {
				this->nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_1_ADDR(addr)) {
				this->nameTables[0][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else if (_IS_NAMETABLE_2_ADDR(addr)) {
				this->nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
			}
			else {
				this->nameTables[1][addr & PPU_NAME_TABLE_MASK] = data;
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
		
		palette_mem[aux_addr] = data;
		_LOG("ppuWrite()/palette: write 0x" << std::hex << (uint16_t)data << " @ 0x" << std::hex << addr << " / 0x" << std::hex << aux_addr << ", NEW VALUE: " << std::hex << (uint16_t)palette_mem[aux_addr] << std::endl);

	}
	else {
		std::cout << "wr some addr: 0x" << std::hex << addr;
	}
}


void PPU::advanceClock() {

	// Is it a drawable scanline?
	if (scanline >= -1 && scanline <= 239) {

		// Pre-render line (-1)
		if (scanline == -1 && scanline_dot == 1) {
			// Beginning of frame... Set vertical blank to 0.
			status_reg.vertical_blank = 0;
		}

		if ((scanline_dot >= 0 && scanline_dot <= 256) || (scanline_dot >= 321 && scanline_dot <= 340)) {

			if (SHOW_BG()) {
				MOVE_PIPES();
			}

			tile_pixel = (scanline_dot - 1) % 8;

			// Extract background tile id from the name table region
			if (tile_pixel == 0) {

				// First off, push calculated values to pipe
				PUSH_PALETTE_ID_LSBS_TO_PIPE(bg_nxt_8px_palette_id);
				PUSH_PALETTE_ID_MSBS_TO_PIPE(bg_nxt_8px_palette_id);
				PUSH_COLOR_ID_LSBS_TO_PIPE(bg_nxt_8px_color_id_lsb);
				PUSH_COLOR_ID_MSBS_TO_PIPE(bg_nxt_8px_color_id_msb);

				// Get the id of the next tile
				uint16_t bg_tile_id_addr =
					PPU_ADDR_SPACE_NAME_TABLE_REGION_START + (vram_addr.raw & PPU_NAME_TABLE_REGION_MASK);
				bg_tile_id_nxt = ppuRead(bg_tile_id_addr); // Is a number in [0, 255]
				_LOG("\n--> FC: " << std::dec << frameCounter << ",  scanline: " << std::dec << scanline << ", scanline_dot: " << std::dec << (uint32_t)scanline_dot << "*" << std::endl);
				_LOG("bg_tile_id_addr: " << std::hex << (uint32_t)bg_tile_id_addr << std::endl);
				_LOG("bg_tile_id_nxt: " << std::dec << (uint32_t)bg_tile_id_nxt << std::endl);

			}

			// Get attribute table info for the current tile
			if (tile_pixel == 2) {

				// A supertile is 2x2 metatiles, and a metatile is 2x2 tiles
				// One attribute table byte contains the palette ids of 4 metatiles, or 1 supertile
				uint16_t supertile_x = vram_addr.coarse_x >> 2; // coarse_x indicates a tile
				uint16_t supertile_y = vram_addr.coarse_y >> 2;
				// This id identifies a supertile with a number in [0, 63]
				uint16_t supertile_id = (supertile_y << 3) | supertile_x;
				// Get the attribute table byte defining the (4) palette ids for this supertile
				uint16_t supertile_attr_byte_addr = PPU_ADDR_SPACE_NAME_TABLE_REGION_START
												+ (vram_addr.raw & 0x0C00) // Identifies the nametable
												+ (PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET + supertile_id); // Identifies a byte in a nametable
				_LOG("vram_addr.coarse_x: " << std::dec << (uint32_t)vram_addr.coarse_x << std::endl);
				_LOG("vram_addr.coarse_y: " << std::dec << (uint32_t)vram_addr.coarse_y << std::endl);
				_LOG("supertile_x: " << std::dec << (uint32_t)supertile_x << std::endl);
				_LOG("supertile_y: " << std::dec << (uint32_t)supertile_y << std::endl);
				_LOG("supertile id: " << std::dec << (uint32_t)supertile_id << std::endl);
				_LOG("PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET: " << std::hex << (uint32_t)PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET << std::endl);
				_LOG("sum: " << std::hex << (uint32_t)(PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET + supertile_id) << std::endl);
				_LOG("(vram_addr.raw): " << std::hex << (uint32_t)(vram_addr.raw) << std::endl);
				_LOG("supertile_attr_byte_addr: 0x" << std::hex << (uint32_t)supertile_attr_byte_addr << std::endl);
				bg_nxt_8px_palette_id = ppuRead(supertile_attr_byte_addr);
				_LOG("bg_nxt_8px_palette_id: 0x" << std::hex << (uint32_t)bg_nxt_8px_palette_id << std::endl);

				// Now, get palette id for the 8 pixels which are currently being calculated
				// This is a function of which metatile said 8 pixels are a part of
				if (vram_addr.coarse_x & 0b10) bg_nxt_8px_palette_id >>= PPU_PALETTE_ID_BIT_LEN;
				if (vram_addr.coarse_y & 0b10) bg_nxt_8px_palette_id >>= (PPU_PALETTE_ID_BIT_LEN * 2);
				bg_nxt_8px_palette_id &= PPU_PALETTE_ID_MASK;

			}

			// Extract lsb's of the palette color ids of the tile's 8 pixels
			if (tile_pixel == 4) {
				uint16_t tile_lsb_addr =
					PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START + // Offset is 0... Useless calculation!
					(control_reg.bg_pattern_table_addr << 12) + // Select pattern table by multiplying by 4 KiB
					(bg_tile_id_nxt << 4) + // Tile id * 16
					vram_addr.fine_y; // Select which 8 pixels
				bg_nxt_8px_color_id_lsb = ppuRead(tile_lsb_addr);

			}

			// Extract msb's of the palette color ids of the tile's 8 pixels
			if (tile_pixel == 6) {
				uint16_t tile_msb_addr =
					PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START + // Offset is 0... Useless calculation!
					(control_reg.bg_pattern_table_addr << 12) + // Select pattern table by multiplying by 4 KiB
					(bg_tile_id_nxt << 4) + // Tile id * 16
					vram_addr.fine_y + // Select which 8 pixels
					8; // MSB plane
				bg_nxt_8px_color_id_msb = ppuRead(tile_msb_addr);
			}

			if (tile_pixel == 7) {

				if (SHOW_BG() || mask_reg.show_spr) {
					INCREMENT_X();
					_LOG("increment X: vram_addr.coarse_x: " << std::dec << (uint32_t)vram_addr.coarse_x << std::endl);
				}

			}

		}

		// Inc. vert(v)
		if (scanline_dot == 256) {
			if (SHOW_BG() || mask_reg.show_spr) {
				INCREMENT_Y();
				_LOG("increment Y: vram_addr.coarse_y: " << std::dec << (uint32_t)vram_addr.coarse_y << std::endl);
			}
		}

		//
		if (scanline_dot == 257) {

			// This scanline_pixel is actually equivalent to a tile_pixel 0
			PUSH_COLOR_ID_LSBS_TO_PIPE(bg_nxt_8px_color_id_lsb);
			PUSH_COLOR_ID_MSBS_TO_PIPE(bg_nxt_8px_color_id_msb);
			PUSH_PALETTE_ID_LSBS_TO_PIPE(bg_nxt_8px_palette_id);
			PUSH_PALETTE_ID_MSBS_TO_PIPE(bg_nxt_8px_palette_id);

			if (SHOW_BG() || mask_reg.show_spr) {
				TRANSFER_ADDR_X();
			}
			
		}

		if (scanline_dot == 337 || scanline_dot == 339) {
			// Useless read... Why keep it?
			uint16_t bg_tile_id_addr =
				PPU_ADDR_SPACE_NAME_TABLE_REGION_START + (vram_addr.raw & PPU_NAME_TABLE_REGION_MASK);
			bg_tile_id_nxt = ppuRead(bg_tile_id_addr); // Is a number in [0, 255]

			// For odd frames, the cycle at the end of the scanline is skipped (this is done internally by jumping directly from (339,261) to (0,0)
			if (IS_PRERENDER_SCANLINE() && SHOW_BG() && isNTSC && odd_frame_switch && scanline_dot == 339) {
				scanline_dot = 340;
			}

		}

		if (IS_PRERENDER_SCANLINE()) {

			if (scanline_dot >= 280 && scanline_dot <= 304) {
				if (SHOW_BG() || mask_reg.show_spr)
					TRANSFER_ADDR_Y();
			}

		}
		
	}

	// Strictly debug... frameCounter has no emulation function.
	if (scanline == 240 && scanline_dot == 0) { // This is the post-render scanline
		frameCounter++;
		std::cout << "new frame: " << frameCounter << std::endl;
		odd_frame_switch = !odd_frame_switch; // Reverse
	}

	if (scanline == 241 && scanline_dot == 1) {
		status_reg.vertical_blank = 1;
		if (control_reg.nmi_at_v_blank_interval_start)
			nmi = true;
		else
			nmi = false;
	}

	/************ Compose image *************/
	uint8_t bg_pixel = 0x00;
	uint8_t bg_palette = 0x00;

	if (SHOW_BG()) {

		uint16_t pixel_selector = 0x8000 >> fine_x;

		uint8_t bg_pixel_lsb = (bg_16px_pipe_color_id_lsb & pixel_selector) > 0;
		uint8_t bg_pixel_msb = (bg_16px_pipe_color_id_msb & pixel_selector) > 0;
		bg_pixel = (bg_pixel_msb << 1) | bg_pixel_lsb;

		uint8_t bg_palette_lsb = (bg_16px_pipe_palette_id_lsb & pixel_selector) > 0;
		uint8_t bg_palette_msb = (bg_16px_pipe_palette_id_msb & pixel_selector) > 0;
		bg_palette = (bg_palette_msb << 1) | bg_palette_lsb;

	}

	sprScreen.SetPixel(scanline_dot - 1, scanline, GetColourFromPaletteRam(bg_palette, bg_pixel));

	scanline_dot++;
	if (scanline_dot > 340)
	{
		scanline_dot = 0;
		scanline++;
		if (scanline >= 261)
		{
			scanline = -1;
			frame_complete = true;
		}
	}

	// Debug
	//std::cout << "   saved dot: " << scanline_dot << std::endl;
	debugPPUState.scanline = scanline;
	debugPPUState.scanline_dot = scanline_dot;
	debugPPUState.frame_counter = frameCounter;
	debugPPUState.mask_reg = mask_reg;
	debugPPUState.control_reg = control_reg;
	debugPPUState.status_reg = status_reg;
	debugPPUState.fine_x = fine_x;
	debugPPUState.vram_addr = vram_addr;
	debugPPUState.tmp_vram_addr = tmp_vram_addr;

	if (frameCounter > 5 && scanline == 0 && scanline_dot == 0) {
		printPPURamRange(0x2000, 0x2FFF);
		printPPURamRange(PPU_ADDR_SPACE_PALETTES_REGION_START, PPU_ADDR_SPACE_PALETTES_REGION_END);
	}

	//_LOG("scanline: " << std::dec << scanline << ", scanline_dot: " << std::dec << scanline_dot << std::endl);
	//std::cout << "scanline: " << std::dec << scanline << ", scanline_dot: " << std::dec << scanline_dot << std::endl;

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

olc::Sprite& PPU::GetNameTable(uint8_t i)
{
	return sprNameTable[i];
}

olc::Sprite& PPU::GetPatternTable(uint8_t i, uint8_t palette)
{
	// This function draw the CHR ROM for a given pattern table into
	// an olc::Sprite, using a specified palette. Pattern tables consist
	// of 16x16 "tiles or characters". It is independent of the running
	// emulation and using it does not change the systems state, though
	// it gets all the data it needs from the live system. Consequently,
	// if the game has not yet established palettes or mapped to relevant
	// CHR ROM banks, the sprite may look empty. This approach permits a 
	// "live" extraction of the pattern table exactly how the NES, and 
	// ultimately the player would see it.

	// A tile consists of 8x8 pixels. On the NES, pixels are 2 bits, which
	// gives an index into 4 different colours of a specific palette. There
	// are 8 palettes to choose from. Colour "0" in each palette is effectively
	// considered transparent, as those locations in memory "mirror" the global
	// background colour being used. This mechanics of this are shown in 
	// detail in ppuRead() & ppuWrite()

	// Characters on NES
	// ~~~~~~~~~~~~~~~~~
	// The NES stores characters using 2-bit pixels. These are not stored sequentially
	// but in singular bit planes. For example:
	//
 	// 2-Bit Pixels       LSB Bit Plane     MSB Bit Plane
	// 0 0 0 0 0 0 0 0	  0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0
	// 0 1 1 0 0 1 1 0	  0 1 1 0 0 1 1 0   0 0 0 0 0 0 0 0
	// 0 1 2 0 0 2 1 0	  0 1 1 0 0 1 1 0   0 0 1 0 0 1 0 0
	// 0 0 0 0 0 0 0 0 =  0 0 0 0 0 0 0 0 + 0 0 0 0 0 0 0 0
	// 0 1 1 0 0 1 1 0	  0 1 1 0 0 1 1 0   0 0 0 0 0 0 0 0
	// 0 0 1 1 1 1 0 0	  0 0 1 1 1 1 0 0   0 0 0 0 0 0 0 0
	// 0 0 0 2 2 0 0 0	  0 0 0 1 1 0 0 0   0 0 0 1 1 0 0 0
	// 0 0 0 0 0 0 0 0	  0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0
	//
	// The planes are stored as 8 bytes of LSB, followed by 8 bytes of MSB

	// Loop through all 16x16 tiles
	for (uint16_t nTileY = 0; nTileY < 16; nTileY++)
	{
		for (uint16_t nTileX = 0; nTileX < 16; nTileX++)
		{
			// Convert the 2D tile coordinate into a 1D offset into the pattern
			// table memory.
			uint16_t nOffset = nTileY * 256 + nTileX * 16;

			// Now loop through 8 rows of 8 pixels
			for (uint16_t row = 0; row < 8; row++)
			{
				// For each row, we need to read both bit planes of the character
				// in order to extract the least significant and most significant 
				// bits of the 2 bit pixel value. in the CHR ROM, each character
				// is stored as 64 bits of lsb, followed by 64 bits of msb. This
				// conveniently means that two corresponding rows are always 8
				// bytes apart in memory.
				uint8_t tile_lsb = ppuRead(i * 0x1000 + nOffset + row + 0x0000);
				uint8_t tile_msb = ppuRead(i * 0x1000 + nOffset + row + 0x0008);


				// Now we have a single row of the two bit planes for the character
				// we need to iterate through the 8-bit words, combining them to give
				// us the final pixel index
				for (uint16_t col = 0; col < 8; col++)
				{
					// We can get the index value by simply adding the bits together
					// but we're only interested in the lsb of the row words because...
					uint8_t pixel = (tile_lsb & 0x01) + (tile_msb & 0x01);

					// ...we will shift the row words 1 bit right for each column of
					// the character.
					tile_lsb >>= 1; tile_msb >>= 1;

					// Now we know the location and NES pixel value for a specific location
					// in the pattern table, we can translate that to a screen colour, and an
					// (x,y) location in the sprite
					sprPatternTable[i].SetPixel
					(
						nTileX * 8 + (7 - col), // Because we are using the lsb of the row word first
												// we are effectively reading the row from right
												// to left, so we need to draw the row "backwards"
						nTileY * 8 + row, 
						GetColourFromPaletteRam(palette, pixel)
					);
				}
			}
		}
	}

	// Finally return the updated sprite representing the pattern table
	return sprPatternTable[i];
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
	return this->debugPPUState;
}
