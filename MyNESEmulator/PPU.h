#pragma once

#include <cstdint>
#include "Cartridge.h"
#include <iostream>

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include "olcPixelGameEngine.h"

union ppuctrl_un {
	struct {
		uint8_t base_nametable_addr : 2; // (0 = $2000; 1 = $2400; 2 = $2800; 3 = $2C00)
		uint8_t vram_addr_increment_per_cpu_rw : 1; // (0: add 1, going across; 1: add 32, going down)
		uint8_t spr_pattern_table_addr_for_8x8_mode : 1; // (0: $0000; 1: $1000; ignored in 8x16 mode)
		uint8_t bg_pattern_table_addr : 1; // (0: $0000; 1: $1000)
		uint8_t spr_size : 1; // (0: 8x8 pixels; 1: 8x16 pixels)
		uint8_t ppu_master_slave_select : 1; // (0: read backdrop from EXT pins; 1: output color on EXT pins)
		uint8_t nmi_at_v_blank_interval_start : 1; // (0: off; 1: on)
	};
	uint8_t raw;
};

union ppumask_un {
	struct {
		uint8_t grayscale : 1; // (0: normal color, 1: produce a greyscale display)
		uint8_t show_bg_left : 1; // 0: hide
		uint8_t show_spr_left : 1; // 0: hide
		uint8_t show_bg : 1;
		uint8_t show_spr : 1;
		uint8_t emphasize_red : 1;
		uint8_t emphasize_green : 1;
		uint8_t emphasize_blue : 1;
	};
	uint8_t raw;
};

union ppustatus_un {
	struct {
		uint8_t unused : 5;
		uint8_t spr_overflow : 1;
		uint8_t spr_zero_hit : 1;
		uint8_t vertical_blank : 1;
	};
	uint8_t raw;
};

union loopy_un {
	struct {
		uint16_t coarse_x : 5;
		uint16_t coarse_y : 5;
		uint16_t nametable_x : 1;
		uint16_t nametable_y : 1;
		uint16_t fine_y : 3;
		uint16_t unused : 1;
	};
	uint16_t raw = 0x0000;
};

struct debug_ppu_state_dsc_st {

	int16_t scanline;
	int16_t scanline_dot;
	uint64_t frame_counter;

	ppuctrl_un control_reg;
	ppumask_un mask_reg;
	ppustatus_un status_reg;

	loopy_un tmp_vram_addr;
	loopy_un vram_addr;
	uint8_t fine_x;

};

#define SHOW_BG() (mask_reg.show_bg)

#define IS_PRERENDER_SCANLINE() (scanline == -1)
#define IS_DRAWABLE_SCANLINE() (scanline >= -1 && scanline <= 239)

#define TRANSFER_ADDR_X() \
	vram_addr.coarse_x = tmp_vram_addr.coarse_x; \
	vram_addr.nametable_x = tmp_vram_addr.nametable_x

#define TRANSFER_ADDR_Y() \
	vram_addr.fine_y = tmp_vram_addr.fine_y; \
	vram_addr.coarse_y = tmp_vram_addr.coarse_y; \
	vram_addr.nametable_y = tmp_vram_addr.nametable_y

#define MOVE_PIPES() \
	bg_16px_pipe_palette_id_lsb <<= 1; \
	bg_16px_pipe_palette_id_msb <<= 1; \
	bg_16px_pipe_color_id_lsb <<= 1; \
	bg_16px_pipe_color_id_msb <<= 1

#define PUSH_PALETTE_ID_LSBS_TO_PIPE(palette_id) \
	bg_16px_pipe_palette_id_lsb = ((bg_16px_pipe_palette_id_lsb & 0xFF00) | (palette_id & 0b01 ? 0xFF : 0x00))

#define PUSH_PALETTE_ID_MSBS_TO_PIPE(palette_id) \
	bg_16px_pipe_palette_id_msb = ((bg_16px_pipe_palette_id_msb & 0xFF00) | (palette_id & 0b10 ? 0xFF : 0x00))

#define PUSH_COLOR_ID_LSBS_TO_PIPE(tile_lsbs) \
	bg_16px_pipe_color_id_lsb = ((bg_16px_pipe_color_id_lsb & 0xFF00) | tile_lsbs)

#define PUSH_COLOR_ID_MSBS_TO_PIPE(tile_msbs) \
	bg_16px_pipe_color_id_msb = ((bg_16px_pipe_color_id_msb & 0xFF00) | tile_msbs)

#define INCREMENT_Y() \
	if (vram_addr.fine_y < 7){ \
		vram_addr.fine_y++; \
	} \
	else { \
		vram_addr.fine_y = 0; \
		if (vram_addr.coarse_y == 29) { \
			vram_addr.coarse_y = 0; \
			vram_addr.nametable_y = ~vram_addr.nametable_y; \
		} \
		else if (vram_addr.coarse_y == 31) { \
			vram_addr.coarse_y = 0; \
		} \
		else { \
			vram_addr.coarse_y++; \
		} \
	}
	/*vram_addr.fine_y = (vram_addr.fine_y + 1) % 8; \
	if (vram_addr.fine_y == 0){ \
		vram_addr.coarse_y = (vram_addr.coarse_y + 1) % PPU_NAME_TABLE_DRAWABLE_ROWS_PER_NAME_TABLE; \
		if (vram_addr.coarse_y == 0){ \
			vram_addr.nametable_y = ~vram_addr.nametable_y; \
		} \
	} \*/

#define INCREMENT_X() \
	if (vram_addr.coarse_x == 31){ \
		vram_addr.coarse_x = 0; \
		vram_addr.nametable_x = ~vram_addr.nametable_x; \
	} \
	else { \
		vram_addr.coarse_x++; \
	}
	/*vram_addr.coarse_x = (vram_addr.coarse_x + 1) % PPU_NAME_TABLE_COLS_PER_ROW; \
	if (vram_addr.coarse_x == 0){ \
		vram_addr.nametable_x = ~vram_addr.nametable_x; \
	} \*/

class PPU {

public:
	PPU();
	~PPU();
	void reset();

public:
	void connectCartridge(const std::shared_ptr<Cartridge>& cartridge);
	void advanceClock();

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

	std::shared_ptr<Cartridge> cartridge;

	// Pattern memory goes back to the bitmap times, would hold char maps.
	// In the times of the NES, pattern tables were holding sprites.
	uint8_t patternTables[2][PPU_PATTERN_TABLE_SIZE];
	uint8_t nameTables[2][PPU_NAME_TABLE_SIZE];
	uint8_t palette_mem[PPU_PALETTE_SIZE * 2]; // Size 32

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
	bool frame_complete = false;
	bool nmi = false;

public:

	// Render
	int16_t scanline;
	int16_t scanline_dot;
	uint16_t tile_pixel;
	bool odd_frame_switch;

	uint8_t bg_tile_id_nxt;
	uint8_t bg_tile_attr_byte_nxt;

	uint8_t bg_nxt_8px_palette_id;
	uint8_t bg_nxt_8px_color_id_lsb;
	uint8_t bg_nxt_8px_color_id_msb;

	uint16_t bg_16px_pipe_palette_id_lsb;
	uint16_t bg_16px_pipe_palette_id_msb;
	uint16_t bg_16px_pipe_color_id_lsb;
	uint16_t bg_16px_pipe_color_id_msb;
	
	uint8_t ppu_addr_scroll_latch;
	uint8_t ppu_data_buffer;
	
	ppuctrl_un control_reg;
	ppumask_un mask_reg;
	ppustatus_un status_reg;
	uint8_t oam_addr;
	uint8_t oam_data;
	loopy_un tmp_vram_addr;
	loopy_un vram_addr;
	uint8_t fine_x;

	bool isNTSC;

	// DEBUG
	debug_ppu_state_dsc_st debugPPUState;
	uint64_t frameCounter = 1; // Start at 1, just like MESEN

#if defined(PPU_FILE_LOG) || defined(PPU_TERMINAL_LOG)
	// Log file
	std::ofstream ppuLogFile;
#endif

};