#pragma once

/**************** GENERAL *****************/

constexpr double NTSC_NES_FREQ = 1789772.72727272;

constexpr double NES_CYCLE_PERIOD = 1 / NTSC_NES_FREQ;
constexpr double SAMPLE_RATE = 44100;
constexpr double SAMPLE_PERIOD = 1 / SAMPLE_RATE;
constexpr double SAMPLES_PER_FRAME = SAMPLE_RATE / 60.1;

constexpr int AMPLITUDE = 26000;

#define RAM_TRASH_VALUE 0x7f
#define INSTRUCTION_CHAR_LEN (4) // Include \0
#define RESET_TICKS 8

#define ACCURATE_PPU_SPRITE_RENDER_EMU
//#define PERFORM_USELESS_NT_READS

/**************** DEBUG *****************/

//#define PPU_DEBUG_MODE

//#define CPU_DEBUG_MODE
//#define FORCE_START_PC
//#define CPU_DEBUG_MODE_START_PC 0xc000

//#define PPU_TERMINAL_LOG
//#define PPU_FILE_LOG
//#define PPU_TERMINAL_LOG2
//#define PPU_FILE_LOG2

//#define CPU_TERMINAL_LOG
//#define CPU_FILE_LOG

//#define BUS_TERMINAL_LOG
//#define BUS_FILE_LOG

/**************** CPU ADDRESS SPACE *****************/

#define CPU_ADDR_SPACE_RAM_START 0x0000
#define CPU_ADDR_SPACE_RAM_SIZE 0x2000 // 8 KiB
#define CPU_ADDR_SPACE_RAM_END (CPU_ADDR_SPACE_RAM_START + CPU_ADDR_SPACE_RAM_SIZE - 1)
#define CPU_ADDR_SPACE_RAM_MIRROR_MASK 0x7FF // Of the 8 KiB, only 2 exist physically
#define CPU_RAM_PAGE_SIZE 256
#define CPU_RAM_PAGE_MASK (CPU_RAM_PAGE_SIZE - 1) // Should yield 0xFF
#define CPU_RAM_PAGE_COUNT (CPU_ADDR_SPACE_RAM_SIZE / CPU_RAM_PAGE_SIZE) // Should yield 32
#define CPU_RAM_PAGE_ID_MASK (CPU_RAM_PAGE_COUNT - 1) // Should yield 0x1F -> The max page id is 31

#define RESET_ADDR 0xFFFC // It's in the cartridge!
#define IRQ_ADDR 0xFFFE
#define NMI_ADDR 0xFFFA

	// Page zero - 0x0000
#define CPU_ADDR_SPACE_RAM_ZERO_PAGE_START CPU_ADDR_SPACE_RAM_START
#define CPU_ADDR_SPACE_RAM_ZERO_PAGE_SIZE 256 // Bytes
#define CPU_ADDR_SPACE_RAM_ZERO_PAGE_END (CPU_ADDR_SPACE_RAM_ZERO_PAGE_START + CPU_ADDR_SPACE_RAM_ZERO_PAGE_SIZE - 1)
#define CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK (CPU_ADDR_SPACE_RAM_ZERO_PAGE_SIZE - 1) // Should evaluate to 0x00FF

	// Stack - 0x0100
#define CPU_ADDR_SPACE_STACK_START (CPU_ADDR_SPACE_RAM_ZERO_PAGE_START + CPU_ADDR_SPACE_RAM_ZERO_PAGE_SIZE) // Should yield 0x0100
#define CPU_ADDR_SPACE_STACK_SIZE 256 // Bytes
#define CPU_ADDR_SPACE_STACK_END (CPU_ADDR_SPACE_STACK_START + CPU_ADDR_SPACE_STACK_SIZE - 1)
#define CPU_ADDR_SPACE_STACK_INITIAL_OFFSET 0xFD

	// OAM (Object Attribute Memory) - 0x0200
#define CPU_ADDR_SPACE_OAM_START (CPU_ADDR_SPACE_STACK_START + CPU_ADDR_SPACE_STACK_SIZE)
#define CPU_ADDR_SPACE_OAM_SIZE 256
#define CPU_ADDR_SPACE_OAM_END (CPU_ADDR_SPACE_OAM_START + CPU_ADDR_SPACE_OAM_SIZE - 1)

	// General memory / RAM... - 0x0300
#define CPU_ADDR_SPACE_GEN_MEM_START (CPU_ADDR_SPACE_OAM_START + CPU_ADDR_SPACE_OAM_SIZE)
#define CPU_ADDR_SPACE_GEN_MEM_SIZE (CPU_ADDR_SPACE_RAM_SIZE - CPU_ADDR_SPACE_RAM_ZERO_PAGE_SIZE - CPU_ADDR_SPACE_STACK_SIZE - CPU_ADDR_SPACE_OAM_SIZE)

	// PPU interface

#define CPU_ADDR_SPACE_PPU_START 0x2000
#define CPU_ADDR_SPACE_PPU_SIZE 0x2000 // 8 KiB
#define CPU_ADDR_SPACE_PPU_END (CPU_ADDR_SPACE_PPU_START + CPU_ADDR_SPACE_PPU_SIZE - 1)
#define CPU_ADDR_SPACE_PPU_MIRROR_MASK 0x0007 // Of the 8 KiB, only 8 bytes exist physically

#define CPU_ADDR_SPACE_PPU_PPU_CTRL 0x2000
#define CPU_ADDR_SPACE_PPU_PPU_MASK 0x2001
#define CPU_ADDR_SPACE_PPU_STATUS_REG 0x2002
#define CPU_ADDR_SPACE_PPU_SPRITE_MEM_ADDR 0x2003
#define CPU_ADDR_SPACE_PPU_SPRITE_MEM_DATA 0x2004
#define CPU_ADDR_SPACE_PPU_BG_SCROLL 0x2005
#define CPU_ADDR_SPACE_PPU_VRAM_ADDR 0x2006
#define CPU_ADDR_SPACE_PPU_VRAM_DATA 0x2007

	// Cartridge program ROM - 0x8000
#define CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START 0x8000
#define CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_SIZE (32 * 1024) // 32 KiB
#define CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_END (CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_START + CPU_ADDR_SPACE_CARTRIDGE_PRG_ROM_SIZE - 1) // Should yield 0xFFFF

	// OAM
#define CPU_ADDR_SPACE_OAM_DMA 0x4014

/**************** PPU ADDRESS SPACE *****************/

#define PPU_ADDR_SPACE_MASK ((1 << 14) - 1) // The bus width is 14 bits

#define PPU_NAME_TABLE_SIZE 0x400 // 1 KiB
#define PPU_NAME_TABLE_MASK (PPU_NAME_TABLE_SIZE - 1) // Should yield 0x3FF
#define PPU_NAME_TABLE_ROWS_PER_NAME_TABLE 32
#define PPU_NAME_TABLE_DRAWABLE_ROWS_PER_NAME_TABLE 30
#define PPU_NAME_TABLE_COLS_PER_ROW 32
#define PPU_NAME_TABLE_ATTRIBUTE_TABLE_SIZE 64 // Last 64 bytes of each nametable
#define PPU_NAME_TABLE_ATTRIBUTE_TABLE_OFFSET (PPU_NAME_TABLE_SIZE - PPU_NAME_TABLE_ATTRIBUTE_TABLE_SIZE) // Should yield 0x3C0
#define PPU_NAME_TABLE_REGION_SIZE (PPU_NAME_TABLE_SIZE * 4) // 4 KiB
#define PPU_NAME_TABLE_REGION_MASK (PPU_NAME_TABLE_REGION_SIZE - 1) // Should yield 0x0FFF

#define PPU_PATTERN_TABLE_SIZE 0x1000
#define PPU_PATTERN_TABLE_MASK (PPU_PATTERN_TABLE_SIZE - 1)

#define PPU_PALETTE_SIZE 0x10
#define PPU_PALETTE_ID_BIT_LEN 2
#define PPU_PALETTE_ID_MASK ((PPU_PALETTE_ID_BIT_LEN << 2) - 1) // Should yield 0b11

#define PPU_OAM_SIZE 256

	//////////// PATTERN TABLE REGION
#define PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START 0x0000

	// PPU pattern table 0 - 0x0000
#define PPU_ADDR_SPACE_PATTERN_TABLE_0_START PPU_ADDR_SPACE_PATTERN_TABLE_REGION_START
#define PPU_ADDR_SPACE_PATTERN_TABLE_0_END (PPU_ADDR_SPACE_PATTERN_TABLE_0_START + PPU_PATTERN_TABLE_SIZE - 1)
	// PPU pattern table 1 - 0x1000
#define PPU_ADDR_SPACE_PATTERN_TABLE_1_START (PPU_ADDR_SPACE_PATTERN_TABLE_0_START + PPU_PATTERN_TABLE_SIZE)
#define PPU_ADDR_SPACE_PATTERN_TABLE_1_END (PPU_ADDR_SPACE_PATTERN_TABLE_1_START + PPU_PATTERN_TABLE_SIZE - 1)

#define PPU_ADDR_SPACE_PATTERN_TABLES_MASK (PPU_ADDR_SPACE_PATTERN_TABLE_0_START + (PPU_PATTERN_TABLE_SIZE * 2) - 1) // 0x1FFF

	//////////// NAME TABLE REGION START
#define PPU_ADDR_SPACE_NAME_TABLE_REGION_START (PPU_ADDR_SPACE_PATTERN_TABLE_1_START + PPU_PATTERN_TABLE_SIZE) // Should yield 0x2000

	// PPU name table 0 - 0x2000
#define PPU_ADDR_SPACE_NAME_TABLE_0_START PPU_ADDR_SPACE_NAME_TABLE_REGION_START
#define PPU_ADDR_SPACE_NAME_TABLE_0_END (PPU_ADDR_SPACE_NAME_TABLE_0_START + PPU_NAME_TABLE_SIZE - 1)
	// PPU name table 1 - 0x2400
#define PPU_ADDR_SPACE_NAME_TABLE_1_START (PPU_ADDR_SPACE_NAME_TABLE_0_START + PPU_NAME_TABLE_SIZE)
#define PPU_ADDR_SPACE_NAME_TABLE_1_END (PPU_ADDR_SPACE_NAME_TABLE_1_START + PPU_NAME_TABLE_SIZE - 1)
	// PPU name table 2 - 0x2800
#define PPU_ADDR_SPACE_NAME_TABLE_2_START (PPU_ADDR_SPACE_NAME_TABLE_1_START + PPU_NAME_TABLE_SIZE)
#define PPU_ADDR_SPACE_NAME_TABLE_2_END (PPU_ADDR_SPACE_NAME_TABLE_2_START + PPU_NAME_TABLE_SIZE - 1)
	// PPU name table 3 - 0x2C00
#define PPU_ADDR_SPACE_NAME_TABLE_3_START (PPU_ADDR_SPACE_NAME_TABLE_2_START + PPU_NAME_TABLE_SIZE)
#define PPU_ADDR_SPACE_NAME_TABLE_3_END (PPU_ADDR_SPACE_NAME_TABLE_3_START + PPU_NAME_TABLE_SIZE - 1)
	// Palettes - 0x3F00
#define PPU_ADDR_SPACE_PALETTES_REGION_START 0x3F00
#define PPU_ADDR_SPACE_PALETTES_REGION_END 0x3FFF
#define PPU_ADDR_SPACE_PALETTES_REGION_MASK 0x1F // Only 32 useful bytes
#define PPU_ADDR_SPACE_SINGLE_PALETTE_MASK 0xF
		// Image palette - 0x3F00
#define PPU_ADDR_SPACE_IMAGE_PALETTE_START PPU_ADDR_SPACE_PALETTES_REGION_START
		// Sprite palette - 0x3F10
#define PPU_ADDR_SPACE_SPRITE_PALETTE_START (PPU_ADDR_SPACE_IMAGE_PALETTE_START + PPU_PALETTE_SIZE)
#define PPU_ADDR_SPACE_SPRITE_PALETTE_END (PPU_ADDR_SPACE_SPRITE_PALETTE_START + PPU_PALETTE_SIZE - 1)

/**************** APU ADDRESS SPACE *****************/

#define APU_ADDR_SPACE_PULSE_1_REG1 0x4000
#define APU_ADDR_SPACE_PULSE_1_REG2 0x4001
#define APU_ADDR_SPACE_PULSE_1_REG3 0x4002
#define APU_ADDR_SPACE_PULSE_1_REG4 0x4003

#define APU_ADDR_SPACE_PULSE_2_REG1 0x4004
#define APU_ADDR_SPACE_PULSE_2_REG2 0x4005
#define APU_ADDR_SPACE_PULSE_2_REG3 0x4006
#define APU_ADDR_SPACE_PULSE_2_REG4 0x4007

#define APU_ADDR_SPACE_TRIANGLE_CH_REG1 0x4008
#define APU_ADDR_SPACE_TRIANGLE_CH_REG2 0x4009
#define APU_ADDR_SPACE_TRIANGLE_CH_REG3 0x400A
#define APU_ADDR_SPACE_TRIANGLE_CH_REG4 0x400B

#define APU_ADDR_SPACE_NOISE_CH_REG1 0x400C
#define APU_ADDR_SPACE_NOISE_CH_REG2 0x400D
#define APU_ADDR_SPACE_NOISE_CH_REG3 0x400E
#define APU_ADDR_SPACE_NOISE_CH_REG4 0x400F

#define APU_ADDR_SPACE_SAMPLE_LENGTH 0x4013

#define APU_ADDR_SPACE_FRAME_STATUS 0x4015
#define APU_ADDR_SPACE_FRAME_COUNTER 0x4017

/**************** CONTROLLER *****************/

#define CPU_ADDR_SPACE_CONTROLLER_1 0x4016
#define CPU_ADDR_SPACE_CONTROLLER_2 0x4017

/**************** CARTRIDGES *****************/

// CARTRIDGE

#define CARTRIDGE_PRG_BANK_SIZE (1024 * 16)
#define CARTRIDGE_CHAR_BANK_SIZE (1024 * 8)


// MAPPERS
	// Mapper 000

#define MAPPER_000_1_PRG_BANKS_ADDR_MASK 0x3FFF // 16 KiB
#define MAPPER_000_2_PRG_BANKS_ADDR_MASK 0x7FFF // 32 KiB

/**************** PPU *****************/

#define DOTS_PER_SCANLINE 341
#define MAX_SCANLINE_DOT (DOTS_PER_SCANLINE - 1)