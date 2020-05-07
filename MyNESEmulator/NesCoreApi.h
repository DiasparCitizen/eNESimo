#pragma once

#include <cstdint>

struct pixel_st {
	uint32_t pixelVal;
	//
	uint16_t scanline;
	uint16_t scanlineCycle;
	uint8_t paletteColorCode;
};