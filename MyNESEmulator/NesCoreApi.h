#pragma once

#include <cstdint>

struct pixel_st {
	int16_t y;
	int16_t x;
	uint8_t paletteColorCode;
	//
	uint32_t pixelVal;
};

#define NES_RESOLUTION_WIDTH 256
#define NES_RESOLUTION_HEIGHT 240