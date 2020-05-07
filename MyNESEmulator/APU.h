#pragma once

#include <cstdint>
#include "NESConstants.h"

struct frame_counter_st {
	uint8_t unused : 6;
	uint8_t mode : 1;
	uint8_t irq_inhibit : 1;
};

// Forward declare
class Bus;

class APU {

public:
	void clock();
	void connectConsole(Bus* bus);

	void writeFrameCounterReg(uint8_t data);
	uint8_t readFrameCounterReg(uint8_t data);











private:
	Bus* _nes;
	frame_counter_st frameCounterReg;

};