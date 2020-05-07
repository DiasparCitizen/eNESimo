#include "APU.h"

void APU::clock()
{





}

void APU::connectConsole(Bus* bus)
{
	_nes = bus;
}

void APU::writeFrameCounterReg(uint8_t data)
{
	*((uint8_t*)&frameCounterReg) = data;
}

uint8_t APU::readFrameCounterReg(uint8_t data)
{
	return uint8_t();
}
