#include "APU.h"
#include "Bus.h"

void APU::reset()
{
	_oddCycle = true;
	_apuCycleCount = 0;
}

void APU::clock()
{

	// Sequencer
	if (frameCounterReg.mode == 0) {
		if (_apuCycleCount == 0) {
			/*if (!frameCounterReg.irq_inhibit) {
				_nes->_cpu._irqOccurred = true;
			}*/
		}
		else if (_apuCycleCount == 3728) {

		}
		else if (_apuCycleCount == 7456) {

		}
		else if (_apuCycleCount == 11185) {

		}
		else if (_apuCycleCount == 14914) {
			_apuCycleCount = 0;
			if (!frameCounterReg.irq_inhibit) {
				_nes->_cpu._irqOccurred = true;
			}
		}
	}
	else { // Mode 1: 5 steps
		if (_apuCycleCount == 3728) {

		}
		else if (_apuCycleCount == 7456) {

		}
		else if (_apuCycleCount == 11185) {

		}
		else if (_apuCycleCount == 14914) {

		}
		else if (_apuCycleCount == 18640) {

		}
	}

	if (_oddCycle) {
		_apuCycleCount++;
	}
	_oddCycle = !_oddCycle;

}

void APU::connectConsole(Bus* bus)
{
	_nes = bus;
}

void APU::writePulseWave1Reg1(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg1) = data;
}

void APU::writePulseWave1Reg2(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg2) = data;
}

void APU::writePulseWave1Reg3(uint8_t data)
{
	pulseWave1Reg3 = data;
}

void APU::writePulseWave1Reg4(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg4) = data;
}

void APU::writePulseWave2Reg1(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg1) = data;
}

void APU::writePulseWave2Reg2(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg2) = data;
}

void APU::writePulseWave2Reg3(uint8_t data)
{
	pulseWave2Reg3 = data;
}

void APU::writePulseWave2Reg4(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg4) = data;
}

void APU::writeStatusReg(uint8_t data)
{
	*((uint8_t*)&statusWrReg) = data;
}

void APU::writeFrameCounterReg(uint8_t data)
{
	*((uint8_t*)&frameCounterReg) = data;
}

uint8_t APU::readStatusReg()
{
	return *((uint8_t*)&statusRdReg);
}
