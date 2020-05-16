#pragma once

#include <cstdint>
#include "NESConstants.h"

struct frame_counter_st {
	uint8_t unused : 6;
	uint8_t mode : 1; // (M, 0 = 4-step, 1 = 5-step)
	uint8_t irq_inhibit : 1; // IRQ (if bit 6 is clear)
};

struct pulse_wave_reg1_st {
	uint8_t envelope : 5; // Volume
	uint8_t lengthCounterHalt : 1; // Length counter halt, loop flag for the length counter
	uint8_t duty : 2; // Duty
};

// Sweep unit
struct pulse_wave_reg2_st {
	uint8_t shiftCount : 3;
	uint8_t negate : 1;
	uint8_t period : 3; // Period of the wave
	uint8_t enabled : 1;
};

struct pulse_wave_reg4_st {
	uint8_t timer_hi : 3; // 3 MSB of the timer
	uint8_t lengthCounterLoad : 5; // Value we get from the length table
};

struct status_wr_reg_st {
	uint8_t enablePulseCh1 : 1;
	uint8_t enablePulseCh2 : 1;
	uint8_t enableTriangleChl : 1;
	uint8_t enableNoiseChl : 1;
	uint8_t enableDMC : 1;
	uint8_t unused : 3;
};

struct status_rd_reg_st {
	uint8_t pulseCh1LenCntActive : 1;
	uint8_t pulseCh2LenCntActive : 1;
	uint8_t triangleChLenCntActive : 1;
	uint8_t noiseChLenCntActive : 1;
	uint8_t dmcActive : 1;
	uint8_t unused : 1;
	uint8_t frameInterrupt : 1;
	uint8_t dmcInterrupt : 1;
};

// Forward declare
class Bus;

class APU {

public:
	void reset();
	void clock();
	void connectConsole(Bus* bus);

	void writePulseWave1Reg1(uint8_t data);
	void writePulseWave1Reg2(uint8_t data);
	void writePulseWave1Reg3(uint8_t data);
	void writePulseWave1Reg4(uint8_t data);

	void writePulseWave2Reg1(uint8_t data);
	void writePulseWave2Reg2(uint8_t data);
	void writePulseWave2Reg3(uint8_t data);
	void writePulseWave2Reg4(uint8_t data);

	void writeStatusReg(uint8_t data);
	void writeFrameCounterReg(uint8_t data);

	uint8_t readStatusReg();

private:
	Bus* _nes;

	// Pulse wave 1
	pulse_wave_reg1_st pulseWave1Reg1;
	pulse_wave_reg2_st pulseWave1Reg2;
	uint8_t pulseWave1Reg3; // Timer LO
	pulse_wave_reg4_st pulseWave1Reg4;
	// Pulse wave 2
	pulse_wave_reg1_st pulseWave2Reg1;
	pulse_wave_reg2_st pulseWave2Reg2;
	uint8_t pulseWave2Reg3; // Timer LO
	pulse_wave_reg4_st pulseWave2Reg4;

	status_wr_reg_st statusWrReg;
	status_rd_reg_st statusRdReg;


	frame_counter_st frameCounterReg;

	uint32_t _apuCycleCount;
	bool _oddCycle;

};