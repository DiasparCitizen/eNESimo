#pragma once

#include <cstdint>
#include "CommonLibs.h"
#include "NESConstants.h"

// http://www.slack.net/~ant/nes-emu/apu_ref.txt

// Audio emulation approaches - NESDEV forum
// https://forums.nesdev.com/viewtopic.php?t=10048
// https://forums.nesdev.com/viewtopic.php?t=18493

// Great thread:
// https://forums.nesdev.com/viewtopic.php?f=5&t=15383

// Blip-buf library
// https://code.google.com/archive/p/blip-buf/

// http://forums.nesdev.com/viewtopic.php?f=3&t=3808

// Already +1; the length counter mutes the channel when clocked *while 0*
constexpr uint16_t lengthCounterLut[] = {10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
										12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30};

constexpr uint8_t triangleSequencerValues[] = {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

constexpr uint8_t waveForms[] = { 0b01000000, 0b01100000, 0b01111000, 0b10011111 };

struct frame_counter_st {
	uint8_t unused : 6;
	uint8_t mode : 1; // (M, 0 = 4-step, 1 = 5-step)
	uint8_t irq_inhibit : 1; // IRQ (if bit 6 is clear)
};

struct pulse_wave_reg1_st { // 0x4000
	uint8_t volume : 4; // Volume
	uint8_t constantVolumeFlag : 1; // Constant volume flag (0: use volume from envelope; 1: use constant volume)
	uint8_t lengthCounterHalt : 1; // Length counter halt, loop flag for the envelope
	uint8_t duty : 2; // Duty
};

// Sweep unit
struct pulse_wave_reg2_st { // 0x4001
	uint8_t shiftCount : 3;
	uint8_t negate : 1; // 0: add to period, sweeping toward lower frequencies; 1: subtract from period, sweeping toward higher frequencies
	uint8_t period : 3; // Period of the wave
	uint8_t enabled : 1;
};

struct pulse_wave_reg4_st { // 0x4003
	uint8_t timer_hi : 3; // 3 MSB of the timer
	uint8_t lengthCounterLoad : 5; // Value we get from the length table
};

struct triangle_wave_reg1_st {
	uint8_t linearCounterLoad : 7;
	uint8_t lengthCounterHaltAndLinearCounterControl : 1;
};

struct triangle_wave_reg3_st {
	uint8_t timerLo;
};

struct triangle_wave_reg4_st {
	uint8_t timerHi : 3;
	uint8_t lengthCounterLoad : 5;
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

struct length_counter_unit_st {

	uint16_t divider;
	bool enabled;
	bool halt; // Halt flag

	void setEnabled(bool en) {
		// Writing a zero to any of the channel enable bits will silence that channel
		// and immediately set its length counter to 0.
		enabled = en;
		if (!en) {
			divider = 0;
		}
	}

	void clock() {

		if (divider > 0 && !halt) {
			divider--;
		}

	}

};

struct envelope_unit_st {

	bool startFlag;
	bool loopFlag;
	bool constantVolumeFlag;

	uint16_t volume; // == period
	uint16_t dividerCount;

	uint16_t decayLevelCounter;

	uint8_t getVolume(){
		return constantVolumeFlag ? volume : decayLevelCounter;
	}

	void clock() {

		if (!startFlag) { // Reviewed, fine

			if (dividerCount == 0) {

				dividerCount = volume;

				if (decayLevelCounter > 0) {
					decayLevelCounter--;
				}
				else {
					if (loopFlag) decayLevelCounter = 15;
				}

			}
			else {
				dividerCount--;
			}

		}
		else {
			startFlag = false;
			decayLevelCounter = 15;
			dividerCount = volume;
		}

	}

};

// https://wiki.nesdev.com/w/index.php/APU_Sweep
struct sweep_unit_st {

	uint8_t pulseChId;
	bool enabled;

	uint8_t sweepDivider;
	uint8_t sweepPeriod;
	bool reloadFlag;

	uint16_t* chTimer;
	uint16_t* chConfiguredTimer; // This is the value written by the CPU

	// 0: add to period, sweeping toward lower frequencies
	// 1: subtract from period, sweeping toward higher frequencies
	bool negate;

	uint8_t shiftCount; // Shift count (number of bits)

	void init(uint16_t* timer, uint16_t* configuredTimer, uint8_t id) {
		// Set values from outside
		chTimer = timer;
		chConfiguredTimer = configuredTimer;
		pulseChId = id;
		// Initialize other values
		enabled = true;
		negate = false;
	}

	bool isMuted() {
		uint16_t changeAmount = *chTimer >> shiftCount;
		return ((*chTimer) < 8) || (!negate && (*chTimer + changeAmount) > 0x7FF);
	}

	void updateTargetPeriod() {

		uint16_t changeAmount = *chTimer >> shiftCount;

		if (negate) {
			*chTimer -= changeAmount + (1 - pulseChId);
		}
		else if (*chTimer + changeAmount <= 0x7FF) {
			*chTimer += changeAmount;
		}

	}

	void clock() {

		sweepDivider--;

		// If the divider's counter is zero, the sweep is enabled, and the sweep
		// unit is not muting the channel: The pulse's period is adjusted.
		if (sweepDivider == 0 && shiftCount && enabled
			&& (*chConfiguredTimer) >= 8) {

			updateTargetPeriod();

			sweepDivider = sweepPeriod;

		}

		if (reloadFlag) {
			// Restart count for internal divider
			sweepDivider = sweepPeriod;
			reloadFlag = false;
		}

	}

};

/*

--------------
Square Channel
--------------

				   +---------+    +---------+
				   |  Sweep  |--->|Timer / 2|
				   +---------+    +---------+
						|              |
						|              v
						|         +---------+    +---------+
						|         |Sequencer|    | Length  |
						|         +---------+    +---------+
						|              |              |
						v              v              v
	+---------+        |\             |\             |\          +---------+
	|Envelope |------->| >----------->| >----------->| >-------->|   DAC   |
	+---------+        |/             |/             |/          +---------+

*/
struct pulse_wave_engine_st {

	// Timer unit
	uint16_t configuredTimer;
	uint16_t timer; // Also called period

	// Sequencer unit
	// Easier if not encapsulated in its own struct
	uint8_t waveForm;
	uint8_t waveFormOffs; // An offset to a bit in waveForm

	// Remaining units
	length_counter_unit_st lengthCounterUnit;
	sweep_unit_st sweepUnit;
	envelope_unit_st envelopeUnit;

	uint8_t output;

	void init(uint8_t id) {
		sweepUnit.init(&timer, &configuredTimer, id);
		timer = 0;
		configuredTimer = 0;
		waveForm = 0;
		waveFormOffs = 0;
	}

	void reloadTimer() {
		timer = (configuredTimer << 1) + 2;
	}

	void restartSequencer() {
		waveFormOffs = 0;
	}

	void clock() {

		timer--;
		if (timer == 0xFFFF) { // When -1 is reached
			// Reload time
			reloadTimer();
			// Advance sequence step (really an index to the wave form sequence)
			waveFormOffs = (waveFormOffs + 1) & 0x7;
			output = (waveForm >> waveFormOffs) & 0x1;
		}

	}

};

struct linear_counter_engine_st {

	uint16_t counter;
	uint16_t configuredCounter;
	bool halt;
	bool control;

	linear_counter_engine_st() {
		halt = false;
		control = false;
		counter = 0;
		configuredCounter = 0;
	}

	void reload() {
		counter = configuredCounter;
	}

	void clock() {

		if (halt) {
			counter = configuredCounter;
		}
		else if (counter > 0) {
			counter--;
		}

		if (!control) {
			halt = false;
		}

	}

};

struct triangle_wave_engine_st {

	uint16_t timer;
	uint16_t configuredTimer;

	linear_counter_engine_st linearCounterUnit;
	length_counter_unit_st lengthCounterUnit;

	uint16_t sequencerOffset = 0;

	uint8_t output;

	void reloadTimer() {
		timer = configuredTimer;
	}

	void clock() {

		if (timer == 0) {
			reloadTimer();
			if (configuredTimer < 2) return;
			// The sequencer is clocked by the timer as long as both the
			// linear counter and the length counter are nonzero.
			if (linearCounterUnit.counter != 0 && lengthCounterUnit.divider != 0) {
				output = triangleSequencerValues[sequencerOffset];
				sequencerOffset = (sequencerOffset + 1) % 32;
			}
		}
		timer--;

	}

};

enum class sequence_step {
	STEP_1,
	STEP_2,
	STEP_3,
	STEP_4,
	STEP_4P5,
	STEP_5,
	CYCLE_0,
	NONE
};

// AKA frame sequencer
struct frame_counter_engine_st {

	uint16_t cpuCycleCount;
	uint8_t mode;

	sequence_step clock() {

		sequence_step step = sequence_step::NONE;

		if (mode == 0) {
			if (cpuCycleCount == 7457 /*3728.5*/) { // Step 1
				step = sequence_step::STEP_1;
			}
			else if (cpuCycleCount == 14913 /*7456.5*/) { // Step 2
				step = sequence_step::STEP_2;
			}
			else if (cpuCycleCount == 22371 /*11185.5*/) { // Step 3
				step = sequence_step::STEP_3;
			}
			else if (cpuCycleCount == 29828 /*14914.0*/) { // Step 4
				step = sequence_step::STEP_4;
			}
			else if (cpuCycleCount == 29829 /*14914.5*/) { // Step 4.5
				step = sequence_step::STEP_4P5;
				cpuCycleCount = 0;
			}
		}
		else { // Mode 1: 5 steps
			if (cpuCycleCount == 7457 /*3728.5*/) { // Step 1
				step = sequence_step::STEP_1;
			}
			else if (cpuCycleCount == 14913 /*7456.5*/) { // Step 2
				step = sequence_step::STEP_2;
			}
			else if (cpuCycleCount == 22371 /*11185.5*/) { // Step 3
				step = sequence_step::STEP_3;
			}
			else if (cpuCycleCount == 29829 /*14914.5*/) { // Step 4
				step = sequence_step::STEP_4;
			}
			else if (cpuCycleCount == 37281 /*18640.5*/) { // Step 5
				step = sequence_step::STEP_5;
				cpuCycleCount = 0;
			}
		}

		if (cpuCycleCount == 0) {
			step = sequence_step::CYCLE_0;
		}

		cpuCycleCount++;

		return step;

	}

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

	void setPulseWaveReg1Fields(uint8_t id, pulse_wave_reg1_st data);
	void setPulseWaveReg2Fields(uint8_t id, pulse_wave_reg2_st data);
	void setPulseWaveReg3Fields(uint8_t id, uint8_t data);
	void setPulseWaveReg4Fields(uint8_t id, pulse_wave_reg4_st data);

	void writeTriangleWaveReg1(uint8_t data);
	void writeTriangleWaveReg2(uint8_t data);
	void writeTriangleWaveReg3(uint8_t data);
	void writeTriangleWaveReg4(uint8_t data);

	void writeStatusReg(uint8_t data);
	void writeFrameCounterReg(uint8_t data);

	uint8_t readStatusReg();

	sample_t getOutput();

private:
	Bus* _nes;

	bool _oddCycle = true;

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
	// Triangle wave
	triangle_wave_reg1_st triangleWaveReg1;
	triangle_wave_reg3_st triangleWaveReg3;
	triangle_wave_reg4_st triangleWaveReg4;

	status_wr_reg_st statusWrReg;
	status_rd_reg_st statusRdReg;

	frame_counter_st frameCounterReg;
	pulse_wave_engine_st _pulseWaveEngines[2];
	triangle_wave_engine_st _triangleWaveEngine;
	frame_counter_engine_st _frameCounterEngine;

public:
	bool _frameInterruptFlag;

};