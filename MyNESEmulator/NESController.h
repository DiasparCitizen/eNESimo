#pragma once
#include <cstdint>

// https://tresi.github.io/nes/

#define CONTROLLER_BTN_A_BITPOS 7
#define CONTROLLER_BTN_B_BITPOS 6
#define CONTROLLER_BTN_SEL_BITPOS 5
#define CONTROLLER_BTN_STA_BITPOS 4
#define CONTROLLER_BTN_UP_BITPOS 3
#define CONTROLLER_BTN_DOWN_BITPOS 2
#define CONTROLLER_BTN_LEFT_BITPOS 1
#define CONTROLLER_BTN_RIGHT_BITPOS 0

#define CONTROLLER_BTN_A_MASK (1 << CONTROLLER_BTN_A_BITPOS)
#define CONTROLLER_BTN_B_MASK (1 << CONTROLLER_BTN_B_BITPOS)
#define CONTROLLER_BTN_SEL_MASK (1 << CONTROLLER_BTN_SEL_BITPOS)
#define CONTROLLER_BTN_STA_MASK (1 << CONTROLLER_BTN_STA_BITPOS)
#define CONTROLLER_BTN_UP_MASK (1 << CONTROLLER_BTN_UP_BITPOS)
#define CONTROLLER_BTN_DOWN_MASK (1 << CONTROLLER_BTN_DOWN_BITPOS)
#define CONTROLLER_BTN_LEFT_MASK (1 << CONTROLLER_BTN_LEFT_BITPOS)
#define CONTROLLER_BTN_RIGHT_MASK (1 << CONTROLLER_BTN_RIGHT_BITPOS)

class NESController {

public:

	NESController() {
		_controllerBtns = 0x00;
		_controllerStateLatch = 0x00;
		_latch = false;
	}
	~NESController() {}

	void cpuWrite(uint8_t data) {
		_latch = data > 0;
		if (_latch) {
			// Latch current state of buttons
			_controllerStateLatch = _controllerBtns;
		}
	}

	uint8_t cpuRead() {
		// Return MSB
		uint8_t msb = (_controllerStateLatch >> 7) & 0x1;
		_controllerStateLatch <<= 1; // Shift left
		return msb;
	}

	void setA(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_A_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_A_BITPOS);
	}

	void setSelect(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_SEL_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_SEL_BITPOS);
	}

	void setB(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_B_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_B_BITPOS);
	}

	void setStart(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_STA_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_STA_BITPOS);
	}

	void setUP(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_UP_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_UP_BITPOS);
	}

	void setDOWN(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_DOWN_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_DOWN_BITPOS);
	}

	void setLEFT(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_LEFT_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_LEFT_BITPOS);
	}

	void setRIGHT(uint8_t set) {
		_controllerBtns &= ~CONTROLLER_BTN_RIGHT_MASK;
		_controllerBtns |= (set << CONTROLLER_BTN_RIGHT_BITPOS);
	}

private:

	volatile uint8_t _controllerBtns;
	uint8_t _controllerStateLatch;
	bool _latch;

};