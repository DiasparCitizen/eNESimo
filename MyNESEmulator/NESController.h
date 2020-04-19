#pragma once
#include <cstdint>

// https://tresi.github.io/nes/

#define CONTROLLER_BTN_X_BITPOS 7
#define CONTROLLER_BTN_Z_BITPOS 6
#define CONTROLLER_BTN_A_BITPOS 5
#define CONTROLLER_BTN_S_BITPOS 4
#define CONTROLLER_BTN_UP_BITPOS 3
#define CONTROLLER_BTN_DOWN_BITPOS 2
#define CONTROLLER_BTN_LEFT_BITPOS 1
#define CONTROLLER_BTN_RIGHT_BITPOS 0

#define CONTROLLER_BTN_X_MASK (1 << CONTROLLER_BTN_X_BITPOS)
#define CONTROLLER_BTN_Z_MASK (1 << CONTROLLER_BTN_Z_BITPOS)
#define CONTROLLER_BTN_A_MASK (1 << CONTROLLER_BTN_A_BITPOS)
#define CONTROLLER_BTN_S_MASK (1 << CONTROLLER_BTN_S_BITPOS)
#define CONTROLLER_BTN_UP_MASK (1 << CONTROLLER_BTN_UP_BITPOS)
#define CONTROLLER_BTN_DOWN_MASK (1 << CONTROLLER_BTN_DOWN_BITPOS)
#define CONTROLLER_BTN_LEFT_MASK (1 << CONTROLLER_BTN_LEFT_BITPOS)
#define CONTROLLER_BTN_RIGHT_MASK (1 << CONTROLLER_BTN_RIGHT_BITPOS)

class NESController {

public:

	NESController() {
		_controller_btns = 0x00;
		_controller_state_latch = 0x00;
		_latch = false;
	}
	~NESController() {}

	void cpuWrite(uint8_t data) {
		_latch = data > 0;
		if (_latch) {
			// Latch current state of buttons
			_controller_state_latch = _controller_btns;
		}
	}

	uint8_t cpuRead() {
		// Return MSB
		uint8_t msb = (_controller_state_latch >> 7) & 0x1;
		_controller_state_latch <<= 1; // Shift left
		return msb;
	}

	void setA(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_A_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_A_BITPOS);
	}

	void setZ(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_Z_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_Z_BITPOS);
	}

	void setX(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_X_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_X_BITPOS);
	}

	void setS(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_S_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_S_BITPOS);
	}

	void setUP(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_UP_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_UP_BITPOS);
	}

	void setDOWN(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_DOWN_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_DOWN_BITPOS);
	}

	void setLEFT(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_LEFT_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_LEFT_BITPOS);
	}

	void setRIGHT(uint8_t set) {
		_controller_btns &= ~CONTROLLER_BTN_RIGHT_MASK;
		_controller_btns |= (set << CONTROLLER_BTN_RIGHT_BITPOS);
	}

private:

	volatile uint8_t _controller_btns;
	uint8_t _controller_state_latch;
	bool _latch;

};