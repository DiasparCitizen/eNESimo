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

// Refs:
// https://wiki.nesdev.com/w/index.php/Standard_controller

class NESController {

public:

	NESController() {
		_controllerBtns = 0x00;
		_controllerStateLatch = 0x00;
		_latch = false;
		_readCount = 0;
		_connected = false;
	}
	~NESController() {}

	void cpuWrite(uint8_t data) {

		_latch = data > 0;
		if (_latch) {
			// Latch current state of buttons
			_controllerStateLatch = _controllerBtns;
			_readCount = 8;
		}

	}

	uint8_t cpuRead() {

		if (!_connected) {
			return 0x0;
		}

		uint8_t readValue = 0x1;

		if (_readCount > 0) {

			// Move counter
			_readCount--;

			// Return MSB
			// Each read reports one bit at a time through D0.The first 8 reads will indicate
			// which buttons or directions are pressed (1 if pressed, 0 if not pressed). All subsequent
			// reads will return 1 on official Nintendo brand controllers but may return 0 on third party controllers such as the U - Force.
			readValue = (_controllerStateLatch >> 7) & 0x1;
			_controllerStateLatch <<= 1; // Shift left

		}

		// In the NES and Famicom, the top three (or five) bits are not driven, and so retain the bits of the previous
		// byte on the bus. Usually this is the most significant byte of the address of the controller port—0x40.
		// Certain games (such as Paperboy) rely on this behavior and require that reads from the controller ports return
		// exactly $40 or $41 as appropriate.
		readValue |= 0x4;

		return readValue;

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

	void setConnected(bool connected) {
		_connected = connected;
	}

private:

	volatile uint8_t _controllerBtns;
	uint8_t _controllerStateLatch;
	bool _latch;

	int16_t _readCount;

	bool _connected;

};