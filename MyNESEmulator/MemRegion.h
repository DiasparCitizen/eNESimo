#pragma once

#include <iostream>

class MemRegion {

public:
	MemRegion() :
		idxOutOfBoundsCnt(0),
		wrapAround(false),
		memSize(0),
		mem(nullptr) {}

	MemRegion(int byteSize) :
		idxOutOfBoundsCnt(0),
		wrapAround(false),
		mem(nullptr) {
		resize(byteSize);
	}

	~MemRegion() {
		// RAII
		delete mem;
	}

	void setWrapAround(bool wrapAround) {
		this->wrapAround = wrapAround;
	}

	void resize(int byteSize) {
		if (mem != nullptr) {
			delete mem;
		}
		mem = (uint8_t*)malloc(byteSize);
		memSize = byteSize;
	}

	uint8_t* data() {
		return mem;
	}

	void clear() {
		if (mem != nullptr) {
			memset(mem, 0x00, memSize);
		}
	}

	uint32_t size() {
		return memSize;
	}

	uint8_t& operator[](uint32_t byteAddr) {
		if (!wrapAround) {
			if (!isValidAddr(byteAddr)) {
				idxOutOfBoundsCnt++;
				return trashBin;
			}
		}
		else {
			byteAddr = byteAddr % memSize;
		}
		return mem[byteAddr];
	}

	void print() {
		for (int i = 0; i < memSize; i++) {
			std::cout << "0x" << std::hex << (int)mem[i] << std::endl;
		}
		std::cout << "Trash bin: " << (int)trashBin << std::endl;
	}

private:
	bool isValidAddr(uint32_t byteAddr) {
		return byteAddr >= 0 && byteAddr < memSize;
	}

	// Disallow copy constructor and copy assignment operator
	MemRegion(MemRegion& memRegion) = delete;
	MemRegion& operator=(MemRegion& memRegion) = delete;

	uint8_t* mem;
	uint32_t memSize;
	uint8_t trashBin = 0;
	uint32_t idxOutOfBoundsCnt;
	bool wrapAround;

};