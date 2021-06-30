#pragma once

#include <vector>
#include <iostream>

class MemRegion {

public:
	MemRegion() : idxOutOfBoundsCnt(0), wrapAround(false) {
		mem.resize(1);
		wrapAround = false;
	}

	MemRegion(int byteSize) : idxOutOfBoundsCnt(0), wrapAround(false) {
		mem.resize(byteSize);
	}

	void setWrapAround(bool wrapAround) {
		this->wrapAround = wrapAround;
	}

	void resize(int byteSize) {
		mem.resize(byteSize);
	}

	uint8_t* data() {
		return mem.data();
	}

	void clear() {
		mem.clear();
	}

	uint8_t& operator[](int byteAddr) {
		if (!wrapAround) {
			if (!isValidIdx(byteAddr)) {
				idxOutOfBoundsCnt++;
				return trashBin;
			}
		}
		else {
			byteAddr = byteAddr % mem.size();
		}
		return mem[byteAddr];
	}

	void print() {
		for (int i = 0; i < mem.size(); i++) {
			std::cout << "0x" << std::hex << (int)mem[i] << std::endl;
		}
		std::cout << "Trash bin: " << (int)trashBin << std::endl;
	}

private:
	bool isValidIdx(int index) {
		return index >= 0 && index < mem.size();
	}

private:
	std::vector<uint8_t> mem;
	uint8_t trashBin = 0;
	uint32_t idxOutOfBoundsCnt;
	bool wrapAround;

};