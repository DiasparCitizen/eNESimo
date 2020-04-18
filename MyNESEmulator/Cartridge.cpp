#include "Cartridge.h"
#include <fstream>
#include "Mapper000.h"
#include <iostream>

Cartridge::Cartridge(const std::string& cartridgeFileName)
{

	ines_header_st inesHeader;

	std::ifstream ifs;
	ifs.open(cartridgeFileName, std::ifstream::binary);

	if (ifs.is_open()) {

		// Just read the header for now
		ifs.read((char*)&inesHeader, sizeof(ines_header_st));

		// Cache the header
		cartridgeHeader = inesHeader;

		// Apparently, trainer info is sort of junk.
		// No need to read it or understand it, for now.
		if (inesHeader.has512ByteTrainer) {
			ifs.seekg(512); // Skip 512 bytes
		}

		// Construct mapper id
		mapperId = (inesHeader.mapperNumberUpperNibble << 4) | (inesHeader.mapperNumberLowerNibble);

		// Scroll type
		vertical = inesHeader.mirroring; // Actually more complex, but good for now

		// File type; assume 1 for now
		uint8_t fileType = 1;

		if (fileType == 0) {

		}

		if (fileType == 1) {

			prgBankCount = inesHeader.prgRom16KiBChunkCount;
			charBankCount = inesHeader.chrRom8KiBChunkCount;
			
			// Form vector to hold program data
			uint32_t totalPrgByteSize = CARTRIDGE_PRG_BANK_SIZE * prgBankCount;
			prgMemory.resize(totalPrgByteSize);

			// Get data
			ifs.read((char*)prgMemory.data(), totalPrgByteSize);

			// Form vector to hold character data
			if (charBankCount == 0) charBankCount = 2;
			uint32_t totalCharByteSize = CARTRIDGE_CHAR_BANK_SIZE * charBankCount;
			charMemory.resize(totalCharByteSize);

			// Get data
			ifs.read((char*)charMemory.data(), totalCharByteSize);

		}

		if (fileType == 2) {
			// TODO
		}
		
		// Load correct mapper
 		switch (mapperId) {
		case 0:
			this->mapper = std::make_shared<Mapper000>(prgBankCount, charBankCount);
			break;
		}

		ifs.close();

	}



}

Cartridge::~Cartridge()
{
}

bool Cartridge::cpuRead(uint16_t addr, uint8_t& data)
{
	uint32_t mapped_addr = 0x0;
	if (this->mapper->cpuMapRead(addr, mapped_addr)) {
		data = this->prgMemory[mapped_addr];
		return true;
	}
	return false;
}

bool Cartridge::cpuWrite(uint16_t addr, uint8_t data)
{
	uint32_t mapped_addr = 0x0;
	if (this->mapper->cpuMapWrite(addr, mapped_addr)) {
		this->prgMemory[mapped_addr] = data;
		return true;
	}
	return false;
}

bool Cartridge::ppuRead(uint16_t addr, uint8_t& data)
{
	uint32_t mapped_addr = 0x0;
	if (this->mapper->ppuMapRead(addr, mapped_addr)) {
		data = this->charMemory[mapped_addr];
		return true;
	}
	return false;
}

bool Cartridge::ppuWrite(uint16_t addr, uint8_t data)
{
	uint32_t mapped_addr = 0x0;
	if (this->mapper->ppuMapWrite(addr, mapped_addr)) {
		this->charMemory[mapped_addr] = data;
		return true;
	}
	return false;
}
