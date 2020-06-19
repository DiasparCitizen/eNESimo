/*
Copyright (C) 2020 Ismael García-Marlowe

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#include "Cartridge.h"
#include <fstream>
#include "Mapper000.h"

Cartridge::Cartridge(const std::string& cartridgeFileName) {

    ines_header_st inesHeader;

    std::ifstream ifs;
    ifs.open(cartridgeFileName, std::ifstream::binary);

    if (ifs.is_open()) {

        // Just read the header for now
        ifs.read((char*)&inesHeader, sizeof(ines_header_st));

        // Cache the header
        _cartridgeHeader = inesHeader;

        // Apparently, trainer info is sort of junk.
        // No need to read it or understand it, for now.
        if (inesHeader.has512ByteTrainer) {
            ifs.seekg(512); // Skip 512 bytes
        }

        // Construct mapper id
        mapperId = (inesHeader.mapperNumberUpperNibble << 4) | (inesHeader.mapperNumberLowerNibble);

        // Scroll type
        _vertical = inesHeader.mirroring; // Actually more complex, but good for now

        // File type; assume 1 for now
        uint8_t fileType = 1;

        if (fileType == 0) {

        }

        if (fileType == 1) {

            prgBankCount = inesHeader.prgRom16KiBChunkCount;
            charBankCount = inesHeader.chrRom8KiBChunkCount;

            // Form vector to hold program data
            uint32_t totalPrgByteSize = CARTRIDGE_PRG_BANK_SIZE * prgBankCount;
            _programRom.resize(totalPrgByteSize);

            // Get data
            ifs.read((char*)_programRom.data(), totalPrgByteSize);

            // Form vector to hold character data
            if (charBankCount == 0) charBankCount = 2;
            uint32_t totalCharByteSize = CARTRIDGE_CHAR_BANK_SIZE * charBankCount;
            _characterRom.resize(totalCharByteSize);

            // Get data
            ifs.read((char*)_characterRom.data(), totalCharByteSize);

        }

        if (fileType == 2) {
            // TODO
        }

        // Load correct mapper
        switch (mapperId) {
        case 0:
            this->_mapper = std::make_shared<Mapper000>(prgBankCount, charBankCount);
            break;
        }

        ifs.close();

    }



}

Cartridge::~Cartridge() {}

bool Cartridge::cpuRead(uint16_t addr, uint8_t& data) {
    uint32_t mapped_addr = 0x0;
    if (this->_mapper->cpuMapRead(addr, mapped_addr)) {
        data = this->_programRom[mapped_addr];
        return true;
    }
    return false;
}

bool Cartridge::cpuWrite(uint16_t addr, uint8_t data) {
    // Program memory in the cartridge should NOT be writable!
    return false;
}

bool Cartridge::ppuRead(uint16_t addr, uint8_t& data) {
    uint32_t mapped_addr = 0x0;
    if (this->_mapper->ppuMapRead(addr, mapped_addr)) {
        data = this->_characterRom[mapped_addr];
        return true;
    }
    return false;
}

bool Cartridge::ppuWrite(uint16_t addr, uint8_t data) {
    uint32_t mapped_addr = 0x0;
    if (this->_mapper->ppuMapWrite(addr, mapped_addr)) {
        this->_characterRom[mapped_addr] = data;
        return true;
    }
    return false;
}
