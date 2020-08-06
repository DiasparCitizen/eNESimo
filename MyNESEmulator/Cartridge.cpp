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
#include "Mapper002.h"
#include "Mapper001.h"
#include "Mapper003.h"

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
        _mapperId = (inesHeader.mapperNumberUpperNibble << 4) | (inesHeader.mapperNumberLowerNibble);

        // Scroll type
        _mirroringType = inesHeader.mirroring ? MIRRORING_TYPE::V : MIRRORING_TYPE::H;

        // File type; assume 1 for now
        uint8_t fileType = 1;

        if (fileType == 0) {

        }

        if (fileType == 1) {

            _prgBankCount = inesHeader.prgRom16KiBChunkCount;
            _charBankCount = inesHeader.chrRom8KiBChunkCount;

            // Form vector to hold program data
            uint32_t totalPrgByteSize = CARTRIDGE_PRG_BANK_SIZE * _prgBankCount;
            _programRom.resize(totalPrgByteSize);

            // Get data
            ifs.read((char*)_programRom.data(), totalPrgByteSize);

            // Form vector to hold character data
            uint32_t totalCharByteSize;
            if (_charBankCount == 0) {
                totalCharByteSize = CARTRIDGE_CHAR_BANK_SIZE;
            }
            else {
                totalCharByteSize = CARTRIDGE_CHAR_BANK_SIZE * _charBankCount;
            }

            _characterRom.resize(totalCharByteSize);

            // Get data
            ifs.read((char*)_characterRom.data(), totalCharByteSize);

        }

        if (fileType == 2) {
            // TODO
        }

        _cartridgeRam.resize(0x2000);

        // Load correct mapper
        switch (_mapperId) {
        case 0:
            this->_mapper = std::make_shared<Mapper000>(_prgBankCount, _charBankCount);
            break;
        case 1:
            this->_mapper = std::make_shared<Mapper001>(_prgBankCount, _charBankCount);
            break;
        case 2:
            this->_mapper = std::make_shared<Mapper002>(_prgBankCount, _charBankCount);
            break;
        case 3:
            this->_mapper = std::make_shared<Mapper003>(_prgBankCount, _charBankCount);
        }

        ifs.close();

    }

}

Cartridge::~Cartridge() {}

bool Cartridge::cpuRead(uint16_t addr, uint8_t& data) {
    uint32_t mappedAddr = 0;
    MEM_MODULE module = _mapper->mapCpuRead(addr, mappedAddr);
    internalRead(mappedAddr, data, module);
    return module != MEM_MODULE::INVALID;
}

bool Cartridge::cpuWrite(uint16_t addr, uint8_t data) {
    uint32_t mappedAddr = 0;
    MEM_MODULE module = _mapper->mapCpuWrite(addr, mappedAddr, data);
    internalWrite(mappedAddr, data, module);
    return module != MEM_MODULE::INVALID;
}

bool Cartridge::ppuRead(uint16_t addr, uint8_t& data) {
    uint32_t mappedAddr = 0;
    MEM_MODULE module = _mapper->mapPpuRead(addr, mappedAddr);
    internalRead(mappedAddr, data, module);
    return module != MEM_MODULE::INVALID;
}

bool Cartridge::ppuWrite(uint16_t addr, uint8_t data) {
    uint32_t mappedAddr = 0;
    MEM_MODULE module = _mapper->mapPpuWrite(addr, mappedAddr);
    internalWrite(mappedAddr, data, module);
    return module != MEM_MODULE::INVALID;
}

bool Cartridge::internalRead(uint32_t addr, uint8_t& data, MEM_MODULE module) {
    switch (module) {
    case MEM_MODULE::PRG_RAM:
        data = _cartridgeRam[addr];
        break;
    case MEM_MODULE::PRG_ROM:
        data = _programRom[addr];
        break;
    case MEM_MODULE::CHR_ROM:
        data = _characterRom[addr];
        break;
    default:
        unmappableReadCnt++;
    }
    return true;
}

bool Cartridge::internalWrite(uint32_t addr, uint8_t data, MEM_MODULE module) {
    switch (module) {
    case MEM_MODULE::PRG_RAM:
        _cartridgeRam[addr] = data;
        break;
    case MEM_MODULE::PRG_ROM:
        break;
    case MEM_MODULE::CHR_ROM:
        _characterRom[addr] = data;
        break;
    default:
        unmappableWriteCnt++;
    }
    return true;
}

MIRRORING_TYPE Cartridge::getMirroringType() {

    if (_mapper->getMirroringType() == MIRRORING_TYPE::STATIC) {
        return _mirroringType;
    }
    else {
        return _mapper->getMirroringType();
    }

}

void Cartridge::reset() {
    if (_mapper != nullptr) {
        _mapper->reset();
    }
}