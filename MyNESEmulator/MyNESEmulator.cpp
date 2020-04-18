// MyNESEmulator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <sstream>
#include "Bus.h"
#include <Windows.h>
#include <conio.h>
#include <string>

void loadCustomProgram(Bus& nes);

void initializeNes(Bus& nes);

#ifdef CPU_DEBUG_MODE
int main()
#else
int main_()
#endif
{
    std::cout << "Hello World!\n";
	uint16_t b = (0 == 0x00) & 0x1;
	std::cout << b << std::endl;


    Bus nes;
	loadCustomProgram(nes);


	nes._cpu.reset();
	nes._cpu.printCpuState();
	//nes.printPrgMemRange(CPU_DEBUG_MODE_START_PC, CPU_DEBUG_MODE_START_PC+25);

	bool freerun = false;
	uint16_t breakpoint = 0xffff;
	bool breakpoint_hit = false;
	
	std::string userInput;
	char c;
	while (true) {

		if (_kbhit()) {
			switch ((c = _getch())) {
			case 'a':
				std::cout << "advance\n";
				nes._cpu.advanceClock();
				while (!nes._cpu.isInstructionComplete())
					nes._cpu.advanceClock();
				if (nes._cpu.isInstructionComplete()) {
					nes._cpu.printCpuState();
				}
				breakpoint_hit = false;
				break;
			case 'm':
			{
				std::cout << "Read memory positions" << std::endl;
				std::getline(std::cin, userInput);
				uint16_t addr = std::stoi(userInput);
				std::getline(std::cin, userInput);
				uint16_t len = std::stoi(userInput);
				std::cout << "Read " << (uint32_t)len << " bytes from addr: " << (uint32_t)addr << std::endl;
				uint16_t lastAddr = addr + len - 1;
				nes.printRamRange(addr, lastAddr);
			}
			break;
			case 'p':
				nes._cpu.printCpuState();
				break;
			case 'r':
				std::cout << "RUN!\n";
				if (freerun) freerun = false;
				else freerun = true;
				std::cout << freerun << "\n";
				break;
			case 's':
				breakpoint_hit = false;
				std::cout << "RESET\n";
				nes._cpu.reset();
			}
		}
		
		if (freerun) {
			nes._cpu.advanceClock();
		}

		if (!breakpoint_hit && nes._cpu.pc == breakpoint) {
			std::cout << "BREAKPOINT hit!";
			breakpoint_hit = true;
			freerun = false;
		}

	}

}

void loadCustomProgram(Bus& nes) {
	std::ifstream ifs;
	ifs.open("nestest.bin", std::ifstream::binary);

	ifs.read((char*)nes.cpuDebugPrgMem.data(), (65536));

	nes.resetNES();
}

void initializeNes(Bus& nes) {

	// Load Program (assembled at https://www.masswerk.at/6502/assembler.html)
			/*
				*=$8000
				LDX #10
				STX $0000
				LDX #3
				STX $0001
				LDY $0000
				LDA #0
				CLC
				loop
				ADC $0001
				DEY
				BNE loop
				STA $0002
				NOP
				NOP
				NOP
			*/

	// Convert hex string into bytes for RAM
	std::stringstream ss;
	ss << "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA";
	uint16_t nOffset = 0x8000;
	while (!ss.eof())
	{
		std::string b;
		ss >> b;
		nes._cpuRam[nOffset++] = (uint8_t)std::stoul(b, nullptr, 16);
	}

	// Set Reset Vector
	nes._cpuRam[RESET_ADDR] = 0x00;
	nes._cpuRam[RESET_ADDR+1] = 0x80;


}

