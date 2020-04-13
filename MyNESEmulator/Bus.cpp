
#include "Bus.h"
#include "NESConstants.h"
#include <iostream>
#include  <iomanip>

#define _IS_RAM_ADDR(addr) (addr >= CPU_ADDR_SPACE_RAM_START && addr <= CPU_ADDR_SPACE_RAM_END)
#define _IS_PPU_ADDR(addr) (addr >= CPU_ADDR_SPACE_PPU_START && addr <= CPU_ADDR_SPACE_PPU_END)

#if defined(BUS_TERMINAL_LOG) && defined(BUS_FILE_LOG)
#define _LOG(txt) \
{ \
std::stringstream myStream; \
myStream << txt; \
std::cout << myStream.str(); \
busLogFile << myStream.str(); \
} 
#elif defined(BUS_TERMINAL_LOG)
#define _LOG(txt) std::cout << txt
#elif defined(BUS_FILE_LOG)
#define _LOG(txt) busLogFile << txt
#else
#define _LOG(txt) ;
#endif

// Constructor
Bus::Bus() {

	for (auto idx = 0; idx < CPU_ADDR_SPACE_RAM_SIZE; idx++)
		cpuRam[idx] = 0x00;
	
	// Connect cpu to itself
	cpu.attachBus(this);

#ifdef BUS_FILE_LOG
	busLogFile.open("bus_log.txt");
#endif

}

Bus::~Bus()
{
#ifdef BUS_FILE_LOG
	busLogFile.close();
#endif
}

void Bus::insertCartridge(const std::shared_ptr<Cartridge>& cartridge)
{
	this->cartridge = cartridge;
	this->ppu.connectCartridge(cartridge);
}

void Bus::resetNES()
{
	cpu.reset(); // Reset CPU
	ppu.reset();
	
	systemClockCounter = 0;
}

void Bus::clockNES()
{
	// The PPU is the fastest

	this->ppu.advanceClock();
	this->ppu.advanceClock();
	this->ppu.advanceClock();

	this->cpu.advanceClock();
	if (this->cpu.justFetched) {
		_LOG(getNESStateAsStr(this));
	}

	if (this->ppu.nmi) {
		this->ppu.nmi = false;
		this->cpu.nmi();
	}

	systemClockCounter++;
}

void Bus::cpuWrite(uint16_t addr, uint8_t data) {

#ifndef  CPU_DEBUG_MODE
	if (this->cartridge->cpuWrite(addr, data)) {
	
	}
	else if (_IS_RAM_ADDR(addr)) {
		cpuRam[addr & CPU_ADDR_SPACE_RAM_MIRROR_MASK] = data;
	}
	else if (_IS_PPU_ADDR(addr)) {
		ppu.cpuWrite(CPU_ADDR_SPACE_PPU_START + (addr & CPU_ADDR_SPACE_PPU_MIRROR_MASK), data);
	}
#else
	cpuDebugPrgMem[addr] = data;
#endif // ! DEBUG_CPU_MODE

}

uint8_t Bus::cpuRead(uint16_t addr, bool bReadOnly) {

	uint8_t readData = RAM_TRASH_VALUE;

#ifndef  CPU_DEBUG_MODE
	if (this->cartridge->cpuRead(addr, readData)) {

	}
	else if (_IS_RAM_ADDR(addr)) {
		readData = cpuRam[addr & CPU_ADDR_SPACE_RAM_MIRROR_MASK];
	}
	else if (_IS_PPU_ADDR(addr)) {
		readData = ppu.cpuRead(CPU_ADDR_SPACE_PPU_START + (addr & CPU_ADDR_SPACE_PPU_MIRROR_MASK), false);
	}
#else
	readData = cpuDebugPrgMem[addr];
#endif // ! DEBUG_CPU_MODE

	return readData;
	
}

/************************************/
/************  DEBUG  ***************/
/************************************/

void Bus::printRamRange(uint16_t startAddr, uint16_t endAddr) {
#ifdef  CPU_DEBUG_MODE
	uint8_t datum = 0;
	std::cout << "***** RAM *****" << std::endl;
	for (uint16_t idx = startAddr; idx < endAddr; idx++) {
		datum = cpuDebugPrgMem[idx];
		std::cout << "@0x" << std::hex << idx << ": 0x" << std::hex << (uint16_t)datum << std::endl;
	}
#endif
}

void Bus::printPrgMemRange(uint16_t startAddr, uint16_t endAddr) {
#ifdef  CPU_DEBUG_MODE
	uint8_t datum = 0;
	std::cout << "***** RAM *****" << std::endl;
	uint16_t currAddr = startAddr;
	while (currAddr < endAddr){
		datum = cpuDebugPrgMem[currAddr];
		std::string middle = datum < 0x10 ? "0" : "";
		std::cout << "@0x" << std::hex << currAddr << ": 0x" << middle << std::hex << (uint16_t)datum << " " << nesCPU.instruction_lut[datum].name << " +" + std::to_string(nesCPU.instruction_lut[datum].cycles) << std::endl;
		currAddr++;

	}
#endif
}

std::string getNESStateAsStr(Bus* bus)
{

	debug_ppu_state_dsc_st ppuState = bus->ppu.getDebugPPUstate();
	debug_cpu_state_dsc_st cpuState = bus->cpu.getDebugCPUState();

	std::stringstream myStream;
	myStream << cpuState.pre_instruction_counter << "  ";
	myStream << std::uppercase << std::hex << (uint16_t)cpuState.pre_pc << "  ";
	myStream << std::dec << (uint16_t)cpuState.opcode << ":";
	myStream << cpuState.inst_name << "-";
	myStream << bus->cpu.getAddrMode(cpuState.opcode) << "(";
	myStream << std::dec << (uint16_t)cpuState.cycles << "+" << (uint16_t)cpuState.extra_cycles << ") ";
	myStream << "$" << std::hex << (uint16_t)cpuState.nxt_inst;
	myStream << std::hex << (uint16_t)cpuState.nxt_nxt_inst << "                        ";
	myStream << "A:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_reg_acc << " ";
	myStream << "X:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_reg_x << " ";
	myStream << "Y:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_reg_y << " ";
	myStream << "P:" << std::hex << (uint16_t)cpuState.pre_reg_status << " ";
	myStream << "SP:" << std::hex << (uint16_t)cpuState.pre_stack_ptr << " ";
	myStream << "CYC:" << std::dec << (int16_t)ppuState.scanline_dot << " ";
	myStream << "SL:" << std::dec << (int16_t)ppuState.scanline << " ";
	myStream << "FC:" << std::dec << (int64_t)ppuState.frame_counter << " ";
	myStream << "CPUCycle:" << std::dec << cpuState.cpu_cycle << " ";
	myStream << "STA:" << std::hex << (uint16_t)ppuState.status_reg.raw << " ";
	myStream << std::endl;

	return myStream.str();

}