
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

	// Initialize RAM
	for (auto idx = 0; idx < CPU_ADDR_SPACE_RAM_SIZE; idx++)
		_cpuRam[idx] = 0x00;
	
	// Connect cpu to itself
	_cpu.attachBus(this);

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
	this->_cartridge = cartridge;
	this->_ppu.connectConsole(this);
	this->_ppu.connectCartridge(cartridge);
}

void Bus::resetNES()
{
	_cpu.reset(); // Reset CPU
	_ppu.reset();
	
	_systemControlCounter = 0;
}

void Bus::clockNES()
{
	// The PPU is the fastest

	_ppu.clock();
	_ppu.clock();
	_ppu.clock();

	if (_dmaControl.dmaState == DMA_STATE_IDLE) {

		_cpu.advanceClock();
		if (_cpu._justFetched) {
			//_LOG(getNESStateAsStr(this));
			//this->_ppu.ppuLogFile << getNESStateAsStr(this);
		}

	} else {

		if (_dmaControl.dmaState == DMA_STATE_TRANSFER_SCHEDULED) {
			if (_systemControlCounter & 0x1 /* ODD */) {
				_dmaControl.dmaState = DMA_STATE_DUMMY_READ; // Wait another cycle
			}
			else { // EVEN
				_dmaControl.dmaState = DMA_STATE_TRANSFERRING;
			}
		}
		else if (_dmaControl.dmaState == DMA_STATE_DUMMY_READ) {
			_dmaControl.dmaState = DMA_STATE_TRANSFERRING;
		}
		if (_dmaControl.dmaState == DMA_STATE_TRANSFERRING) {
			if (_systemControlCounter & 0x1 /* ODD */) {
				//_LOG( "DMA OAM write: 0x" << std::hex << (uint16_t)_dmaControl.data << " @ 0x" << (uint16_t)_dmaControl.dmaDstAddr << std::endl);
				_ppu._oamMem.raw[_dmaControl.dmaDstAddr++] = _dmaControl.data;
				if (_dmaControl.dmaDstAddr == PPU_OAM_SIZE) {
					_dmaControl.dmaState = DMA_STATE_IDLE;
				}
			}
			else { // EVEN
				// On even cycles, read next data to write
				_dmaControl.data = _cpuRam[_dmaControl.dmaSrcAddr];
				//_LOG("Set DMA OAM data: 0x" << std::hex << (uint16_t)_dmaControl.data << " <-- source 0x" << (uint16_t)_dmaControl.dmaSrcAddr << std::endl);
				_dmaControl.dmaSrcAddr++;
			}
		}

	}

	if (_cpu._nmiOccurred) {
		_cpu.nmi();
		_cpu._nmiOccurred = false;
	}

	_systemControlCounter++;

}

void Bus::cpuWrite(uint16_t addr, uint8_t data) {
	//std::cout << "write 0x: " << std::hex << data << " @ " << std::hex << addr << std::endl;
#ifndef  CPU_DEBUG_MODE
	if (this->_cartridge->cpuWrite(addr, data)) {
	
	}
	else if (_IS_RAM_ADDR(addr)) {
		_cpuRam[addr & CPU_ADDR_SPACE_RAM_MIRROR_MASK] = data;
	}
	else if (_IS_PPU_ADDR(addr)) {
		_ppu.cpuWrite(CPU_ADDR_SPACE_PPU_START + (addr & CPU_ADDR_SPACE_PPU_MIRROR_MASK), data);
	}
	else if (addr == CPU_ADDR_SPACE_OAM_DMA) {
		// Switch on DMA
		_dmaControl.dmaState = DMA_STATE_TRANSFER_SCHEDULED;
		_dmaControl.dmaSrcAddr = (data & CPU_RAM_PAGE_ID_MASK /* Protect */ ) * CPU_RAM_PAGE_SIZE; // Start read offset = page_id * 256b
		_dmaControl.dmaDstAddr = 0x0000;
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_1) {
		_controllers[1].cpuWrite(data);
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_2) {
		_controllers[0].cpuWrite(data);
	}
	else {
		//std::cout << "OUT OF RANGE write 0x" << std::hex << data << " @ " << std::hex << addr << std::endl;
	}
#else
	cpuDebugPrgMem[addr] = data;
#endif // ! DEBUG_CPU_MODE

}

uint8_t Bus::cpuRead(uint16_t addr, bool bReadOnly) {

	uint8_t readData = RAM_TRASH_VALUE;

#ifndef  CPU_DEBUG_MODE
	if (this->_cartridge->cpuRead(addr, readData)) {

	}
	else if (_IS_RAM_ADDR(addr)) {
		readData = _cpuRam[addr & CPU_ADDR_SPACE_RAM_MIRROR_MASK];
	}
	else if (_IS_PPU_ADDR(addr)) {
		readData = _ppu.cpuRead(CPU_ADDR_SPACE_PPU_START + (addr & CPU_ADDR_SPACE_PPU_MIRROR_MASK), false);
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_1) {
		readData = _controllers[1].cpuRead();
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_2) {
		readData = _controllers[0].cpuRead();
	}
	else {
		//std::cout << "OUT OF RANGE read from 0x: " << std::hex << addr << std::endl;
	}
#else
	readData = cpuDebugPrgMem[addr];
#endif // ! DEBUG_CPU_MODE

	return readData;
	
}

/************************************/
/************  DEBUG  ***************/
/************************************/

void Bus::printBufferRange(uint16_t startAddr, uint16_t endAddr, uint8_t* buffer) {
//#ifdef  CPU_DEBUG_MODE
	uint8_t datum = 0;
	std::cout << "***** BUFFER *****" << std::endl;
	for (uint16_t idx = startAddr; idx < endAddr; idx++) {
		datum = buffer[idx];
		std::cout << "@0x" << std::hex << idx << ": 0x" << std::hex << (uint16_t)datum << std::endl;
	}
//#endif
}

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

	debug_ppu_state_dsc_st ppuState = bus->_ppu.getDebugPPUstate();
	debug_cpu_state_dsc_st cpuState = bus->_cpu.getDebugCPUState();

	std::stringstream myStream;
	myStream << cpuState.pre_instruction_counter << "  ";
	myStream << std::uppercase << std::hex << (uint16_t)cpuState.pre_pc << "  ";
	myStream << std::dec << (uint16_t)cpuState.opcode << ":";
	myStream << cpuState.inst_name << "-";
	myStream << bus->_cpu.getAddrMode(cpuState.opcode) << "(";
	myStream << std::dec << (uint16_t)cpuState.cycles << "+" << (uint16_t)cpuState.extra_cycles << ") ";
	myStream << "$" << std::hex << (uint16_t)cpuState.nxt_inst;
	myStream << std::hex << (uint16_t)cpuState.nxt_nxt_inst << "                        ";
	myStream << "A:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_reg_acc << " ";
	myStream << "X:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_reg_x << " ";
	myStream << "Y:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_reg_y << " ";
	myStream << "P:" << std::hex << (uint16_t)cpuState.pre_reg_status << " ";
	myStream << "SP:" << std::hex << (uint16_t)cpuState.pre_stack_ptr << " ";
	myStream << "CYC:" << std::dec << (int16_t)ppuState.scanlineDot << " ";
	myStream << "SL:" << std::dec << (int16_t)ppuState.scanline << " ";
	myStream << "FC:" << std::dec << (int64_t)ppuState.frameCounter << " ";
	myStream << "CPUCycle:" << std::dec << cpuState.cpu_cycle << " ";
	myStream << "STA:" << std::hex << (uint16_t)ppuState.statusReg.raw << " ";
	myStream << "MSK:" << std::hex << (uint16_t)ppuState.maskReg.raw << " ";
	myStream << std::endl;

	return myStream.str();

}