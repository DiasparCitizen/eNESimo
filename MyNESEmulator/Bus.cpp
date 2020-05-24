
#include "Bus.h"
#include  <iomanip>

#define _IS_RAM_ADDR(addr) (addr >= CPU_ADDR_SPACE_RAM_START && addr <= CPU_ADDR_SPACE_RAM_END)
#define _IS_PPU_ADDR(addr) (addr >= CPU_ADDR_SPACE_PPU_START && addr <= CPU_ADDR_SPACE_PPU_END)

#define _IS_APU_ADDR(addr) ( (addr >= APU_ADDR_SPACE_PULSE_1_REG1 && addr <= APU_ADDR_SPACE_SAMPLE_LENGTH) || addr == APU_ADDR_SPACE_FRAME_STATUS || addr == APU_ADDR_SPACE_FRAME_COUNTER )

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
	this->_apu.connectConsole(this);
	this->_ppu.connectCartridge(cartridge);
}

void Bus::resetNES()
{
	_cpu.reset(); // Reset CPU
	_ppu.reset();
	_apu.reset();
	_systemControlCounter = 0;
}

void Bus::clockNES()
{

	_ppu.clock();

	// The CPU is clocked at approx. 1/3 of the PPU's frequency
	if ((_systemControlCounter % 3) == 0) {

		_apu.clock();

		if (_dmaControl.dmaState == DMA_STATE_IDLE) {

			_cpu.advanceClock();
			if (_cpu._justFetched) {
				_LOG(getNESStateAsStr(this));
			}

		}
		else {

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
					_ppu._oamMem.raw[_dmaControl.dmaDstAddr++] = _dmaControl.data;
					if (_dmaControl.dmaDstAddr == PPU_OAM_SIZE) {
						_dmaControl.dmaState = DMA_STATE_IDLE;
					}
				}
				else { // EVEN
					// On even cycles, read next data to write
					_dmaControl.data = _cpuRam[_dmaControl.dmaSrcAddr];
					_dmaControl.dmaSrcAddr++;
				}
			}

		}

		_accumulatedTime += NES_CYCLE_PERIOD;
		_globalTime += NES_CYCLE_PERIOD;

		// Now, sample APU output if it's time
		while (_accumulatedTime > SAMPLE_PERIOD && _nextSampleIdx < 10) {
			_smallSampleBuffer[_nextSampleIdx].sample = 0xff; // Some sample
			_smallSampleBuffer[_nextSampleIdx].time = _globalTime;
			_nextSampleIdx++;
			_accumulatedTime -= SAMPLE_PERIOD;
		}

		if (_cpu._nmiOccurred) {
			_cpu.nmi();
			_cpu._nmiOccurred = false;
		}

		_apu.clock();

		if (_apu._frameInterruptFlag) {
			_apu._frameInterruptFlag = false;
			_cpu.irq();
		}

	}

	_systemControlCounter++;

}

void Bus::cpuWrite(uint16_t addr, uint8_t data) {

#ifndef  CPU_DEBUG_MODE
	if (this->_cartridge->cpuWrite(addr, data)) {
	
	}
	else if (_IS_RAM_ADDR(addr)) {
		_cpuRam[addr & CPU_ADDR_SPACE_RAM_MIRROR_MASK] = data;
	}
	else if (_IS_PPU_ADDR(addr)) {

		switch (addr &= CPU_ADDR_SPACE_PPU_MIRROR_MASK) {
		case 0: _ppu.writeControlReg(data); break;
		case 1: _ppu.writeMaskReg(data); break;
		case 2: _ppu.writeStatusReg(data); break;
		case 3: _ppu.writeSpriteMemAddr(data); break;
		case 4: _ppu.writeSpriteMemData(data); break;
		case 5: _ppu.writeBgScroll(data); break;
		case 6: _ppu.writeVramAddr(data); break;
		case 7: _ppu.writeVramData(data); break;
		}

	}
	else if (addr == CPU_ADDR_SPACE_OAM_DMA) {
		// Switch on DMA
		_dmaControl.dmaState = DMA_STATE_TRANSFER_SCHEDULED;
		_dmaControl.dmaSrcAddr = (data & CPU_RAM_PAGE_ID_MASK /* Protect */ ) * CPU_RAM_PAGE_SIZE; // Start read offset = page_id * 256b
		_dmaControl.dmaDstAddr = 0x0000;
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_1) {
		_controllers[0].cpuWrite(data);
		_controllers[1].cpuWrite(data);
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_2) {
		// read-only
	}
	else if (_IS_APU_ADDR(addr)) {

		switch (addr) {

		case APU_ADDR_SPACE_PULSE_1_REG1: _apu.writePulseWave1Reg1(data); break;
		case APU_ADDR_SPACE_PULSE_1_REG2: _apu.writePulseWave1Reg2(data); break;
		case APU_ADDR_SPACE_PULSE_1_REG3: _apu.writePulseWave1Reg3(data); break;
		case APU_ADDR_SPACE_PULSE_1_REG4: _apu.writePulseWave1Reg4(data); break;

		case APU_ADDR_SPACE_PULSE_2_REG1: _apu.writePulseWave2Reg1(data); break;
		case APU_ADDR_SPACE_PULSE_2_REG2: _apu.writePulseWave2Reg2(data); break;
		case APU_ADDR_SPACE_PULSE_2_REG3: _apu.writePulseWave2Reg3(data); break;
		case APU_ADDR_SPACE_PULSE_2_REG4: _apu.writePulseWave2Reg4(data); break;

		case APU_ADDR_SPACE_FRAME_STATUS: _apu.writeStatusReg(data); break;
		case APU_ADDR_SPACE_FRAME_COUNTER: _apu.writeFrameCounterReg(data); break;
		}
	}
	else {
		std::cout << "OUT OF RANGE write 0x" << std::hex << data << " @ " << std::hex << addr << std::endl;
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

		switch (addr &= CPU_ADDR_SPACE_PPU_MIRROR_MASK) {
		case 0: readData = _ppu.readControlReg(); break;
		case 1: readData = _ppu.readMaskReg(); break;
		case 2: readData = _ppu.readStatusReg(); break;
		case 3: readData = _ppu.readSpriteMemAddr(); break;
		case 4: readData = _ppu.readSpriteMemData(); break;
		case 5: readData = _ppu.readBgScroll(); break;
		case 6: readData = _ppu.readVramAddr(); break;
		case 7: readData = _ppu.readVramData(); break;
		}

	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_1) {
		readData = _controllers[0].cpuRead();
	}
	else if (addr == CPU_ADDR_SPACE_CONTROLLER_2) {
		readData = _controllers[1].cpuRead();
	}
	else if (_IS_APU_ADDR(addr)) {
		if (addr == APU_ADDR_SPACE_FRAME_STATUS) {
			readData = _apu.readStatusReg();
		}
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

uint8_t* Bus::getFrameBuffer() {
	return _ppu.getFrameBuffer();
}

pixel_st* Bus::getPtrToLastPixelDrawn() {
	return _ppu.getPtrToLastPixelDrawn();
}

bool Bus::areNewSamplesAvailable() {
	return _nextSampleIdx > 0;
}

sample_st* Bus::getPtrToNewSamples(uint16_t& numSamples) {
	numSamples = _nextSampleIdx;
	_nextSampleIdx = 0; // Reset to 0, since all new samples were consumed
	return _smallSampleBuffer;
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

std::string getNESStateAsStr(Bus* bus) {

	debug_ppu_state_dsc_st ppuState = bus->_ppu.getDebugPPUstate();
	debug_cpu_state_dsc_st cpuState = bus->_cpu.getDebugCPUState();

	std::stringstream myStream;
	myStream << cpuState.instructionCounter << "  ";
	myStream << std::uppercase << std::hex << (uint16_t)cpuState.pre_pc << "  ";
	myStream << std::dec << (uint16_t)cpuState.opcode << ":";
	myStream << cpuState.instructionName << "-";
	myStream << bus->_cpu.getAddrMode(cpuState.opcode) << "(";
	myStream << std::dec << (uint16_t)cpuState.cycles << "+" << (uint16_t)cpuState.extraCycles << ") ";
	myStream << "$" << std::hex << (uint16_t)cpuState.pcPlus1;
	myStream << std::hex << (uint16_t)cpuState.pcPlus2 << "                        ";
	myStream << "A:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_acc << " ";
	myStream << "X:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_x << " ";
	myStream << "Y:" << std::setfill('0') << std::setw(2) << std::right << std::hex << (uint16_t)cpuState.pre_y << " ";
	myStream << "P:" << std::hex << (uint16_t)cpuState.pre_status << " ";
	myStream << "SP:" << std::hex << (uint16_t)cpuState.pre_stackPtr << " ";
	myStream << "CYC:" << std::dec << (int16_t)ppuState.scanlineCycle << " ";
	myStream << "SL:" << std::dec << (int16_t)ppuState.scanline << " ";
	myStream << "FC:" << std::dec << (int64_t)ppuState.frameCounter << " ";
	myStream << "CPUCycle:" << std::dec << cpuState.cpuCycleCounter << " ";
	myStream << "STA:" << std::hex << (uint16_t)ppuState.statusReg.raw << " ";
	myStream << "MSK:" << std::hex << (uint16_t)ppuState.maskReg.raw << " ";
	myStream << std::endl;

	return myStream.str();

}