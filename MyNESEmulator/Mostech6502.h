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
#pragma once

#include "CommonLibs.h"
#include "NESConstants.h"

#include <vector>
#include <map>
#include <fstream>

union reg_status_t {
    struct {
        uint8_t C : 1; // Carry
        uint8_t Z : 1; // Zero
        uint8_t I : 1; // Interrupt
        uint8_t D : 1; // Decimal
        uint8_t B : 1; // Break
        uint8_t reserved : 1;
        uint8_t V : 1; // Overflow
        uint8_t S : 1; // Signed (negative)
    };
    uint8_t raw;
};

struct debug_cpu_state_dsc_st {

    uint64_t instructionCounter;
    uint64_t cpuCycleCounter;

    uint16_t pre_pc;
    uint8_t pre_x;
    uint8_t pre_y;
    uint8_t pre_acc;
    uint8_t pre_status;
    uint8_t pre_stackPtr;

    uint16_t post_pc;
    uint8_t post_x;
    uint8_t post_y;
    uint8_t post_acc;
    uint8_t post_status;
    uint8_t post_stackPtr;

    std::string instructionName;
    uint8_t opcode;
    uint8_t cycles;
    uint8_t extraCycles;
    uint8_t pcPlus1;
    uint8_t pcPlus2;

};

// https://wiki.nesdev.com/w/index.php/CPU_interrupts
class NMI {

private:
    uint8_t nmiLevel;
    bool nmiSignal;

public:
    NMI() {
        nmiLevel = 1;
        nmiSignal = false;
    }

    void setNMILevel(uint8_t newLevel) {
        // The NMI input is edge-sensitive (reacts to high-to-low transitions in the signal).
        // The NMI input is connected to an edge detector.
        nmiSignal = nmiLevel == 1 && newLevel == 0;
        nmiLevel = newLevel;
    }

    void setNMIHigh() {
        nmiSignal = false;
        nmiLevel = 1;
    }

    void setNMILow() {
        // Set signal only on high-to-low transition
        nmiSignal = nmiLevel == 1;
        nmiLevel = 0;
    }

    bool getNMISignal() {
        return nmiSignal;
    }

};

class IRQ {

private:
    uint8_t irqLevel;
    bool irqSignal;

public:
    void setIRQLevel(uint8_t newLevel) {
        // The IRQ input is connected to a level detector
        irqLevel = newLevel;
        irqSignal = irqLevel == 0;
    }

    void setIRQHigh() {
        irqLevel = 0;
        irqSignal = false;
    }

    void setIRQLow() {
        irqLevel = 1;
        irqSignal = true;
    }

    bool getIRQSignal() {
        return irqSignal;
    }

};

// Forward declaration of Bus to prevent circular inclusions
class Bus;
class Mostech6502;

struct inst {
    char name[INSTRUCTION_CHAR_LEN];
    uint8_t(Mostech6502::* op)(void) = nullptr;
    uint8_t(Mostech6502::* addr_mode)(void) = nullptr;
    uint8_t cycles;
};

#define _DEBUG_FILL_PRE_CPU_STATE() \
	_debugCPUState.cpuCycleCounter = _cpuCycleCounter; \
	_debugCPUState.instructionCounter = _instructionCounter; \
	_debugCPUState.pre_pc = _pc; \
	_debugCPUState.pre_x = _x; \
	_debugCPUState.pre_y = _y; \
	_debugCPUState.pre_acc = _acc; \
	_debugCPUState.pre_status = _status.raw; \
	_debugCPUState.pre_stackPtr = _stackPtr

#define _DEBUG_FILL_POST_CPU_STATE(argInstructionName, argCycles, argExtraCycles, argPcPlus1, argPcPlus2) \
	_debugCPUState.post_pc = _pc; \
	_debugCPUState.post_x = _x; \
	_debugCPUState.post_y = _y; \
	_debugCPUState.post_acc = _acc; \
	_debugCPUState.post_status = _status.raw; \
	_debugCPUState.post_stackPtr = _stackPtr; \
	_debugCPUState.opcode = _opcode; \
	_debugCPUState.cycles = argCycles; \
	_debugCPUState.extraCycles = argExtraCycles; \
	_debugCPUState.instructionName = std::string(argInstructionName); \
	_debugCPUState.pcPlus1 = argPcPlus1; \
	_debugCPUState.pcPlus2 = argPcPlus2

#define _FETCH() if ( Mostech6502::_instructionLut[Mostech6502::_opcode].addr_mode != &Mostech6502::imp ) Mostech6502::_M = Mostech6502::read(Mostech6502::_addrAbs)

#define _STACK_PUSH(value) \
	write(CPU_ADDR_SPACE_STACK_START + _stackPtr, value); \
	_stackPtr--;

#define _STACK_POP(value) \
	_stackPtr++; \
	value = read(CPU_ADDR_SPACE_STACK_START + _stackPtr);

class Mostech6502 {

public:

    Mostech6502();
    ~Mostech6502();

    // The following functions represent pins in the chip
    void reset(); // Reset CPU
    void irq(); // Interrupt request
    void nmi(); // Non-maskable interrupt request
    void advanceClock(); // Advance one clock cycle

    //
    bool isInstructionComplete();

    // Link this CPU to a bus
    void attachBus(Bus* bus);

    // Debug
    void printCpuState();
    debug_cpu_state_dsc_st& getDebugCPUState();
    std::string getPreExecuteStateAsStr();
    std::string getPostExecuteStateAsStr();
    std::string getAddrMode(uint8_t _opcode);

private:

    ///////////////////////////////////////////
    // Addressing modes
    ///////////////////////////////////////////

    uint8_t imm();	// Immediate
    uint8_t imp();	// Implied
    uint8_t rel();	// Relative

    uint8_t abs();	// Absolute
    uint8_t absx();	// Zero-page absolute
    uint8_t absy();	// Zero-page absolute

    uint8_t zpg();	// Zero page
    uint8_t zpgx(); // Zero page x
    uint8_t zpgy(); // Zero page y

    uint8_t ind();	// Indirect
    uint8_t indx();	// Indirect x
    uint8_t indy();	// Indirect y

    uint8_t acc();

    ///////////////////////////////////////////
    ///////////////////////////////////////////

    ///////////////////////////////////////////
    // Instruction set
    ///////////////////////////////////////////

    uint8_t _adc(); // Add memory to accumulator with carry
    uint8_t _and(); // "AND" memory with acc
    uint8_t _asl(); // Shift left one bit (mem or acc)

    uint8_t _bcc(); // Branch on carry clear
    uint8_t _bcs(); // Branch on carry set
    uint8_t _beq(); // Branch on result zero
    uint8_t _bit(); // Test bits in memory with acc
    uint8_t _bmi(); // Branch on result minus
    uint8_t _bne(); // Branch on result not zero
    uint8_t _bpl(); // Branch on result plus
    uint8_t _brk(); // Force break
    uint8_t _bvc(); // Branch on overflow clear
    uint8_t _bvs(); // Branch on overflow set

    uint8_t _clc(); // Clear carry flag
    uint8_t _cld(); // Clear decimal mode
    uint8_t _cli(); // Clear interrupt disable
    uint8_t _clv(); // Clear overflow flag
    uint8_t _cmp(); // Compare memory and acc
    uint8_t _cpx(); // Compare memory and index x
    uint8_t _cpy(); // Compare memory and index y

    uint8_t _dec(); // Decrement memory by one
    uint8_t _dex(); // Decrement index x by one
    uint8_t _dey(); // Decrement index y by one

    uint8_t _eor(); // X-OR memory with acc

    uint8_t _inc(); // Increment memory by one
    uint8_t _inx(); // Increment index x by one
    uint8_t _iny(); // Increment index y by one

    uint8_t _jmp(); // Jump to new location
    uint8_t _jsr(); // Jump to new location saving return address

    uint8_t _lda(); // Load acc with memory 
    uint8_t _ldx(); // Load index x with memory
    uint8_t _ldy(); // Load index y with memory
    uint8_t _lsr(); // Shift right one bit (memory or accumulator)

    uint8_t _nop(); // No operation at all, bietch 

    uint8_t _ora(); // OR memory with acc 

    uint8_t _pha(); // Push acc on stack 
    uint8_t _php(); // Push processor status on stack
    uint8_t _pla(); // Pull acc from stack
    uint8_t _plp(); // Pull processor status from stack

    uint8_t _rol(); // Rotate one bit left (memory or acc)
    uint8_t _ror(); // Rotate one bit right (memory or acc)
    uint8_t _rti(); // Return from interrupt
    uint8_t _rts(); // Return from subroutine

    uint8_t _sbc(); // Subtract memory from accumulator with borrow
    uint8_t _sec(); // Set carry flag
    uint8_t _sed(); // Set decimal mode
    uint8_t _sei(); // Set interrupt disable status
    uint8_t _sta(); // Store acc in memory
    uint8_t _stx(); // Store index x in memory
    uint8_t _sty(); // Store index y in memory

    uint8_t _tax(); // Transfer acc to index x
    uint8_t _tay(); // Transfer acc to index y
    uint8_t _tsx(); // Transfer stack pointer to index x
    uint8_t _txa(); // Transfer index x to acc
    uint8_t _txs(); // Transfer index x to stack pointer 
    uint8_t _tya(); // Transfer index y to acc

    uint8_t _xxx(); // Not available instruction

    ///////////////////////////////////////////
    ///////////////////////////////////////////

private:
    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t data);
    uint8_t fetch(); // Read an instruction

public:
    // CPU internal registers
    uint16_t _pc;
    uint8_t _stackPtr;
    uint8_t _x;
    uint8_t _y;
    uint8_t _acc;
    reg_status_t _status;

    // Helper vars
    uint8_t _M; // Fetched byte, represented as 'M' in proc description in nesdev
    uint8_t _opcode;

    uint64_t _cpuCycleCounter;
    uint16_t _cycles;
    uint64_t _instructionCounter;

    uint16_t _addrAbs; // RAM address from which to read
    uint16_t _addrRel;

    bool _nmiOccurred;
    bool _irqOccurred; // Produced by APU or game hw cartridge

    // Devices
    Bus* _bus;

    // Instruction set
    std::vector<inst> _instructionLut;

    // Debug
    debug_cpu_state_dsc_st _debugCPUState;
    bool _justFetched = false;

    NMI _nmiLine;
    IRQ _irqLine;

#ifdef CPU_FILE_LOG
    // Log file
    std::ofstream cpuLogFile;
#endif
};