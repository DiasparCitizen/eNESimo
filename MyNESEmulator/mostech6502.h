#pragma once

#include <vector>
#include "NESConstants.h"
#include <iostream>
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

	uint64_t pre_instruction_counter;
	uint64_t cpu_cycle;

	uint16_t pre_pc;
	uint8_t pre_reg_x;
	uint8_t pre_reg_y;
	uint8_t pre_reg_acc;
	uint8_t pre_reg_status;
	uint8_t pre_stack_ptr;

	uint16_t post_pc;
	uint8_t post_reg_x;
	uint8_t post_reg_y;
	uint8_t post_reg_acc;
	uint8_t post_reg_status;
	uint8_t post_stack_ptr;

	std::string inst_name;
	uint8_t opcode;
	uint8_t cycles;
	uint8_t extra_cycles;
	uint8_t nxt_inst;
	uint8_t nxt_nxt_inst;
};

// Forward declaration of Bus to prevent circular inclusions
class Bus;
class mostech6502;

struct inst {
	char name[4];
	uint8_t (mostech6502::*op)(void) = nullptr;
	uint8_t (mostech6502::*addr_mode)(void) = nullptr;
	uint8_t cycles;
};

#define _DEBUG_FILL_PRE_CPU_STATE() \
	debugCPUState.cpu_cycle = clock; \
	debugCPUState.pre_instruction_counter = instruction_counter; \
	debugCPUState.pre_pc = pc; \
	debugCPUState.pre_reg_x = reg_x; \
	debugCPUState.pre_reg_y = reg_y; \
	debugCPUState.pre_reg_acc = reg_acc; \
	debugCPUState.pre_reg_status = reg_status.raw; \
	debugCPUState.pre_stack_ptr = stack_ptr

#define _DEBUG_FILL_POST_CPU_STATE(in_inst_name, in_cycles, in_extra_cycles, in_nxt_inst, in_nxt_nxt_inst) \
	debugCPUState.post_pc = pc; \
	debugCPUState.post_reg_x = reg_x; \
	debugCPUState.post_reg_y = reg_y; \
	debugCPUState.post_reg_acc = reg_acc; \
	debugCPUState.post_reg_status = reg_status.raw; \
	debugCPUState.post_stack_ptr = stack_ptr; \
	debugCPUState.opcode = opcode; \
	debugCPUState.cycles = in_cycles; \
	debugCPUState.extra_cycles = in_extra_cycles; \
	debugCPUState.inst_name = std::string(in_inst_name); \
	debugCPUState.nxt_inst = in_nxt_inst; \
	debugCPUState.nxt_nxt_inst = in_nxt_nxt_inst

#define _FETCH() if ( mostech6502::instruction_lut[mostech6502::opcode].addr_mode != &mostech6502::imp ) mostech6502::M = mostech6502::read(mostech6502::addr_abs)

#define _STACK_PUSH(value) \
	write(CPU_ADDR_SPACE_STACK_START + stack_ptr, value); \
	stack_ptr--;

#define _STACK_POP(value) \
	stack_ptr++; \
	value = read(CPU_ADDR_SPACE_STACK_START + stack_ptr);

class mostech6502 {

public:

	mostech6502();
	~mostech6502();

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
	std::string getAddrMode(uint8_t opcode);

	// olc
	std::map<uint16_t, std::string> disassemble(uint16_t nStart, uint16_t nStop);

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
	uint16_t pc;
	uint8_t stack_ptr;
	uint8_t reg_x;
	uint8_t reg_y;
	uint8_t reg_acc;
	reg_status_t reg_status;

	// Helper vars
	uint8_t M; // Fetched byte, represented as 'M' in proc description in nesdev
	uint8_t opcode;

	uint32_t clock = 0;
	uint16_t cycles;
	uint64_t instruction_counter;

	uint16_t addr_abs; // RAM address from which to read
	uint16_t addr_rel;

	// Devices
	Bus* bus;

	// Instruction set
	std::vector<inst> instruction_lut;

	// Debug
	debug_cpu_state_dsc_st debugCPUState;
	bool justFetched = false;

#ifdef CPU_FILE_LOG
	// Log file
	std::ofstream cpuLogFile;
#endif
};