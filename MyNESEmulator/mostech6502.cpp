#include "mostech6502.h"
#include "Bus.h"

#if defined(CPU_TERMINAL_LOG) && defined(CPU_FILE_LOG)
#define _LOG(txt) \
{ \
std::stringstream myStream; \
myStream << txt; \
std::cout << myStream.str(); \
cpuLogFile << myStream.str(); \
} 
#elif defined(CPU_TERMINAL_LOG)
#define _LOG(txt) std::cout << txt
#elif defined(CPU_FILE_LOG)
#define _LOG(txt) cpuLogFile << txt
#else
#define _LOG(txt) ;
#endif

mostech6502::mostech6502()
{

	pc = 0x0000;
	reg_acc = 0x00;
	reg_x = 0x00;
	reg_y = 0x00;
	stack_ptr = 0x00;
	reg_status.raw = 0x00;
	
	M = 0x00;

	clock = 0;
	cycles = 0;
	instruction_counter = 0;

	addr_abs = 0x0000;
	addr_rel = 0x0000;

	_nmi_occurred = false;

	instruction_lut = { // Check: https://www.masswerk.at/6502/6502_instruction_set.html#PLA
		//////////////
		// 0
		{"BRK", &mostech6502::_brk, &mostech6502::imm, 7}, {"ORA", &mostech6502::_ora, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_nop, &mostech6502::imp, 3}, {"ORA", &mostech6502::_ora, &mostech6502::zpg, 3}, {"ASL", &mostech6502::_asl, &mostech6502::zpg, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 8
		{"PHP", &mostech6502::_php, &mostech6502::imp, 3}, {"ORA", &mostech6502::_ora, &mostech6502::imm, 2}, {"ASL", &mostech6502::_asl, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"ORA", &mostech6502::_ora, &mostech6502::abs, 4}, {"ASL", &mostech6502::_asl, &mostech6502::abs, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 16
		{"BPL", &mostech6502::_bpl, &mostech6502::rel, 2}, {"ORA", &mostech6502::_ora, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"ORA", &mostech6502::_ora, &mostech6502::zpgx, 4}, {"ASL", &mostech6502::_asl, &mostech6502::zpgx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 24
		{"CLC", &mostech6502::_clc, &mostech6502::imp, 2}, {"ORA", &mostech6502::_ora, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"ORA", &mostech6502::_ora, &mostech6502::absx, 4}, {"ASL", &mostech6502::_asl, &mostech6502::absx, 7}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		// 32
		{"JSR", &mostech6502::_jsr, &mostech6502::abs, 6}, {"AND", &mostech6502::_and, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"BIT", &mostech6502::_bit, &mostech6502::zpg, 3}, {"AND", &mostech6502::_and, &mostech6502::zpg, 3}, {"ROL", &mostech6502::_rol, &mostech6502::zpg, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 40
		{"PLP", &mostech6502::_plp, &mostech6502::imp, 4}, {"AND", &mostech6502::_and, &mostech6502::imm, 2}, {"ROL", &mostech6502::_rol, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"BIT", &mostech6502::_bit, &mostech6502::abs, 4}, {"AND", &mostech6502::_and, &mostech6502::abs, 4}, {"ROL", &mostech6502::_rol, &mostech6502::abs, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 48
		{"BMI", &mostech6502::_bmi, &mostech6502::rel, 2}, {"AND", &mostech6502::_and, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"AND", &mostech6502::_and, &mostech6502::zpgx, 4}, {"ROL", &mostech6502::_rol, &mostech6502::zpgx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 56
		{"SEC", &mostech6502::_sec, &mostech6502::imp, 2}, {"AND", &mostech6502::_and, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"AND", &mostech6502::_and, &mostech6502::absx, 4}, {"ROL", &mostech6502::_rol, &mostech6502::absx, 7}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		//////////////
		// 64
		{"RTI", &mostech6502::_rti, &mostech6502::imp, 6}, {"EOR", &mostech6502::_eor, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 3}, {"EOR", &mostech6502::_eor, &mostech6502::zpg, 3}, {"LSR", &mostech6502::_lsr, &mostech6502::zpg, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 72
		{"PHA", &mostech6502::_pha, &mostech6502::imp, 3}, {"EOR", &mostech6502::_eor, &mostech6502::imm, 2}, {"LSR", &mostech6502::_lsr, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"JMP", &mostech6502::_jmp, &mostech6502::abs, 3}, {"EOR", &mostech6502::_eor, &mostech6502::abs, 4}, {"LSR", &mostech6502::_lsr, &mostech6502::abs, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 80
		{"BVC", &mostech6502::_bvc, &mostech6502::rel, 2}, {"EOR", &mostech6502::_eor, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"EOR", &mostech6502::_eor, &mostech6502::zpgx, 4}, {"LSR", &mostech6502::_lsr, &mostech6502::zpgx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 88
		{"CLI", &mostech6502::_cli, &mostech6502::imp, 2}, {"EOR", &mostech6502::_eor, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"EOR", &mostech6502::_eor, &mostech6502::absx, 4}, {"LSR", &mostech6502::_lsr, &mostech6502::absx, 7}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		// 96
		{"RTS", &mostech6502::_rts, &mostech6502::imp, 6}, {"ADC", &mostech6502::_adc, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 3}, {"ADC", &mostech6502::_adc, &mostech6502::zpg, 3}, {"ROR", &mostech6502::_ror, &mostech6502::zpg, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 104
		{"PLA", &mostech6502::_pla, &mostech6502::imp, 4}, {"ADC", &mostech6502::_adc, &mostech6502::imm, 2}, {"ROR", &mostech6502::_ror, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"JMP", &mostech6502::_jmp, &mostech6502::ind, 5}, {"ADC", &mostech6502::_adc, &mostech6502::abs, 4}, {"ROR", &mostech6502::_ror, &mostech6502::abs, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 112
		{"BVS", &mostech6502::_bvs, &mostech6502::rel, 2}, {"ADC", &mostech6502::_adc, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"ADC", &mostech6502::_adc, &mostech6502::zpgx, 4}, {"ROR", &mostech6502::_ror, &mostech6502::zpgx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 120
		{"SEI", &mostech6502::_sei, &mostech6502::imp, 2}, {"ADC", &mostech6502::_adc, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"ADC", &mostech6502::_adc, &mostech6502::absx, 4}, {"ROR", &mostech6502::_ror, &mostech6502::absx, 7}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		//////////////
		// 128
		{"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"STA", &mostech6502::_sta, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		{"STY", &mostech6502::_sty, &mostech6502::zpg, 3}, {"STA", &mostech6502::_sta, &mostech6502::zpg, 3}, {"STX", &mostech6502::_stx, &mostech6502::zpg, 3}, {"***", &mostech6502::_xxx, &mostech6502::imp, 3},
		// 136
		{"DEY", &mostech6502::_dey, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"TXA", &mostech6502::_txa, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"STY", &mostech6502::_sty, &mostech6502::abs, 4}, {"STA", &mostech6502::_sta, &mostech6502::abs, 4}, {"STX", &mostech6502::_stx, &mostech6502::abs, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 4},
		// 144
		{"BCC", &mostech6502::_bcc, &mostech6502::rel, 2}, {"STA", &mostech6502::_sta, &mostech6502::indy, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		{"STY", &mostech6502::_sty, &mostech6502::zpgx, 4}, {"STA", &mostech6502::_sta, &mostech6502::zpgx, 4}, {"STX", &mostech6502::_stx, &mostech6502::zpgy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 4},
		// 152
		{"TYA", &mostech6502::_tya, &mostech6502::imp, 2}, {"STA", &mostech6502::_sta, &mostech6502::absy, 5}, {"TXS", &mostech6502::_txs, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 5}, {"STA", &mostech6502::_sta, &mostech6502::absx, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 160
		{"LDY", &mostech6502::_ldy, &mostech6502::imm, 2}, {"LDA", &mostech6502::_lda, &mostech6502::indx, 6}, {"LDX", &mostech6502::_ldx, &mostech6502::imm, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		{"LDY", &mostech6502::_ldy, &mostech6502::zpg, 3}, {"LDA", &mostech6502::_lda, &mostech6502::zpg, 3}, {"LDX", &mostech6502::_ldx, &mostech6502::zpg, 3}, {"***", &mostech6502::_xxx, &mostech6502::imp, 3},
		// 168
		{"TAY", &mostech6502::_tay, &mostech6502::imp, 2}, {"LDA", &mostech6502::_lda, &mostech6502::imm, 2}, {"TAX", &mostech6502::_tax, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"LDY", &mostech6502::_ldy, &mostech6502::abs, 4}, {"LDA", &mostech6502::_lda, &mostech6502::abs, 4}, {"LDX", &mostech6502::_ldx, &mostech6502::abs, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 4},
		// 176
		{"BCS", &mostech6502::_bcs, &mostech6502::rel, 2}, {"LDA", &mostech6502::_lda, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		{"LDY", &mostech6502::_ldy, &mostech6502::zpgx, 4}, {"LDA", &mostech6502::_lda, &mostech6502::zpgx, 4}, {"LDX", &mostech6502::_ldx, &mostech6502::zpgy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 4},
		// 184
		{"CLV", &mostech6502::_clv, &mostech6502::imp, 2}, {"LDA", &mostech6502::_lda, &mostech6502::absy, 4}, {"TSX", &mostech6502::_tsx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 4},
		{"LDY", &mostech6502::_ldy, &mostech6502::absx, 4}, {"LDA", &mostech6502::_lda, &mostech6502::absx, 4}, {"LDX", &mostech6502::_ldx, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 4},
		//////////////
		// 192
		{"CPY", &mostech6502::_cpy, &mostech6502::imm, 2}, {"CMP", &mostech6502::_cmp, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"CPY", &mostech6502::_cpy, &mostech6502::zpg, 3}, {"CMP", &mostech6502::_cmp, &mostech6502::zpg, 3}, {"DEC", &mostech6502::_dec, &mostech6502::zpg, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 200
		{"INY", &mostech6502::_iny, &mostech6502::imp, 2}, {"CMP", &mostech6502::_cmp, &mostech6502::imm, 2}, {"DEX", &mostech6502::_dex, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"CPY", &mostech6502::_cpy, &mostech6502::abs, 4}, {"CMP", &mostech6502::_cmp, &mostech6502::abs, 4}, {"DEC", &mostech6502::_dec, &mostech6502::abs, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 208
		{"BNE", &mostech6502::_bne, &mostech6502::rel, 2}, {"CMP", &mostech6502::_cmp, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"CMP", &mostech6502::_cmp, &mostech6502::zpgx, 4}, {"DEC", &mostech6502::_dec, &mostech6502::zpgx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 216
		{"CLD", &mostech6502::_cld, &mostech6502::imp, 2}, {"CMP", &mostech6502::_cmp, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"CMP", &mostech6502::_cmp, &mostech6502::absx, 4}, {"DEC", &mostech6502::_dec, &mostech6502::absx, 7}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		// 224
		{"CPX", &mostech6502::_cpx, &mostech6502::imm, 2}, {"SBC", &mostech6502::_sbc, &mostech6502::indx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"CPX", &mostech6502::_cpx, &mostech6502::zpg, 3}, {"SBC", &mostech6502::_sbc, &mostech6502::zpg, 3}, {"INC", &mostech6502::_inc, &mostech6502::zpg, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 5},
		// 232
		{"INX", &mostech6502::_inx, &mostech6502::imp, 2}, {"SBC", &mostech6502::_sbc, &mostech6502::imm, 2}, {"NOP", &mostech6502::_nop, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2},
		{"CPX", &mostech6502::_cpx, &mostech6502::abs, 4}, {"SBC", &mostech6502::_sbc, &mostech6502::abs, 4}, {"INC", &mostech6502::_inc, &mostech6502::abs, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 240
		{"BEQ", &mostech6502::_beq, &mostech6502::rel , 2}, {"SBC", &mostech6502::_sbc, &mostech6502::indy, 5}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 8},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"SBC", &mostech6502::_sbc, &mostech6502::zpgx, 4}, {"INC", &mostech6502::_inc, &mostech6502::zpgx, 6}, {"***", &mostech6502::_xxx, &mostech6502::imp, 6},
		// 248
		{"SED", &mostech6502::_sed, &mostech6502::imp, 2}, {"SBC", &mostech6502::_sbc, &mostech6502::absy, 4}, {"***", &mostech6502::_xxx, &mostech6502::imp, 2}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7},
		{"***", &mostech6502::_xxx, &mostech6502::imp, 4}, {"SBC", &mostech6502::_sbc, &mostech6502::absx, 4}, {"INC", &mostech6502::_inc, &mostech6502::absx, 7}, {"***", &mostech6502::_xxx, &mostech6502::imp, 7}
	};

#ifdef CPU_FILE_LOG
	cpuLogFile.open("cpu_log.txt");
#endif
}

mostech6502::~mostech6502()
{
#ifdef CPU_FILE_LOG
	cpuLogFile.close();
#endif
}



/**********************************************************/
/************ Addressing modes ****************************/
/**********************************************************/

//  Immediate
uint8_t mostech6502::imm()
{
	addr_abs = pc++; // The address of the operand is the address of the next instruction, which is in itself the operand!
	return 0;
}

// Absolute
uint8_t mostech6502::abs()
{
	addr_abs = read(pc++); // LO part
	addr_abs |= read(pc++) << 8; // HI part
	return 0;
}

// Absolute X
uint8_t mostech6502::absx()
{
	addr_abs = read(pc++); // LO part
	uint16_t hi = read(pc++) << 8;
	addr_abs |= hi; // HI part
	addr_abs += reg_x; // Add X

	// If the page boundary is crossed, an extra cycle is needed
	if ((addr_abs & 0xFF00) != hi)
		return 1;
	else
		return 0;

}

// Absolute Y
uint8_t mostech6502::absy()
{
	addr_abs = read(pc++); // LO part
	uint16_t hi = read(pc++) << 8;
	addr_abs |= hi; // HI part
	addr_abs += reg_y; // Add Y

	// If the page boundary is crossed, an extra cycle is needed
	if ((addr_abs & 0xFF00) != hi)
		return 1;
	else
		return 0;

}

// Zero page
uint8_t mostech6502::zpg()
{

	addr_abs = read(pc);
	addr_abs &= CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK; // Will do even though it's unneeded
	pc++;
	return 0;

}

// Zero page X
uint8_t mostech6502::zpgx()
{
	addr_abs = read(pc) + reg_x; // M will be located at memory[pc+1 + x]
	addr_abs &= CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK; // Protect against reg_x breaking boundary
	pc++;
	return 0;
}

// Zero page Y
uint8_t mostech6502::zpgy()
{

	addr_abs = read(pc) + reg_y; // M will be located at memory[pc+1 + y]
	addr_abs &= CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK; // Protect against reg_y breaking boundary
	pc++;
	return 0;

}

//  Implied
uint8_t mostech6502::imp()
{
	M = reg_acc; // Dunno why, apparently a wa for _pha()
	return 0;
}

//  Accumulator
uint8_t mostech6502::acc()
{
	M = reg_acc;
	return 0;
}

// Indirect
uint8_t mostech6502::ind()
{

	uint16_t ptr_lo = read(pc++);
	uint16_t ptr_hi = read(pc++);

	uint16_t ptr = (ptr_hi << 8) | ptr_lo;

	if (ptr_lo != 0x00FF) {

		// Normal behavior
		addr_abs = read(ptr); // LO part
		addr_abs |= read(ptr + 1) << 8; // HI part

	}
	else {

		// Simulate an existing hardware bug!
		// Specifically, the page boundary hardware bug
		addr_abs = read(ptr); // LO part
		addr_abs |= read(ptr & 0xFF00) << 8; // HI part

	}

	return 0;

}

// Indirect X
// Strictly read from page 0
// *(mem( inst[pc+1] + x ))
uint8_t mostech6502::indx() {

	uint16_t ptr = read(pc++);

	uint16_t lo_addr = (ptr + reg_x) & 0xFF;
	uint16_t hi_addr = (ptr + 1 + reg_x) & 0xFF;

	uint8_t lo = read(lo_addr);
	uint8_t hi = read(hi_addr);

	addr_abs = (hi << 8) | lo;

	return 0;
}

// Indirect Y
// Strictly read from page 0
// *(mem( inst[pc+1] + y ))
uint8_t mostech6502::indy() {

	uint16_t ptr = (uint16_t)read(pc++) & 0xFF;

	uint8_t lo = read(ptr);
	uint8_t hi = read((ptr + 1) & 0xFF); // Read only first page

	addr_abs = (hi << 8) | lo;
	addr_abs += reg_y;

	if ((addr_abs & 0xFF00) != (hi << 8)) {
		return 1;
	}
	else {
		return 0;
	}
}

// Relative address
uint8_t mostech6502::rel()
{
	// Read from the next instruction
	// the address delta, which will be a jump
	// between -128 and +127
	addr_rel = read(pc++);
	if (addr_rel & 0x0080)
		addr_rel |= 0xFF00;
	return 0;
}

/**********************************************************/
/************ Instructions ********************************/
/**********************************************************/

uint8_t mostech6502::_xxx()
{
	// What to do, what to do...
	return 0;
}

uint8_t mostech6502::_adc()
{
	_FETCH();

	uint16_t res = (uint16_t)reg_acc + (uint16_t)M + (uint16_t)reg_status.C;

	reg_status.C = res > 0xFF; // Set carry if overflow
	reg_status.Z = (res & 0x00FF) == 0x00; // Set zero flag if zero
	reg_status.S = (res & 0x0080) != 0; // Get signed bit

	// V = ~(A^M) & (A^R)
	reg_status.V = ((~((uint16_t)reg_acc ^ (uint16_t)M) & ((uint16_t)reg_acc ^ res)) & 0x0080) != 0;

	// Load into acc
	reg_acc = res & 0xFF;

	// This instruction will require an additional clock cycle
	return 1;
}

// Subtract memory from accumulator with borrow
/*
	temp = AC - M - (IF_CARRY() ? 0 : 1);

	temp = acc - M - (1 - C)
	temp = acc - M - 1 + C
	temp = acc + (-M) - 1 + C
	temp = acc + (~M + 1) - 1 + C
	temp = acc + ~M + C

	To invert a number:
	invert bits
	add one
*/
uint8_t mostech6502::_sbc()
{
	// Basically the same implementation of ADC, just inverting the bits of M

	_FETCH();

	M ^= 0xFF; // Invert

	uint16_t op_result = (uint16_t)reg_acc + (uint16_t)M + (uint16_t)reg_status.C;

	reg_status.C = op_result > 0xFF; // Set carry if overflow
	reg_status.Z = (op_result & 0x00FF) == 0x00; // Set zero flag if zero
	reg_status.S = (op_result & 0x0080) != 0; // Get signed bit

	// V = ~(A^M) & (A^R)
	reg_status.V = ((~((uint16_t)reg_acc ^ (uint16_t)M) & ((uint16_t)reg_acc ^ op_result)) & 0x0080) != 0;

	// Load into acc
	reg_acc = op_result & 0xFF;

	// This instruction will require an additional clock cycle
	return 1;

	/*_FETCH();

	uint16_t inverted_M = (uint16_t)M ^ 0xFF; // Equivalente a ~M & 0xFF
	uint16_t temp = (uint16_t)reg_acc - (uint16_t)inverted_M + reg_status.C;

	// Sign
	reg_status.S = (temp & 0x80) != 0;
	// Zero
	reg_status.Z = (temp & 0xFF) == 0x00;
	// Carry
	reg_status.C = temp > 0xFF;
	// Signed overflow
	// V = ~(A^M) & (A^R) = (A^~M) & (A^R)

	// WATCH OUT!!!!!!

	uint16_t signed_ovfl = (((uint16_t)temp ^ (uint16_t)inverted_M) & ((uint16_t)reg_acc ^ (uint16_t)temp)) & 0x0080;
	reg_status.V = signed_ovfl != 0x0000;

	reg_acc = temp & 0xFF;

	return 1;*/
}

uint8_t mostech6502::_and()
{
	_FETCH();

	uint8_t res = M & reg_acc;

	reg_status.S = (res & 0x80) != 0;
	reg_status.Z = res == 0x00;

	reg_acc = res;

	return 1;
}

// Arithmetic shift left
uint8_t mostech6502::_asl()
{
	_FETCH();

	// Save msb, set carry
	reg_status.C = (M & 0x80) != 0;

	// Shift
	M <<= 1;

	// Set sign
	reg_status.S = (M & 0x80) != 0;

	// Zero
	reg_status.Z = M == 0x00;

	// Depending on the addressing mode, store
	// either in the accumulator or in back in memory
	if (instruction_lut[opcode].addr_mode == &mostech6502::imp) {
		reg_acc = M;
	}
	else {
		write(addr_abs, M);
	}

	return 0;
}

// Branch if carry clear
uint8_t mostech6502::_bcc()
{
	if (reg_status.C == 0) {

		cycles++;
		
		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

// Branch if carry set
uint8_t mostech6502::_bcs()
{
	if (reg_status.C) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

uint8_t mostech6502::_beq()
{
	if (reg_status.Z) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

// Test bits in memory with accumulator
// S and V are always set according to the value
// fetched from memory. Z depends on the result
// of the AND operation.
uint8_t mostech6502::_bit()
{
	_FETCH();

	uint8_t temp = reg_acc & M;

	// Signed flag
	reg_status.S = (M & 0x80) != 0;

	// Overflow flag
	reg_status.V = (M & 0x40) != 0;

	// Zero flag
	reg_status.Z = temp == 0x00;

	return 0;
}

uint8_t mostech6502::_bmi()
{
	if (reg_status.S) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

uint8_t mostech6502::_bne()
{
	if (reg_status.Z == 0) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

uint8_t mostech6502::_bpl()
{
	if (reg_status.S == 0) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

// Break
uint8_t mostech6502::_brk()
{
	// First off, increase pc;
	// this new address of pc is the one we'll be coming
	// back to
	pc++;

	// Save pc in stack
	write(CPU_ADDR_SPACE_STACK_START + stack_ptr, pc >> 8);
	stack_ptr--;
	write(CPU_ADDR_SPACE_STACK_START + stack_ptr, pc & 0x00FF);
	stack_ptr--;

	// Set break
	reg_status.B = 1;

	// Write status reg in stack too
	write(CPU_ADDR_SPACE_STACK_START + stack_ptr, reg_status.raw);
	stack_ptr--;

	// Interrupt
	// I guess the method here is to just overwrite pc
	// with the address of the interrupt routine
	pc = (uint16_t)read(0xFFFE) | ((uint16_t)read(0xFFFF) << 8);

	return 0;
}

uint8_t mostech6502::_bvc()
{
	if (reg_status.V == 0) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

uint8_t mostech6502::_bvs()
{
	if (reg_status.V) {

		cycles++;

		// If we've crossed the page boundary,
		// the whole thing takes one more cycle
		addr_abs = addr_rel + pc;
		if ((addr_abs & 0xFF00) != (pc & 0xFF00))
			cycles++;

		pc = addr_abs;

	}
	return 0;
}

// Clear carry flag
uint8_t mostech6502::_clc()
{
	reg_status.C = 0;
	return 0;
}

// Clear decimal flag
uint8_t mostech6502::_cld()
{
	reg_status.D = 0;
	return 0;
}

// Clear interrupt flag
uint8_t mostech6502::_cli()
{
	reg_status.I = 0;
	return 0;
}

// Clear overflow flag
uint8_t mostech6502::_clv()
{
	reg_status.V = 0;
	return 0;
}

uint8_t mostech6502::_cmp()
{
	_FETCH();

	uint16_t temp = (uint16_t)reg_acc - (uint16_t)M;

	reg_status.C = reg_acc >= M;

	reg_status.S = (temp & 0x80) != 0;

	reg_status.Z = (temp & 0x00FF) == 0x00;

	return 1;
}

uint8_t mostech6502::_cpx()
{
	_FETCH();

	uint16_t temp = (uint16_t)reg_x - (uint16_t)M;

	reg_status.C = reg_x >= M;
	reg_status.S = (temp >> 7) & 0x01;
	reg_status.Z = (temp & 0x00FF) == 0;

	return 0;
}

uint8_t mostech6502::_cpy()
{
	_FETCH();

	uint16_t temp = (uint16_t)reg_y - (uint16_t)M;

	reg_status.C = reg_y >= temp;
	reg_status.S = (temp >> 7) & 0x01;
	reg_status.Z = (temp & 0x00FF) == 0;

	return 0;
}

// Decrement value at memory location
uint8_t mostech6502::_dec()
{
	_FETCH();

	uint16_t temp = ((uint16_t)M - 1) & 0xFF;

	reg_status.S = (temp >> 7) & 0x01;
	reg_status.Z = (temp & 0x00FF) == 0x00;
	
	write(addr_abs, (uint8_t)temp);

	return 0;
}

uint8_t mostech6502::_dex()
{
	reg_x--;
	reg_status.S = (reg_x & 0x80) != 0;
	reg_status.Z = (reg_x & 0xFF) == 0x00;
	return 0;
}

uint8_t mostech6502::_dey()
{
	reg_y--;
	reg_status.S = (reg_y & 0x80) != 0;
	reg_status.Z = (reg_y & 0xFF) == 0x00;
	return 0;
}

// Bitwise logic XOR
// Function: acc = acc xor M
uint8_t mostech6502::_eor()
{
	_FETCH();

	reg_acc ^= M;

	reg_status.S = (reg_acc & 0x80) != 0;
	reg_status.Z = (reg_acc & 0xFF) == 0x00;

	return 1;

}

uint8_t mostech6502::_inc()
{
	_FETCH();

	M += 1;

	reg_status.S = (M & 0x80) != 0;
	reg_status.Z = M == 0x00;

	write(addr_abs, M);

	return 0;
}

uint8_t mostech6502::_inx()
{
	reg_x += 1;

	reg_status.S = (reg_x & 0x80) != 0;
	reg_status.Z = reg_x == 0x00;

	return 0;
}

uint8_t mostech6502::_iny()
{
	reg_y += 1;

	reg_status.S = (reg_y & 0x80) != 0;
	reg_status.Z = reg_y == 0x00;

	return 0;
}

uint8_t mostech6502::_jmp()
{
	pc = addr_abs;
	return 0;
}

// Jump to a subroutine
uint8_t mostech6502::_jsr()
{
	pc = pc - 1;

	uint8_t lo = pc & 0xFF;
	uint8_t hi = (pc >> 8) & 0xFF;

	// Push to stack
	_STACK_PUSH(hi);
	_STACK_PUSH(lo);

	pc = addr_abs;

	return 0;
}

// Load the accumulator
uint8_t mostech6502::_lda()
{
	_FETCH();

	reg_acc = M;

	reg_status.S = (reg_acc & 0x80) != 0;
	reg_status.Z = reg_acc == 0x00;

	return 1;
}

uint8_t mostech6502::_ldx()
{
	_FETCH();

	reg_x = M;

	reg_status.S = (reg_x & 0x80) != 0;
	reg_status.Z = reg_x == 0x00;

	return 1;
}

uint8_t mostech6502::_ldy()
{
	_FETCH();

	reg_y = M;

	reg_status.S = (reg_y & 0x80) != 0;
	reg_status.Z = reg_y == 0x00;

	return 1;
}

uint8_t mostech6502::_lsr()
{
	_FETCH();

	reg_status.C = M & 0x1;

	M >>= 1;

	reg_status.S = 0;// (M & 0x80) != 0; --> Will always be 0
	reg_status.Z = M == 0x00;

	// Depending on the addressing mode, store
	// either in the accumulator or in back in memory
	if (instruction_lut[opcode].addr_mode == &mostech6502::imp) {
		reg_acc = M;
	}
	else {
		write(addr_abs, M);
	}

	return 0;
}

uint8_t mostech6502::_nop()
{
	// Well, not all nops are created equal.
	// Need to look for more info on this
	switch (opcode) {
	case 0x1C:
	case 0x3C:
	case 0x5C:
	case 0x7C:
	case 0xDC:
	case 0xFC:
		return 1;
		break;
	default:
		return 0;
	}

}

// Function: A = A | M
uint8_t mostech6502::_ora()
{
	_FETCH();
	reg_acc |= M;
	reg_status.S = (reg_acc & 0x80) != 0;
	reg_status.Z = reg_acc == 0x00;
	return 0;
}

// Push accumulator to stack
uint8_t mostech6502::_pha()
{
	write(CPU_ADDR_SPACE_STACK_START + stack_ptr, reg_acc);
	stack_ptr--;
	return 0;
}

// Push status register to stack
uint8_t mostech6502::_php()
{
	// Apparently I gotta set two flags before the push:
	// Break flag and reserved. Why and why?
	reg_status.B = 1;
	reg_status.reserved = 1;
	_STACK_PUSH(reg_status.raw);
	// Reset flags to original state
	reg_status.B = 0;
	reg_status.reserved = 0;
	return 0;
}

// Pop accumulator from stack
uint8_t mostech6502::_pla()
{
	stack_ptr++; // Increment; keep in mind that the ptr always points to the next position to be written
	reg_acc = read(CPU_ADDR_SPACE_STACK_START + stack_ptr);
	// Set flags
	reg_status.S = (reg_acc & 0x80) != 0;
	reg_status.Z = reg_acc == 0x00;
	return 0;
}

// Pop status register from stack
uint8_t mostech6502::_plp()
{
	stack_ptr++; // Increment; keep in mind that the ptr always points to the next position to be written
	reg_status.raw = read(CPU_ADDR_SPACE_STACK_START + stack_ptr);
	// Set flags
	reg_status.reserved = 1; // Wft?
	return 0;
}

// Rotate one bit left
uint8_t mostech6502::_rol()
{
	_FETCH();
	// Before doing the shift, save bit 7 (msb)
	uint8_t saved_msb = (M >> 7) & 0x1;
	// Do op
	M = (M << 1) | (reg_status.C ? 0x01 : 0x00);
	// Set flags
	reg_status.C = saved_msb;
	reg_status.Z = M == 0x00;
	reg_status.S = (M & 0x80) != 0;
	// Depending on the addressing mode, store
	// either in the accumulator or in back in memory
	if (instruction_lut[opcode].addr_mode == &mostech6502::imp) {
		reg_acc = M;
	}
	else {
		write(addr_abs, M);
	}
	return 0;
}

// Rotate one bit right
uint8_t mostech6502::_ror()
{
	_FETCH();
	// Before doing the shift, register bit 7 in the carry
	uint8_t saved_lsb = M & 0x1;
	// Do op
	M = (M >> 1) | (reg_status.C ? 0x80 : 0x00);
	// Set flags
	reg_status.C = saved_lsb;
	reg_status.Z = M == 0x00;
	reg_status.S = (M & 0x80) != 0;
	// Depending on the addressing mode, store
	// either in the accumulator or in back in memory
	if (instruction_lut[opcode].addr_mode == &mostech6502::imp) {
		reg_acc = M;
	}
	else {
		write(addr_abs, M);
	}
	return 0;
}

// Return from interrupt
uint8_t mostech6502::_rti()
{
	// Recover status register
	stack_ptr++;
	reg_status.raw = read(CPU_ADDR_SPACE_STACK_START + stack_ptr);
	// Again, this is unknown to me...
	reg_status.B = 0;
	reg_status.reserved = 0;

	// Recover Program Counter
	stack_ptr++;
	uint8_t lo = read(CPU_ADDR_SPACE_STACK_START + stack_ptr); // LO part
	stack_ptr++;
	uint8_t hi = read(CPU_ADDR_SPACE_STACK_START + stack_ptr); // Hi part

	pc = (hi << 8) | lo;

	return 0;
}

// Return from subroutine
uint8_t mostech6502::_rts()
{
	// Recover Program Counter
	stack_ptr++;
	uint8_t lo = read(CPU_ADDR_SPACE_STACK_START + stack_ptr); // LO part
	stack_ptr++;
	uint8_t hi = read(CPU_ADDR_SPACE_STACK_START + stack_ptr); // Hi part

	pc = (hi << 8) | lo;
	// Advance PC to execute next instruction
	// When going to a subroutine, we store the last address executed,
	// not the next to execute.
	pc++;
	return 0;
}

// Set decimal flag
uint8_t mostech6502::_sec()
{
	reg_status.C = 1;
	return 0;
}

// Set decimal flag
uint8_t mostech6502::_sed()
{
	reg_status.D = 1;
	return 0;
}

// Set interrupt flag
uint8_t mostech6502::_sei()
{
	reg_status.I = 1;
	return 0;
}

// Store accumulator at address
uint8_t mostech6502::_sta()
{
	write(addr_abs, reg_acc);
	return 0;
}

// Store X register at address
uint8_t mostech6502::_stx()
{
	write(addr_abs, reg_x);
	return 0;
}

// Store Y register at address
uint8_t mostech6502::_sty()
{
	write(addr_abs, reg_y);
	return 0;
}

// Transfer accumulator to X register
uint8_t mostech6502::_tax()
{
	reg_x = reg_acc;
	// Set flags
	reg_status.Z = reg_x == 0x00;
	reg_status.S = (reg_x & 0x80) != 0;
	return 0;
}

// Transfer accumulator to Y register
uint8_t mostech6502::_tay()
{
	reg_y = reg_acc;
	// Set flags
	reg_status.Z = reg_y == 0x00;
	reg_status.S = (reg_y & 0x80) != 0;
	return 0;
}

// Transfer stack pointer to X register
uint8_t mostech6502::_tsx()
{
	reg_x = stack_ptr;
	// Set flags
	reg_status.Z = reg_x == 0x00;
	reg_status.S = (reg_x & 0x80) != 0;
	return 0;
}

// Transfer X register to accumulator
uint8_t mostech6502::_txa()
{
	reg_acc = reg_x;
	// Set flags
	reg_status.Z = reg_acc == 0x00;
	reg_status.S = (reg_acc & 0x80) != 0;
	return 0;
}

// Transfer X register to stack pointer
uint8_t mostech6502::_txs()
{
	stack_ptr = reg_x;
	return 0;
}

// Transfer Y register to accumulator
uint8_t mostech6502::_tya()
{
	reg_acc = reg_y;
	// Set flags
	reg_status.Z = reg_acc == 0x00;
	reg_status.S = (reg_acc & 0x80) != 0;
	return 0;
}




/**********************************************************/
/************ Basic ops ***********************************/
/**********************************************************/

uint8_t mostech6502::fetch()
{
	return bus->cpuRead(pc++, false);
}

uint8_t mostech6502::read(uint16_t addr)
{
	return bus->cpuRead(addr, false);
}

void mostech6502::write(uint16_t addr, uint8_t data)
{
	bus->cpuWrite(addr, data);
}

void mostech6502::advanceClock() {

	if (cycles == 0) {

		_DEBUG_FILL_PRE_CPU_STATE();

		// pc should always point at the next operation code
		opcode = read(pc);

		// Leave pc ready to fetch the first instruction argument, if any
		pc++;

		// Get number of cycles for the current instruction
		cycles = instruction_lut[opcode].cycles;

		// Fetch intermediate data (M), accumulate an extra cycle if needed
		uint8_t extra_cycle = (this->*(instruction_lut[opcode].addr_mode))();

		// Perform operation associated to this instruction
		uint8_t allows_extra_cycle = (this->*(instruction_lut[opcode].op))();

		cycles += (extra_cycle & allows_extra_cycle);

		// Apparently, set Unused flag to 1 every instruction execution
		reg_status.reserved = 1;

		// Increment global instruction counter
		instruction_counter++;

		_DEBUG_FILL_POST_CPU_STATE( instruction_lut[opcode].name, instruction_lut[opcode].cycles, (extra_cycle & allows_extra_cycle), read(debugCPUState.pre_pc + 1), read(debugCPUState.pre_pc + 2) );

		_LOG(getPreExecuteStateAsStr());

		justFetched = true;

	}
	else {
		justFetched = false;
	}

	cycles--;

	// Increment global clock count
	clock++;

}

bool mostech6502::isInstructionComplete()
{
	return cycles == 0;
}

void mostech6502::nmi() {

	uint8_t pc_lo = pc & 0xFF;
	uint8_t pc_hi = (pc >> 8) & 0xFF;

	// Add the PC to the stack
	_STACK_PUSH(pc_hi);
	_STACK_PUSH(pc_lo);

	// Save the status register
	reg_status.B = 1;
	reg_status.I = 1; // No more interrupts!
	reg_status.reserved = 1;
	_STACK_PUSH(reg_status.raw);

	// Read the new program counter
	// The reset address is located at address 0xFFFC
	addr_abs = NMI_ADDR;
	pc_lo = read(addr_abs);
	pc_hi = read(addr_abs + 1);
	pc = (pc_hi << 8) | pc_lo; // New program counter address

	// NMIs take 7 cycles
	cycles = 8;

}

void mostech6502::irq() {

	if (reg_status.I == 0) {

		uint8_t pc_lo = pc & 0xFF;
		uint8_t pc_hi = (pc >> 8) & 0xFF;

		// Add the PC to the stack
		_STACK_PUSH(pc_hi);
		_STACK_PUSH(pc_lo);

		// Save the status register
		reg_status.B = 1;
		reg_status.I = 1; // No more interrupts!
		reg_status.reserved = 1;
		_STACK_PUSH(reg_status.raw)

		// Read the new program counter
		// The reset address is located at address 0xFFFC
		addr_abs = IRQ_ADDR;
		pc_lo = read(addr_abs);
		pc_hi = read(addr_abs + 1);
		pc = (pc_hi << 8) | pc_lo; // New program counter address

		// IRQs take 7 cycles
		cycles = 7;

	}

}

void mostech6502::reset()
{

	// The reset address is located at address 0xFFFC
	addr_abs = RESET_ADDR;

	uint16_t lo = read(addr_abs);
	uint16_t hi = read(addr_abs + 1);

#if defined(CPU_DEBUG_MODE) || defined(FORCE_START_PC)
	pc = CPU_DEBUG_MODE_START_PC;
#else
	pc = (hi << 8) | lo; // New program counter address
#endif

	// Reset (zero) main regs
	reg_x = 0x00;
	reg_y = 0x00;
	reg_acc = 0x00;
	reg_status.raw = 0x00;
	reg_status.reserved = 1; // Set reserved bit to 1, God knows why...

	M = 0x00;
	addr_abs = 0x0000;
	addr_rel = 0x0000;
	stack_ptr = CPU_ADDR_SPACE_STACK_INITIAL_OFFSET;

	// Reset will take 8 cycles to execute
	cycles = RESET_TICKS;

}

//////////////////////////////////
//////////// DEBUG ///////////////
//////////////////////////////////

std::string mostech6502::getAddrMode(uint8_t opcode) {

	std::string addrMode = "?";

	if (instruction_lut[opcode].addr_mode == &mostech6502::imp)
	{
		addrMode = "imp";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::imm)
	{
		addrMode = "imm";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::zpg)
	{
		addrMode = "zpg";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::zpgx)
	{
		addrMode = "zpgx";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::zpgy)
	{
		addrMode = "zpgy";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::indx)
	{
		addrMode = "indx";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::indy)
	{
		addrMode = "indy";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::abs)
	{
		addrMode = "abs";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::absx)
	{
		addrMode = "absx";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::absy)
	{
		addrMode = "absy";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::ind)
	{
		addrMode = "ind";
	}
	else if (instruction_lut[opcode].addr_mode == &mostech6502::rel)
	{
		addrMode = "rel";
	}

	return addrMode;

}

void mostech6502::attachBus(Bus* bus)
{
	this->bus = bus;
}

void mostech6502::printCpuState() {

	std::cout << "*** CPU STATE ***" << std::endl;
	std::cout << "x          -> 0x" << std::hex << (uint16_t)reg_x << std::endl;
	std::cout << "y          -> 0x" << std::hex << (uint16_t)reg_y << std::endl;
	std::cout << "acc        -> 0x" << std::hex << (uint16_t)reg_acc << std::endl;

	std::cout << "pc         -> 0x" << std::hex << (uint16_t)pc << std::endl;
	std::cout << "stack_ptr  -> 0x" << std::hex << (uint16_t)stack_ptr << std::endl;
	
	std::cout << "stat       -> 0x" << std::hex << (uint16_t)reg_status.raw << std::endl;
	std::cout << "   stat[B] -> " << (uint16_t)reg_status.B << std::endl;
	std::cout << "   stat[C] -> " << (uint16_t)reg_status.C << std::endl;
	std::cout << "   stat[D] -> " << (uint16_t)reg_status.D << std::endl;
	std::cout << "   stat[I] -> " << (uint16_t)reg_status.I << std::endl;
	std::cout << "   stat[U] -> " << (uint16_t)reg_status.reserved << std::endl;
	std::cout << "   stat[S] -> " << (uint16_t)reg_status.S << std::endl;
	std::cout << "   stat[V] -> " << (uint16_t)reg_status.V << std::endl;
	std::cout << "   stat[Z] -> " << (uint16_t)reg_status.Z << std::endl;

	std::cout << "addr_abs   -> 0x" << std::hex << (uint16_t)addr_abs << std::endl;
	std::cout << "addr_rel   -> 0x" << std::hex << (uint16_t)addr_rel << std::endl;
	std::cout << "M          -> 0x" << std::hex << (uint16_t)M << std::endl;
	std::cout << "opcode     -> 0x" << std::hex << (uint16_t)opcode << std::endl;
	std::cout << "inst_cnt   -> " << std::dec << this->instruction_counter << std::endl;

}

debug_cpu_state_dsc_st& mostech6502::getDebugCPUState()
{
	return this->debugCPUState;
}

std::string mostech6502::getPreExecuteStateAsStr()
{
	std::stringstream myStream;
	myStream << debugCPUState.pre_instruction_counter << "  ";
	myStream << std::uppercase << std::hex << (uint16_t)debugCPUState.pre_pc << "  ";
	myStream << debugCPUState.inst_name << "         ";
	myStream << "A:" << std::hex << (uint16_t)debugCPUState.pre_reg_acc << " ";
	myStream << "X:" << std::hex << (uint16_t)debugCPUState.pre_reg_x << " ";
	myStream << "Y:" << std::hex << (uint16_t)debugCPUState.pre_reg_y << " ";
	myStream << "P:" << std::hex << (uint16_t)debugCPUState.pre_reg_status << " ";
	myStream << "SP:" << std::hex << (uint16_t)debugCPUState.pre_stack_ptr << " ";
	myStream << std::hex << (uint16_t)debugCPUState.opcode << " ";
	myStream << std::hex << (uint16_t)debugCPUState.nxt_inst << " ";
	myStream << std::hex << (uint16_t)debugCPUState.nxt_nxt_inst << "     ";
	myStream << std::endl;
	return myStream.str();
}

std::string mostech6502::getPostExecuteStateAsStr()
{
	std::stringstream myStream;
	myStream << debugCPUState.pre_instruction_counter << "  ";
	myStream << std::uppercase << std::hex << (uint16_t)debugCPUState.post_pc << "  ";
	myStream << debugCPUState.inst_name << "         ";
	myStream << "A:" << std::hex << (uint16_t)debugCPUState.post_reg_acc << " ";
	myStream << "X:" << std::hex << (uint16_t)debugCPUState.post_reg_x << " ";
	myStream << "Y:" << std::hex << (uint16_t)debugCPUState.post_reg_y << " ";
	myStream << "P:" << std::hex << (uint16_t)debugCPUState.post_reg_status << " ";
	myStream << "SP:" << std::hex << (uint16_t)debugCPUState.post_stack_ptr << " ";
	myStream << std::hex << (uint16_t)debugCPUState.opcode << " ";
	myStream << std::hex << (uint16_t)debugCPUState.nxt_inst << " ";
	myStream << std::hex << (uint16_t)debugCPUState.nxt_nxt_inst << "     ";
	myStream << std::endl;
	return myStream.str();
}