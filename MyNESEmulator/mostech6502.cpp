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

mostech6502::mostech6502() {

    _pc = 0x0000;
    _acc = 0x00;
    _x = 0x00;
    _y = 0x00;
    _stackPtr = 0x00;
    _status.raw = 0x00;

    _M = 0x00;

    _cpuCycleCounter = 0;
    _cycles = 0;
    _instructionCounter = 0;

    _addrAbs = 0x0000;
    _addrRel = 0x0000;

    _nmiOccurred = false;
    _irqOccurred = false;

    _cpuCycleCounter = 0;

    _instructionLut = { // Check: https://www.masswerk.at/6502/6502_instruction_set.html#PLA
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

mostech6502::~mostech6502() {
#ifdef CPU_FILE_LOG
    cpuLogFile.close();
#endif
}



/**********************************************************/
/************ Addressing modes ****************************/
/**********************************************************/

//  Immediate
uint8_t mostech6502::imm() {
    _addrAbs = _pc++; // The address of the operand is the address of the next instruction, which is in itself the operand!
    return 0;
}

// Absolute
uint8_t mostech6502::abs() {
    _addrAbs = read(_pc++); // LO part
    _addrAbs |= read(_pc++) << 8; // HI part
    return 0;
}

// Absolute X
uint8_t mostech6502::absx() {
    _addrAbs = read(_pc++); // LO part
    uint16_t hi = read(_pc++) << 8;
    _addrAbs |= hi; // HI part
    _addrAbs += _x; // Add X

    // If the page boundary is crossed, an extra cycle is needed
    if ((_addrAbs & 0xFF00) != hi)
        return 1;
    else
        return 0;

}

// Absolute Y
uint8_t mostech6502::absy() {
    _addrAbs = read(_pc++); // LO part
    uint16_t hi = read(_pc++) << 8;
    _addrAbs |= hi; // HI part
    _addrAbs += _y; // Add Y

    // If the page boundary is crossed, an extra cycle is needed
    if ((_addrAbs & 0xFF00) != hi)
        return 1;
    else
        return 0;

}

// Zero page
uint8_t mostech6502::zpg() {

    _addrAbs = read(_pc);
    _addrAbs &= CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK; // Will do even though it's unneeded
    _pc++;
    return 0;

}

// Zero page X
uint8_t mostech6502::zpgx() {
    _addrAbs = read(_pc) + _x; // M will be located at memory[pc+1 + x]
    _addrAbs &= CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK; // Protect against reg_x breaking boundary
    _pc++;
    return 0;
}

// Zero page Y
uint8_t mostech6502::zpgy() {

    _addrAbs = read(_pc) + _y; // M will be located at memory[pc+1 + y]
    _addrAbs &= CPU_ADDR_SPACE_RAM_ZERO_PAGE_MASK; // Protect against reg_y breaking boundary
    _pc++;
    return 0;

}

//  Implied
uint8_t mostech6502::imp() {
    _M = _acc; // Dunno why, apparently a wa for _pha()
    return 0;
}

//  Accumulator
uint8_t mostech6502::acc() {
    _M = _acc;
    return 0;
}

// Indirect
uint8_t mostech6502::ind() {

    uint16_t ptr_lo = read(_pc++);
    uint16_t ptr_hi = read(_pc++);

    uint16_t ptr = (ptr_hi << 8) | ptr_lo;

    if (ptr_lo != 0x00FF) {

        // Normal behavior
        _addrAbs = read(ptr); // LO part
        _addrAbs |= read(ptr + 1) << 8; // HI part

    }
    else {

        // Simulate an existing hardware bug!
        // Specifically, the page boundary hardware bug
        _addrAbs = read(ptr); // LO part
        _addrAbs |= read(ptr & 0xFF00) << 8; // HI part

    }

    return 0;

}

// Indirect X
// Strictly read from page 0
// *(mem( inst[pc+1] + x ))
uint8_t mostech6502::indx() {

    uint16_t ptr = read(_pc++);

    uint16_t lo_addr = (ptr + _x) & 0xFF;
    uint16_t hi_addr = (ptr + 1 + _x) & 0xFF;

    uint8_t lo = read(lo_addr);
    uint8_t hi = read(hi_addr);

    _addrAbs = (hi << 8) | lo;

    return 0;
}

// Indirect Y
// Strictly read from page 0
// *(mem( inst[pc+1] + y ))
uint8_t mostech6502::indy() {

    uint16_t ptr = (uint16_t)read(_pc++) & 0xFF;

    uint8_t lo = read(ptr);
    uint8_t hi = read((ptr + 1) & 0xFF); // Read only first page

    _addrAbs = (hi << 8) | lo;
    _addrAbs += _y;

    if ((_addrAbs & 0xFF00) != (hi << 8)) {
        return 1;
    }
    else {
        return 0;
    }
}

// Relative address
uint8_t mostech6502::rel() {
    // Read from the next instruction
    // the address delta, which will be a jump
    // between -128 and +127
    _addrRel = read(_pc++);
    if (_addrRel & 0x0080)
        _addrRel |= 0xFF00;
    return 0;
}

/**********************************************************/
/************ Instructions ********************************/
/**********************************************************/

uint8_t mostech6502::_xxx() {
    // What to do, what to do...
    return 0;
}

uint8_t mostech6502::_adc() {
    _FETCH();

    uint16_t res = (uint16_t)_acc + (uint16_t)_M + (uint16_t)_status.C;

    _status.C = res > 0xFF; // Set carry if overflow
    _status.Z = (res & 0x00FF) == 0x00; // Set zero flag if zero
    _status.S = (res & 0x0080) != 0; // Get signed bit

    // V = ~(A^M) & (A^R)
    _status.V = ((~((uint16_t)_acc ^ (uint16_t)_M) & ((uint16_t)_acc ^ res)) & 0x0080) != 0;

    // Load into acc
    _acc = res & 0xFF;

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
uint8_t mostech6502::_sbc() {
    // Basically the same implementation of ADC, just inverting the bits of M

    _FETCH();

    _M ^= 0xFF; // Invert

    uint16_t op_result = (uint16_t)_acc + (uint16_t)_M + (uint16_t)_status.C;

    _status.C = op_result > 0xFF; // Set carry if overflow
    _status.Z = (op_result & 0x00FF) == 0x00; // Set zero flag if zero
    _status.S = (op_result & 0x0080) != 0; // Get signed bit

    // V = ~(A^M) & (A^R)
    _status.V = ((~((uint16_t)_acc ^ (uint16_t)_M) & ((uint16_t)_acc ^ op_result)) & 0x0080) != 0;

    // Load into acc
    _acc = op_result & 0xFF;

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

uint8_t mostech6502::_and() {
    _FETCH();

    uint8_t res = _M & _acc;

    _status.S = (res & 0x80) != 0;
    _status.Z = res == 0x00;

    _acc = res;

    return 1;
}

// Arithmetic shift left
uint8_t mostech6502::_asl() {
    _FETCH();

    // Save msb, set carry
    _status.C = (_M & 0x80) != 0;

    // Shift
    _M <<= 1;

    // Set sign
    _status.S = (_M & 0x80) != 0;

    // Zero
    _status.Z = _M == 0x00;

    // Depending on the addressing mode, store
    // either in the accumulator or in back in memory
    if (_instructionLut[_opcode].addr_mode == &mostech6502::imp) {
        _acc = _M;
    }
    else {
        write(_addrAbs, _M);
    }

    return 0;
}

// Branch if carry clear
uint8_t mostech6502::_bcc() {
    if (_status.C == 0) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

// Branch if carry set
uint8_t mostech6502::_bcs() {
    if (_status.C) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

uint8_t mostech6502::_beq() {
    if (_status.Z) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

// Test bits in memory with accumulator
// S and V are always set according to the value
// fetched from memory. Z depends on the result
// of the AND operation.
uint8_t mostech6502::_bit() {
    _FETCH();

    uint8_t temp = _acc & _M;

    // Signed flag
    _status.S = (_M & 0x80) != 0;

    // Overflow flag
    _status.V = (_M & 0x40) != 0;

    // Zero flag
    _status.Z = temp == 0x00;

    return 0;
}

uint8_t mostech6502::_bmi() {
    if (_status.S) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

uint8_t mostech6502::_bne() {
    if (_status.Z == 0) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

uint8_t mostech6502::_bpl() {
    if (_status.S == 0) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

// Break
uint8_t mostech6502::_brk() {
    // First off, increase pc;
    // this new address of pc is the one we'll be coming
    // back to
    _pc++;

    // Save pc in stack
    _STACK_PUSH((_pc >> 8));
    _STACK_PUSH((_pc & 0x00FF));

    // Set break
    _status.B = 1;

    // Write status reg in stack too
    _STACK_PUSH(_status.raw);

    // Interrupt
    // I guess the method here is to just overwrite pc
    // with the address of the interrupt routine
    _pc = (uint16_t)read(IRQ_ADDR) | ((uint16_t)read(IRQ_ADDR + 1) << 8);

    return 0;
}

uint8_t mostech6502::_bvc() {
    if (_status.V == 0) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

uint8_t mostech6502::_bvs() {
    if (_status.V) {

        _cycles++;

        // If we've crossed the page boundary,
        // the whole thing takes one more cycle
        _addrAbs = _addrRel + _pc;
        if ((_addrAbs & 0xFF00) != (_pc & 0xFF00))
            _cycles++;

        _pc = _addrAbs;

    }
    return 0;
}

// Clear carry flag
uint8_t mostech6502::_clc() {
    _status.C = 0;
    return 0;
}

// Clear decimal flag
uint8_t mostech6502::_cld() {
    _status.D = 0;
    return 0;
}

// Clear interrupt flag
uint8_t mostech6502::_cli() {
    _status.I = 0;
    return 0;
}

// Clear overflow flag
uint8_t mostech6502::_clv() {
    _status.V = 0;
    return 0;
}

uint8_t mostech6502::_cmp() {
    _FETCH();

    uint16_t temp = (uint16_t)_acc - (uint16_t)_M;

    _status.C = _acc >= _M;
    _status.S = (temp & 0x80) != 0;
    _status.Z = (temp & 0x00FF) == 0x00;

    return 1;
}

uint8_t mostech6502::_cpx() {
    _FETCH();

    uint16_t temp = (uint16_t)_x - (uint16_t)_M;

    _status.C = _x >= _M;
    _status.S = (temp >> 7) & 0x01;
    _status.Z = (temp & 0x00FF) == 0;

    return 0;
}

uint8_t mostech6502::_cpy() {
    _FETCH();

    uint16_t temp = (uint16_t)_y - (uint16_t)_M;

    _status.C = _y >= temp;
    _status.S = (temp >> 7) & 0x01;
    _status.Z = (temp & 0x00FF) == 0;

    return 0;
}

// Decrement value at memory location
uint8_t mostech6502::_dec() {
    _FETCH();

    uint16_t temp = ((uint16_t)_M - 1) & 0xFF;

    _status.S = (temp >> 7) & 0x01;
    _status.Z = (temp & 0x00FF) == 0x00;

    write(_addrAbs, (uint8_t)temp);

    return 0;
}

uint8_t mostech6502::_dex() {
    _x--;
    _status.S = (_x & 0x80) != 0;
    _status.Z = (_x & 0xFF) == 0x00;
    return 0;
}

uint8_t mostech6502::_dey() {
    _y--;
    _status.S = (_y & 0x80) != 0;
    _status.Z = (_y & 0xFF) == 0x00;
    return 0;
}

// Bitwise logic XOR
// Function: acc = acc xor M
uint8_t mostech6502::_eor() {
    _FETCH();

    _acc ^= _M;

    _status.S = (_acc & 0x80) != 0;
    _status.Z = (_acc & 0xFF) == 0x00;

    return 1;

}

uint8_t mostech6502::_inc() {
    _FETCH();

    _M += 1;

    _status.S = (_M & 0x80) != 0;
    _status.Z = _M == 0x00;

    write(_addrAbs, _M);

    return 0;
}

uint8_t mostech6502::_inx() {
    _x += 1;

    _status.S = (_x & 0x80) != 0;
    _status.Z = _x == 0x00;

    return 0;
}

uint8_t mostech6502::_iny() {
    _y += 1;

    _status.S = (_y & 0x80) != 0;
    _status.Z = _y == 0x00;

    return 0;
}

uint8_t mostech6502::_jmp() {
    _pc = _addrAbs;
    return 0;
}

// Jump to a subroutine
uint8_t mostech6502::_jsr() {
    _pc = _pc - 1;

    uint8_t lo = _pc & 0xFF;
    uint8_t hi = (_pc >> 8) & 0xFF;

    // Push to stack
    _STACK_PUSH(hi);
    _STACK_PUSH(lo);

    _pc = _addrAbs;

    return 0;
}

// Load the accumulator
uint8_t mostech6502::_lda() {
    _FETCH();

    _acc = _M;

    _status.S = (_acc & 0x80) != 0;
    _status.Z = _acc == 0x00;

    return 1;
}

uint8_t mostech6502::_ldx() {
    _FETCH();

    _x = _M;

    _status.S = (_x & 0x80) != 0;
    _status.Z = _x == 0x00;

    return 1;
}

uint8_t mostech6502::_ldy() {
    _FETCH();

    _y = _M;

    _status.S = (_y & 0x80) != 0;
    _status.Z = _y == 0x00;

    return 1;
}

uint8_t mostech6502::_lsr() {
    _FETCH();

    _status.C = _M & 0x1;

    _M >>= 1;

    _status.S = 0;// (M & 0x80) != 0; --> Will always be 0
    _status.Z = _M == 0x00;

    // Depending on the addressing mode, store
    // either in the accumulator or in back in memory
    if (_instructionLut[_opcode].addr_mode == &mostech6502::imp) {
        _acc = _M;
    }
    else {
        write(_addrAbs, _M);
    }

    return 0;
}

uint8_t mostech6502::_nop() {
    // Well, not all nops are created equal.
    // Need to look for more info on this
    switch (_opcode) {
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
uint8_t mostech6502::_ora() {
    _FETCH();
    _acc |= _M;
    _status.S = (_acc & 0x80) != 0;
    _status.Z = _acc == 0x00;
    return 0;
}

// Push accumulator to stack
uint8_t mostech6502::_pha() {
    _STACK_PUSH(_acc);
    return 0;
}

// Push status register to stack
uint8_t mostech6502::_php() {
    // Apparently I gotta set two flags before the push:
    // Break flag and reserved. Why and why?
    _status.B = 1;
    _status.reserved = 1;
    _STACK_PUSH(_status.raw);
    // Reset flags to original state
    _status.B = 0;
    _status.reserved = 0;
    return 0;
}

// Pop accumulator from stack
uint8_t mostech6502::_pla() {
    // Increment; keep in mind that the ptr always points to the next position to be written
    _STACK_POP(_acc);
    // Set flags
    _status.S = (_acc & 0x80) != 0;
    _status.Z = _acc == 0x00;
    return 0;
}

// Pop status register from stack
uint8_t mostech6502::_plp() {
    // Increment; keep in mind that the ptr always points to the next position to be written
    _STACK_POP(_status.raw);
    // Set flags
    _status.reserved = 1; // Wft?
    return 0;
}

// Rotate one bit left
uint8_t mostech6502::_rol() {
    _FETCH();
    // Before doing the shift, save bit 7 (msb)
    uint8_t saved_msb = (_M >> 7) & 0x1;
    // Do op
    _M = (_M << 1) | (_status.C ? 0x01 : 0x00);
    // Set flags
    _status.C = saved_msb;
    _status.Z = _M == 0x00;
    _status.S = (_M & 0x80) != 0;
    // Depending on the addressing mode, store
    // either in the accumulator or in back in memory
    if (_instructionLut[_opcode].addr_mode == &mostech6502::imp) {
        _acc = _M;
    }
    else {
        write(_addrAbs, _M);
    }
    return 0;
}

// Rotate one bit right
uint8_t mostech6502::_ror() {
    _FETCH();
    // Before doing the shift, register bit 7 in the carry
    uint8_t saved_lsb = _M & 0x1;
    // Do op
    _M = (_M >> 1) | (_status.C ? 0x80 : 0x00);
    // Set flags
    _status.C = saved_lsb;
    _status.Z = _M == 0x00;
    _status.S = (_M & 0x80) != 0;
    // Depending on the addressing mode, store
    // either in the accumulator or in back in memory
    if (_instructionLut[_opcode].addr_mode == &mostech6502::imp) {
        _acc = _M;
    }
    else {
        write(_addrAbs, _M);
    }
    return 0;
}

// Return from interrupt
uint8_t mostech6502::_rti() {
    // Recover status register
    _STACK_POP(_status.raw);

    // Again, this is unknown to me...
    _status.B = 0;
    _status.reserved = 0;

    // Recover Program Counter
    uint8_t lo, hi;
    _STACK_POP(lo);
    _STACK_POP(hi);

    _pc = (hi << 8) | lo;

    return 0;
}

// Return from subroutine
uint8_t mostech6502::_rts() {
    // Recover Program Counter
    uint8_t lo, hi;
    _STACK_POP(lo);
    _STACK_POP(hi);

    _pc = (hi << 8) | lo;
    // Advance PC to execute next instruction
    // When going to a subroutine, we store the last address executed,
    // not the next to execute.
    _pc++;
    return 0;
}

// Set decimal flag
uint8_t mostech6502::_sec() {
    _status.C = 1;
    return 0;
}

// Set decimal flag
uint8_t mostech6502::_sed() {
    _status.D = 1;
    return 0;
}

// Set interrupt flag
uint8_t mostech6502::_sei() {
    _status.I = 1;
    return 0;
}

// Store accumulator at address
uint8_t mostech6502::_sta() {
    write(_addrAbs, _acc);
    return 0;
}

// Store X register at address
uint8_t mostech6502::_stx() {
    write(_addrAbs, _x);
    return 0;
}

// Store Y register at address
uint8_t mostech6502::_sty() {
    write(_addrAbs, _y);
    return 0;
}

// Transfer accumulator to X register
uint8_t mostech6502::_tax() {
    _x = _acc;
    // Set flags
    _status.Z = _x == 0x00;
    _status.S = (_x & 0x80) != 0;
    return 0;
}

// Transfer accumulator to Y register
uint8_t mostech6502::_tay() {
    _y = _acc;
    // Set flags
    _status.Z = _y == 0x00;
    _status.S = (_y & 0x80) != 0;
    return 0;
}

// Transfer stack pointer to X register
uint8_t mostech6502::_tsx() {
    _x = _stackPtr;
    // Set flags
    _status.Z = _x == 0x00;
    _status.S = (_x & 0x80) != 0;
    return 0;
}

// Transfer X register to accumulator
uint8_t mostech6502::_txa() {
    _acc = _x;
    // Set flags
    _status.Z = _acc == 0x00;
    _status.S = (_acc & 0x80) != 0;
    return 0;
}

// Transfer X register to stack pointer
uint8_t mostech6502::_txs() {
    _stackPtr = _x;
    return 0;
}

// Transfer Y register to accumulator
uint8_t mostech6502::_tya() {
    _acc = _y;
    // Set flags
    _status.Z = _acc == 0x00;
    _status.S = (_acc & 0x80) != 0;
    return 0;
}




/**********************************************************/
/************ Basic ops ***********************************/
/**********************************************************/

uint8_t mostech6502::fetch() {
    return _bus->cpuRead(_pc++, false);
}

uint8_t mostech6502::read(uint16_t addr) {
    return _bus->cpuRead(addr, false);
}

void mostech6502::write(uint16_t addr, uint8_t data) {
    _bus->cpuWrite(addr, data);
}

void mostech6502::advanceClock() {

    if (_cycles == 0) {

        _DEBUG_FILL_PRE_CPU_STATE();

        // pc should always point at the next operation code
        _opcode = read(_pc);

        // Leave pc ready to fetch the first instruction argument, if any
        _pc++;

        // Get number of cycles for the current instruction
        _cycles = _instructionLut[_opcode].cycles;

        // Fetch intermediate data (M), accumulate an extra cycle if needed
        uint8_t extra_cycle = (this->*(_instructionLut[_opcode].addr_mode))();

        // Perform operation associated to this instruction
        uint8_t allows_extra_cycle = (this->*(_instructionLut[_opcode].op))();

        _cycles += (extra_cycle & allows_extra_cycle);

        // Apparently, set Unused flag to 1 every instruction execution
        _status.reserved = 1;

        // Increment global instruction counter
        _instructionCounter++;

        _DEBUG_FILL_POST_CPU_STATE(_instructionLut[_opcode].name, _instructionLut[_opcode].cycles, (extra_cycle & allows_extra_cycle), read(_debugCPUState.pre_pc + 1), read(_debugCPUState.pre_pc + 2));

        _LOG(getPreExecuteStateAsStr());

        _justFetched = true;

    }
    else {
        _justFetched = false;
    }

    _cycles--;

    // Increment global clock count
    _cpuCycleCounter++;

}

bool mostech6502::isInstructionComplete() {
    return _cycles == 0;
}

void mostech6502::nmi() {

    uint8_t pc_lo = _pc & 0xFF;
    uint8_t pc_hi = (_pc >> 8) & 0xFF;

    // Add the PC to the stack
    _STACK_PUSH(pc_hi);
    _STACK_PUSH(pc_lo);

    // Save the status register
    _status.B = 1;
    _status.I = 1; // No more interrupts!
    _status.reserved = 1;
    _STACK_PUSH(_status.raw);

    // Read the new program counter
    // The reset address is located at address 0xFFFC
    _addrAbs = NMI_ADDR;
    pc_lo = read(_addrAbs);
    pc_hi = read(_addrAbs + 1);
    _pc = (pc_hi << 8) | pc_lo; // New program counter address

    // NMIs take 7 cycles
    _cycles = 8;

}

void mostech6502::irq() {

    if (_status.I == 0) {

        uint8_t pc_lo = _pc & 0xFF;
        uint8_t pc_hi = (_pc >> 8) & 0xFF;

        // Add the PC to the stack
        _STACK_PUSH(pc_hi);
        _STACK_PUSH(pc_lo);

        // Save the status register
        _status.B = 1;
        _status.I = 1; // No more interrupts!
        _status.reserved = 1;
        _STACK_PUSH(_status.raw)

        // Read the new program counter
        // The reset address is located at address 0xFFFC
        _addrAbs = IRQ_ADDR;
        pc_lo = read(_addrAbs);
        pc_hi = read(_addrAbs + 1);
        _pc = (pc_hi << 8) | pc_lo; // New program counter address

        // IRQs take 7 cycles
        _cycles = 7;

    }

}

void mostech6502::reset() {

    // The reset address is located at address 0xFFFC
    _addrAbs = RESET_ADDR;

    uint16_t lo = read(_addrAbs);
    uint16_t hi = read(_addrAbs + 1);

#if defined(CPU_DEBUG_MODE) || defined(FORCE_START_PC)
    _pc = CPU_DEBUG_MODE_START_PC;
#else
    _pc = (hi << 8) | lo; // New program counter address
#endif

    // Reset (zero) main regs
    _x = 0x00;
    _y = 0x00;
    _acc = 0x00;
    _status.raw = 0x00;
    _status.reserved = 1; // Set reserved bit to 1, God knows why...

    _M = 0x00;
    _addrAbs = 0x0000;
    _addrRel = 0x0000;
    _stackPtr = CPU_ADDR_SPACE_STACK_INITIAL_OFFSET;

    // Reset will take 8 cycles to execute
    _cycles = RESET_TICKS;

}

//////////////////////////////////
//////////// DEBUG ///////////////
//////////////////////////////////

std::string mostech6502::getAddrMode(uint8_t opcode) {

    std::string addrMode = "?";

    if (_instructionLut[opcode].addr_mode == &mostech6502::imp) {
        addrMode = "imp";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::imm) {
        addrMode = "imm";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::zpg) {
        addrMode = "zpg";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::zpgx) {
        addrMode = "zpgx";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::zpgy) {
        addrMode = "zpgy";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::indx) {
        addrMode = "indx";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::indy) {
        addrMode = "indy";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::abs) {
        addrMode = "abs";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::absx) {
        addrMode = "absx";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::absy) {
        addrMode = "absy";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::ind) {
        addrMode = "ind";
    }
    else if (_instructionLut[opcode].addr_mode == &mostech6502::rel) {
        addrMode = "rel";
    }

    return addrMode;

}

void mostech6502::attachBus(Bus* bus) {
    this->_bus = bus;
}

void mostech6502::printCpuState() {

    std::cout << "*** CPU STATE ***" << std::endl;
    std::cout << "x          -> 0x" << std::hex << (uint16_t)_x << std::endl;
    std::cout << "y          -> 0x" << std::hex << (uint16_t)_y << std::endl;
    std::cout << "acc        -> 0x" << std::hex << (uint16_t)_acc << std::endl;

    std::cout << "pc         -> 0x" << std::hex << (uint16_t)_pc << std::endl;
    std::cout << "stack_ptr  -> 0x" << std::hex << (uint16_t)_stackPtr << std::endl;

    std::cout << "stat       -> 0x" << std::hex << (uint16_t)_status.raw << std::endl;
    std::cout << "   stat[B] -> " << (uint16_t)_status.B << std::endl;
    std::cout << "   stat[C] -> " << (uint16_t)_status.C << std::endl;
    std::cout << "   stat[D] -> " << (uint16_t)_status.D << std::endl;
    std::cout << "   stat[I] -> " << (uint16_t)_status.I << std::endl;
    std::cout << "   stat[U] -> " << (uint16_t)_status.reserved << std::endl;
    std::cout << "   stat[S] -> " << (uint16_t)_status.S << std::endl;
    std::cout << "   stat[V] -> " << (uint16_t)_status.V << std::endl;
    std::cout << "   stat[Z] -> " << (uint16_t)_status.Z << std::endl;

    std::cout << "addr_abs   -> 0x" << std::hex << (uint16_t)_addrAbs << std::endl;
    std::cout << "addr_rel   -> 0x" << std::hex << (uint16_t)_addrRel << std::endl;
    std::cout << "M          -> 0x" << std::hex << (uint16_t)_M << std::endl;
    std::cout << "opcode     -> 0x" << std::hex << (uint16_t)_opcode << std::endl;
    std::cout << "inst_cnt   -> " << std::dec << this->_instructionCounter << std::endl;

}

debug_cpu_state_dsc_st& mostech6502::getDebugCPUState() {
    return this->_debugCPUState;
}

std::string mostech6502::getPreExecuteStateAsStr() {
    std::stringstream myStream;
    myStream << _debugCPUState.instructionCounter << "  ";
    myStream << std::uppercase << std::hex << (uint16_t)_debugCPUState.pre_pc << "  ";
    myStream << _debugCPUState.instructionName << "         ";
    myStream << "A:" << std::hex << (uint16_t)_debugCPUState.pre_acc << " ";
    myStream << "X:" << std::hex << (uint16_t)_debugCPUState.pre_x << " ";
    myStream << "Y:" << std::hex << (uint16_t)_debugCPUState.pre_y << " ";
    myStream << "P:" << std::hex << (uint16_t)_debugCPUState.pre_status << " ";
    myStream << "SP:" << std::hex << (uint16_t)_debugCPUState.pre_stackPtr << " ";
    myStream << std::hex << (uint16_t)_debugCPUState.opcode << " ";
    myStream << std::hex << (uint16_t)_debugCPUState.pcPlus1 << " ";
    myStream << std::hex << (uint16_t)_debugCPUState.pcPlus2 << "     ";
    myStream << std::endl;
    return myStream.str();
}

std::string mostech6502::getPostExecuteStateAsStr() {
    std::stringstream myStream;
    myStream << _debugCPUState.instructionCounter << "  ";
    myStream << std::uppercase << std::hex << (uint16_t)_debugCPUState.post_pc << "  ";
    myStream << _debugCPUState.instructionName << "         ";
    myStream << "A:" << std::hex << (uint16_t)_debugCPUState.post_acc << " ";
    myStream << "X:" << std::hex << (uint16_t)_debugCPUState.post_x << " ";
    myStream << "Y:" << std::hex << (uint16_t)_debugCPUState.post_y << " ";
    myStream << "P:" << std::hex << (uint16_t)_debugCPUState.post_status << " ";
    myStream << "SP:" << std::hex << (uint16_t)_debugCPUState.post_stackPtr << " ";
    myStream << std::hex << (uint16_t)_debugCPUState.opcode << " ";
    myStream << std::hex << (uint16_t)_debugCPUState.pcPlus1 << " ";
    myStream << std::hex << (uint16_t)_debugCPUState.pcPlus2 << "     ";
    myStream << std::endl;
    return myStream.str();
}