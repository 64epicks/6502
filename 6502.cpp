/*
6502.cpp
Copyright (c) 2020 Jacob Paul <sixfour@64epicks.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <6502.hpp>
#include <string.h>
namespace CPU {
void CPU::variablesInit()
{
    memset(&P, 0, sizeof(P));

    A, X, Y = 0;
    S = 0xFD;

    P[INTERRUPT] = true;
    P[B0]        = true;
    P[B1]        = true;

    readByte = nullptr;
    writeByte = nullptr;

    state = SRESET;
    cycle_count = 0;
    av = 0;

    irqc = true;
}
void CPU::setRW(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char))
{
    CPU::readByte = readByte;
    CPU::writeByte = writeByte;
}
CPU::CPU()
{
    variablesInit();
}
CPU::CPU(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char))
{
    variablesInit();
    setRW(readByte, writeByte);
}

#define VECTOR_NMI   0xFFFA
#define VECTOR_RESET 0xFFFC
#define VECTOR_IRQ   0xFFFE
#define P_CHAR (P[CARRY] | (P[ZERO] << 1) | (P[INTERRUPT] << 2) | (P[DECIMAL] << 3) | (P[B0] << 4) | (P[B1] << 5) | (P[OVERFLOW] << 6) | (P[NEGATIVE] << 7))
void CPU::irq()
{
    if (!irqc)
        return;
    switch (cycle_count) {
    case 0:
        if (state == SNMI)
            cycle_count += 2;
        else
            ++cycle_count;
        break;
    case 1:
        ++cycle_count;
        if (state == SRESET)
            cycle_count += 2;
        break;
    case 2:
        push(PC >> 8);
        ++cycle_count;
        break;
    case 3:
        push(PC & 0xFF);
        ++cycle_count;
        break;
    case 4:
        push(P_CHAR | ((state == SNMI) << 4));
        cycle_count += 4;
        break;
    case 5:
        S -= 3;
        ++cycle_count;
        break;
    case 6:
        ++cycle_count;
        break;
    case 7:
        ++cycle_count;
        break;
    case 8:
        switch (state) {
        case SNMI:
            PC = rd(VECTOR_NMI);
            break;
        case SRESET:
            PC = rd(VECTOR_RESET);
            break;
        case SIRQ:
            PC = rd(VECTOR_IRQ);
            break;
        }
        ++cycle_count;
        break;
    case 9:
        switch (state) {
        case SNMI:
            PC = (rd(VECTOR_NMI+1) << 8) | PC;
            break;
        case SRESET:
            PC = (rd(VECTOR_RESET+1) << 8) | PC;
            break;
        case SIRQ:
            PC = (rd(VECTOR_IRQ+1) << 8) | PC;
            break;
        }
        cycle_count = 0;
        state = FETCH;
        irqc = false;
        break;
    }
}
void CPU::raise(enum raise_modes m)
{
    switch (m) {
    case NMI:
        if (P[INTERRUPT]) {
            step();
            irqc = true;
            state = SNMI;
        }
        break;
    case RESET:
        step();
        irqc = true;
        state = SRESET;
        break;
    case IRQ:
        step();
        irqc = true;
        state = SIRQ;
        break;
    }
}
void CPU::fetch()
{
    cycle_count = 0;
    opcode = rd(PC++);

    switch ((opcode >> 2) & 7) {
    case 0b010:
        addm = IMM;
        break;
    case 0b001:
        addm = ZP;
        break;
    case 0b101:
        addm = ZPX;
        break;
    case 0b011:
        addm = ABS;
        break;
    case 0b111:
        addm = ABSX;
        break;
    case 0b110:
        addm = ABSY;
        break;
    case 0b000:
        addm = INDX;
        break;
    case 0b100:
        addm = INDY;
        break;
    }

    // Exceptions
    switch (opcode) {
        case 0xE0: // CPX
            addm = IMM;
            break;
        case 0xC0: // CPY
            addm = IMM;
            break;
        case 0x6C: // JMP
            addm = IND;
            break;
        case 0x10: // Branch instructions
        case 0x30:
        case 0x50:
        case 0x70:
        case 0x90:
        case 0xB0:
        case 0xD0:
        case 0xF0:
        case 0x00: // BRK
        case 0xAA: // Register tranfer instructions
        case 0x8A:
        case 0xCA:
        case 0xE8:
        case 0xA8:
        case 0x98:
        case 0x88:
        case 0xC8:
        case 0x9A:
        case 0xBA:
        case 0x40: // RTI
        case 0x60: // RTS
        case 0x18: // Flag instructions
        case 0x38:
        case 0x58:
        case 0x78:
        case 0xB8:
        case 0xD8:
        case 0xF8:
            addm == IMM;
            break;
        default:
            break;
    }

    if (addm == IMM) {
        state = EXECUTE;
        ++PC;
    }
    else {
        state = LOAD;
    }
    
    av = PC;
    wv0 = PC & 0xFF;
    wv1 = (PC >> 8) & 0xFF;
}
void CPU::load()
{
    switch (addm) {
    case IMM:
        state = EXECUTE;
        exec();
        return;
    case ZP:
        return zp();
    case ZPX:
        return zpx();
    case ABS:
        return abs();
    case ABSX:
        return absx();
    case ABSY:
        return absy();
    case IND:
        return ind();
    case INDX:
        return indx();
    case INDY:
        return indy();
    }
}

void CPU::cycle()
{
    if (irqc)
        return irq();
    switch (state)
    {
    case FETCH:
        return fetch();
        break;
    case LOAD:
        return load();
        break;
    case EXECUTE:
        return exec();
        break;
    }
}

// Addressing mode functions
void CPU::zp()
{
    wv0 = rd(av);
    wv1 = 0;
    ++PC;
    state = EXECUTE;
}
void CPU::zpx()
{
    if (cycle_count == 0) {
        wv0 = rd(av) + X;
        wv1 = 0;
        ++cycle_count;
    }
    else {
        cycle_count = 0;
        ++PC;
        state = EXECUTE;
    }
}
void CPU::abs()
{
    if (cycle_count == 0) {
        wv0 = rd(av++);
        ++cycle_count;
    }
    else {
        wv1 = rd(av);
        cycle_count = 0;
        PC += 2;
        state = EXECUTE;

        if (opcode == 0x4C || opcode == 0x20)
            exec();
    }
}
void CPU::absx()
{
    if (cycle_count == 0) {
        wv0 = rd(av++);
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        wv1 = rd(av);
        unsigned char tmp = wv0;
        wv0 += X;
        if (wv0 < tmp) {
            ++wv1;
            ++cycle_count;
        }
        else {
            cycle_count = 0;
            PC += 2;
            state = EXECUTE;
        }
    }
    else {
        cycle_count = 0;
        PC += 2;
        state = EXECUTE;
    }
}
void CPU::absy()
{
    if (cycle_count == 0) {
        wv0 = rd(av++);
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        wv1 = rd(av);
        unsigned char tmp = wv0;
        wv0 += Y;
        if (wv0 < tmp) {
            ++wv1;
            ++cycle_count;
        }
        else {
            cycle_count = 0;
            PC += 2;
            state = EXECUTE;
        }
    }
    else {
        cycle_count = 0;
        PC += 2;
        state = EXECUTE;
    }
}
void CPU::ind()
{
    if (cycle_count == 0) {
        wv0 = rd(av++);
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        wv1 = rd(av);
        ++cycle_count;
    }
    else if (cycle_count == 2) {
        av = rd((wv1 << 8) | wv0++);
        ++cycle_count;
    }
    else {
        wv1 = rd((wv1 << 8) | wv0);
        wv0 = av;
        cycle_count = 0;
        PC += 2;
        state = EXECUTE;

        if (opcode == 0x4C)
            exec();
    }
}
void CPU::indx()
{
    if (cycle_count == 0) {
        av = rd(av);
        ++cycle_count;
    }
    else {
        av = (av + X) & 0xFF;
        cycle_count = 0;
        ++PC;
        addm = ABS;
    }
}
void CPU::indy()
{
    av = rd(av);
    ++PC;
    addm = ABSY;
}

// Instructions
#define OPCODE_ADC 0b01101
#define OPCODE_AND 0b00101
#define OPCODE_ASL 0b00010
#define OPCODE_BIT 0b00100
#define OPCODE_EOR 0b01001
#define OPCODE_LSR 0b01010
#define OPCODE_INC 0b11110
#define OPCODE_ORA 0b00001
#define OPCODE_ROL 0b00110
#define OPCODE_ROR 0b01110
#define OPCODE_SBC 0b11101
#define OPCODE_CMP 0b11001
#define OPCODE_CPX 0b11100
#define OPCODE_CPY 0b11000
#define OPCODE_DEC 0b11010
#define OPCODE_INC 0b11110
#define OPCODE_JMP 0b01000
#define OPCODE_JSR 0b00100
#define OPCODE_LDA 0b10101
#define OPCODE_LDX 0b10110
#define OPCODE_LDY 0b10100
#define OPCODE_STA 0b10001
#define OPCODE_STX 0b10010
#define OPCODE_STY 0b10000
#define OPCODE_BPL 0b00000
#define OPCODE_BVS 0b01100
void CPU::exec()
{
    switch (opcode) { // Special (usually implied) instructions
        case 0xEA: return o_nop();
        case 0x20: return o_jsr();
        case 0x10: return o_br(P[NEGATIVE], false);
        case 0x30: return o_br(P[NEGATIVE], true);
        case 0x50: return o_br(P[OVERFLOW], false);
        case 0x70: return o_br(P[OVERFLOW], true);
        case 0x90: return o_br(P[CARRY], false);
        case 0xB0: return o_br(P[CARRY], true);
        case 0xD0: return o_br(P[ZERO], false);
        case 0xF0: return o_br(P[ZERO], true);
        case 0x00: return o_brk();
        case 0xAA: return o_tr(X, A);
        case 0x8A: return o_tr(A, X);
        case 0xA8: return o_tr(Y, A);
        case 0x98: return o_tr(A, Y);
        case 0x9A: return o_tr(S, X);
        case 0xBA: return o_tr(X, S);
        case 0x48: return o_ph(A);
        case 0x68: return o_pl(A);
        case 0x08: return o_php();
        case 0x28: return o_plp();
        case 0xE8: return o_in(X);
        case 0xC8: return o_in(Y);
        case 0xCA: return o_de(X);
        case 0x88: return o_de(Y);
        case 0x40: return o_rti();
        case 0x60: return o_rts();
        case 0x18: return o_fl(P[CARRY], false);
        case 0x38: return o_fl(P[CARRY], true);
        case 0x58: return o_fl(P[INTERRUPT], false);
        case 0x78: return o_fl(P[INTERRUPT], true);
        case 0xB8: return o_fl(P[OVERFLOW], false);
        case 0xD8: return o_fl(P[DECIMAL], false);
        case 0xF8: return o_fl(P[DECIMAL], true);
        default:
            break;
    }
    switch (((opcode >> 3) & 0x1C) | (opcode & 3)) {
        case OPCODE_ADC: return o_adc();                    case OPCODE_AND: return o_and();
        case OPCODE_ASL: return o_asl();                    case OPCODE_STY: return o_st(Y);
        case OPCODE_EOR: return o_eor();                    case OPCODE_LSR: return o_lsr();
        case OPCODE_ORA: return o_ora();                    case OPCODE_DEC: return o_dec();
        case OPCODE_ROL: return o_rol();                    case OPCODE_ROR: return o_ror();
        case OPCODE_SBC: return o_sbc();                    case OPCODE_CMP: return o_cp(A);
        case OPCODE_CPX: return o_cp(X);                    case OPCODE_CPY: return o_cp(Y);
        case OPCODE_LDA: return o_ld(A);                    case OPCODE_LDX: return o_ld(X);
        case OPCODE_LDY: return o_ld(Y);                    case OPCODE_STA: return o_st(A);
        case OPCODE_STX: return o_st(X);                    case OPCODE_JMP: return o_jmp();
        case OPCODE_INC: return o_inc();                    case OPCODE_BIT: return o_bit();

    }
}

void CPU::o_adc()
{
    unsigned char p = rd((wv1 << 8) | wv0);

    signed short r = p + A + P[CARRY];
    P[CARRY] = r > 0xFF;
    P[OVERFLOW] = ~(A^p) & (A^p) & 0x80;
    A = r;
    flUpdate(A);
    state = FETCH;
}
void CPU::o_and()
{
    unsigned char p = rd((wv1 << 8) | wv0);
    A &= p;
    flUpdate(A);
    state = FETCH;
}
void CPU::o_asl()
{
    if (addm == IMM) {
        --PC;

        P[CARRY] = (A >> 7) & 1;
        A = A << 1;
        flUpdate(A);
        state = FETCH;
    }
    else {
        if (cycle_count == 0) {
            av = (wv1 << 8) | wv0;
            wv0 = rd(av);
            ++cycle_count;
        }
        else if (cycle_count == 1) {
            P[CARRY] = (wv0 >> 7) & 1;
            wv0 = wv0 << 1;
            flUpdate(A);
            ++cycle_count;
        }
        else {
            wr(av, wv0);
            cycle_count = 0;
            state = FETCH;
        }
    }
}
void CPU::o_bit()
{
    unsigned char p = rd((wv1 << 8) | wv0);
    p = A & p;

    P[OVERFLOW] = (p >> 6) & 1;
    flUpdate(p);
    state = FETCH;
}
void CPU::o_eor()
{
    unsigned char p = rd((wv1 << 8) | wv0);
    A = A ^ p;
    flUpdate(A);
    state = FETCH;
}
void CPU::o_lsr()
{
    if (addm == IMM) {
        --PC;

        P[CARRY] = A & 1;
        A = A >> 1;
        flUpdate(A);
        state = FETCH;
    }
    else {
        if (cycle_count == 0) {
            av = (wv1 << 8) | wv0;
            wv0 = rd(av);
            ++cycle_count;
        }
        else if (cycle_count == 1) {
            P[CARRY] = wv0 & 1;
            wv0 = wv0 >> 1;
            flUpdate(A);
            ++cycle_count;
        }
        else {
            wr(av, wv0);
            cycle_count = 0;
            state = FETCH;
        }
    }
}
void CPU::o_nop()
{
    --PC;
    state = FETCH;
}
void CPU::o_ora()
{
    unsigned char p = rd((wv1 << 8) | wv0);
    A |= p;
    flUpdate(A);
    state = FETCH;
}
void CPU::o_rol()
{
    if (addm == IMM) {
        --PC;
        bool tmp = P[CARRY];
        P[CARRY] = (A >> 7) & 1;
        A  = A << 1;
        A |= tmp;
        flUpdate(A);
        state = FETCH;
    }
    else {
        if (cycle_count == 0) {
            av = (wv1 << 8) | wv0;
            wv0 = rd(av);
            ++cycle_count;
        }
        else if (cycle_count == 1) {
            bool tmp = P[CARRY];
            P[CARRY] = (wv0 >> 7) & 1;
            wv0  = wv0 << 1;
            wv0 |= tmp;
            flUpdate(wv0);
            ++cycle_count;
        }
        else {
            wr(av, wv0);
            cycle_count = 0;
            state = FETCH;
        }
    }
}
void CPU::o_ror()
{
    if (addm == IMM) {
        --PC;
        bool tmp = P[CARRY];
        P[CARRY] = A & 1;
        A  = A >> 1;
        A |= (tmp << 7);
        flUpdate(A);
        state = FETCH;
    }
    else {
        if (cycle_count == 0) {
            av = (wv1 << 8) | wv0;
            wv0 = rd(av);
            ++cycle_count;
        }
        else if (cycle_count == 1) {
            bool tmp = P[CARRY];
            P[CARRY] = wv0 & 1;
            wv0  = wv0 >> 1;
            wv0 |= (tmp >> 7);
            flUpdate(wv0);
            ++cycle_count;
        }
        else {
            wr(av, wv0);
            cycle_count = 0;
            state = FETCH;
        }
    }
}
void CPU::o_sbc()
{
    unsigned char p = rd((wv1 << 8) | wv0);
    unsigned short r = (0x100 + A) - p;
    if (r < 0x100)
        P[CARRY] = false;
    P[OVERFLOW] = ~(A^p) & (A^p) & 0x80;
    A = r;
    flUpdate(A);
    state = FETCH;
}
void CPU::o_cp(unsigned char& reg)
{
    unsigned char p = rd((wv1 << 8) | wv0);
    P[CARRY] = reg >= p;
    P[ZERO] = reg == p;
    P[NEGATIVE] = reg >= 0x80;
    state = FETCH;
}
void CPU::o_dec()
{
    if (cycle_count == 0) {
        av = (wv1 << 8) | wv0;
        wv0 = rd(av);
        ++cycle_count;
    }
    else if (cycle_count) {
        --wv0;
        flUpdate(wv0);
        ++cycle_count;
    }
    else {
        wr(av, wv0);
        cycle_count = 0;
        state = FETCH;
    }
}
void CPU::o_inc()
{
    if (cycle_count == 0) {
        av = (wv1 << 8) | wv0;
        wv0 = rd(av);
        ++cycle_count;
    }
    else if (cycle_count) {
        ++wv0;
        flUpdate(wv0);
        ++cycle_count;
    }
    else {
        wr(av, wv0);
        cycle_count = 0;
        state = FETCH;
    }
}
void CPU::o_jmp()
{
    PC = (wv1 << 8) | wv0;
    state = FETCH;
}
void CPU::o_jsr()
{
    if (cycle_count == 0) {
        push((PC++) >> 8);
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        push(PC & 0xFF);
        ++cycle_count;
    }
    else {
        PC = (wv1 << 8) | wv0;
        cycle_count = 0;
        state = FETCH;
    }
}
void CPU::o_ld(unsigned char& reg)
{
    unsigned char p = rd((wv1 << 8) | wv0);
    reg = p;
    flUpdate(reg);
    state = FETCH;
}
void CPU::o_st(unsigned char& reg)
{
    wr(reg, (wv1 << 8) | wv0);
    state = FETCH;
}
void CPU::o_br(bool& flag, bool v)
{
    if (cycle_count == 0) {
        wv0 = rd((wv1 << 8) | wv0);
        if (flag == v) {
            ++cycle_count;
        } 
        else {
            state = FETCH;
        }
    }
    else {
        PC += wv0;
        cycle_count = 0;
        state = FETCH;
    }
}
void CPU::o_brk()
{
    irqc = true;
    state = SNMI;
    irq();
}
void CPU::o_tr(unsigned char& reg0, unsigned char reg1)
{
    --PC;
    reg0 = reg1;
    flUpdate(reg0);
    state = FETCH;
}
void CPU::o_in(unsigned char& reg0)
{
    --PC;
    ++reg0;
    flUpdate(reg0);
    state = FETCH;
}
void CPU::o_de(unsigned char& reg0)
{
    --PC;
    --reg0;
    flUpdate(reg0);
    state = FETCH;
}
void CPU::o_rti()
{
    if (cycle_count == 0) {
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        ++cycle_count;
    }
    else if (cycle_count == 2) {
        o_plp();

        ++cycle_count;
    }
    else if (cycle_count == 3) {
        PC = pop();
        ++cycle_count;
    }
    else {
        PC = (pop() << 8) | PC;
        cycle_count = 0;
        state = FETCH;
    }
}
void CPU::o_rts()
{
    if (cycle_count == 0) {
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        ++cycle_count;
    }
    else if (cycle_count == 2) {
        ++cycle_count;
    }
    else if (cycle_count == 3) {
        PC = pop();
        ++cycle_count;
    }
    else {
        PC = (pop() << 8) | PC;
        ++PC;
        cycle_count = 0;
        state = FETCH;
    }
}
void CPU::o_fl(bool& reg0, bool value)
{
    reg0 = value;
    state = FETCH;
}
void CPU::o_ph(unsigned char value)
{
    --PC;
    push(value);
    state = FETCH;
}
void CPU::o_pl(unsigned char& reg0)
{
    --PC;
    reg0 = pop();
    state = FETCH;
}
void CPU::o_php()
{
    --PC;
    push(P_CHAR);
    state = FETCH;
}
void CPU::o_plp()
{
    --PC;
    unsigned char p = pop();

    P[CARRY]        = p & 1;
    P[ZERO]         = (p >> 1) & 1;
    P[INTERRUPT]    = (p >> 2) & 1;
    P[DECIMAL]      = (p >> 3) & 1;
    P[B0]           = (p >> 4) & 1;
    P[B1]           = (p >> 5) & 1;
    P[OVERFLOW]     = (p >> 6) & 1;
    P[NEGATIVE]     = (p >> 7) & 1;
    state = FETCH;
}
}