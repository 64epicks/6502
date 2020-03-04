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

    state = FETCH;
    mode = RESET;
    cycle_count = 0;
    av = 0;

    vectorFetch = false;
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

void CPU::vectorPull(enum raise_modes mode)
{
    if (vectorFetch)
        return;
    unsigned char a;
    switch (mode) {
    case NMI:
        a = rd(VECTOR_NMI);
        break;
    case RESET:
        a = rd(VECTOR_RESET);
        break;
    case IRQ:
        a = rd(VECTOR_IRQ);
        break;
    }
    if (cycle_count == 0) {
        PC = a;
        ++cycle_count;
    }
    else {
        PC |= (a << 8);
        cycle_count = 0;
        vectorFetch = true;
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
        case 0x20: // JSR
            addm = ABS;
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
    if (!vectorFetch)
        return vectorPull(mode);
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
#define OPCODE_BIT_JSR 0b00100
#define OPCODE_EOR 0b01001
#define OPCODE_LSR 0b01010
#define OPCODE_NOP_INC 0b11110
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
void CPU::exec()
{
    switch (((opcode >> 3) & 0x1C) | (opcode & 3)) {
        case OPCODE_ADC: return o_adc();        case OPCODE_AND: return o_and();
        case OPCODE_ASL: return o_asl();        case OPCODE_JMP: return o_jmp();
        case OPCODE_EOR: return o_eor();        case OPCODE_LSR: return o_lsr();
        case OPCODE_ORA: return o_ora();        case OPCODE_DEC: return o_dec();
        case OPCODE_ROL: return o_rol();        case OPCODE_ROR: return o_ror();
        case OPCODE_SBC: return o_sbc();        case OPCODE_CMP: return o_cp(A);
        case OPCODE_CPX: return o_cp(X);        case OPCODE_CPY: return o_cp(Y);
        case OPCODE_LDA: return o_ld(A);        case OPCODE_LDX: return o_ld(X);
        case OPCODE_LDY: return o_ld(Y);        case OPCODE_STA: return o_st(A);
        case OPCODE_STX: return o_st(X);        case OPCODE_STY: return o_st(Y);

        case OPCODE_NOP_INC:
            if (addm == IMM)
                return o_nop();
            else
                return o_inc();
        case OPCODE_BIT_JSR:
            if (addm = ABS)
                return o_jsr();
            else
                return o_bit();
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
        push(PC & 0xFF);
        ++cycle_count;
    }
    else if (cycle_count == 1) {
        push((PC >> 8) & 0xFF);
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
}