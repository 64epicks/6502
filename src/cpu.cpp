#include <6502.hpp>
namespace CPU {

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

    if (addm == IMM)
        state = EXECUTE;
    else
        state = LOAD;
    
    av = PC;
    opcode = ((opcode & 7) >> 3) | (opcode & 3);
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
void CPU::exec()
{
    switch (opcode) {
        case OPCODE_ADC: return o_adc();     case OPCODE_AND: return o_and();
    }
}
}