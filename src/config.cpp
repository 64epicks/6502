#include <6502.hpp>
#include <string.h>

namespace CPU {
void CPU::variablesInit()
{
    memset(&pinout, 0, sizeof(pinout));
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
    cycle_count, opcode, wv0, wv1 = 0;
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

}