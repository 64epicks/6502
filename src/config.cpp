#include <6502.hpp>
#include <string.h>

namespace CPU {
void CPU::variablesInit()
{
    memset(&pinout, 0, sizeof(pinout));
}
CPU::CPU()
{

}
}