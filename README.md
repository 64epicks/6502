# 6502
6502 is a cycle-perfect emulator of the 6502 microprocessor in just ~1000 lines of code. The library only consist of a header and a source file, which makes it easy to integrate into other projects which rely on a 6502 CPU (like a NES emulator for example). lib6502 can also be configured and compiled automatically if the project uses CMake.

# Building
## Standalone
Since lib6502 only has one file, it can be compiled without any overlying toolchain; simply compile with `g++ -o 6502.o 6502.cpp -I /path/where/6502.hpp/resides -c -O2`. This will create the object file `6502.o` that the project can link with.
## CMake
Start by adding 6502 to your project: `add_subdirectory(path/to/6502)`. The project can then link with lib6502 with for example `target_include_directories(TARGET PRIVATE ${6502_DIRECTORIES})` and `target_link_libraries(TARGET ${6502_LIBRARIES})`.

# Usage
Using lib6502 is very simple since everything is contained in the CPU::CPU class. Creating a new instance is as easy as creating a new class:
```cpp
#include <6502.hpp>

int main()
{
    CPU::CPU processor();
}
```
This will initialize a new instance with default values.

To keep lib6502 as flexible as possible, the emulator does not handle any of the memory and outsources it for the program to handle. Because of that, the program needs to configure read and write functions for the CPU. This can be done with the `setRW` functions or the constructor itself.
```cpp
#include <6502.hpp>

unsigned char read(unsigned short address)
{
    // * Return the correct value based on the address *
}
void write(unsigned short address, unsigned char value)
{
    // * Write [value] to [address] *
}

int main()
{
    // Create new CPU class with default values
    CPU::CPU processor0();
    // Set the correct read-write functions
    processor0.setRW(read, write);
    // Or set them directly in the constructor
    CPU::CPU processor1(read, write);
}
```
Since lib6502 emulates the 6502 down to the cycle, the program can run the processor either cycle-by-cycle, instruction-by-instruction, or millisecond-by-millisecond. The library comes with functions for all these types:
```cpp
#include <6502.hpp>

unsigned char read(unsigned short address)
{
    // * Return the correct value based on the address *
}
void write(unsigned short address, unsigned char value)
{
    // * Write [value] to [address] *
}

int main()
{
    CPU::CPU processor(read, write);

    // Execute a single cycle
    processor.cycle();
    // Execute 10 cycles
    processor.cycles(10);
    // Complete a full instruction
    processor.step();
    // Execute 10 instructions
    processor.steps(10);

    // Execute indefinitely at default speed of ~1.79 MHz
    processor.run();
    // Execute indefinitely at a speed of 100 Hz
    processor.run(100UL);
    // Execute for 100 milliseconds
    processor.run(100ULL);
    // Execute for 100 milliseconds at a speed of 100 Hz
    processor.run(100ULL, 100UL);
}
```