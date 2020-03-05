# 6502
6502 is a cycle-perfect emulator of the 6502 microprocessor in just ~1000 lines of code. The library only consist of a header and a source file, which makes it easy to integrate into other projects which rely on a 6502 CPU (like a NES emulator for example). lib6502 can also be configured and compiled automatically if the project uses CMake.

# Building
## Standalone
Since lib6502 only has one file, it can be compiled without any overlying toolchain; simply compile with `g++ -o 6502.o 6502.cpp -I /path/where/6502.hpp/resides -c -O2`. This will create the object file `6502.o` that the project can link with.
## CMake
Start by adding 6502 to your project: `add_subdirectory(path/to/6502)`. The project can then link with lib6502 with for example `target_include_directories(TARGET PRIVATE ${6502_DIRECTORIES})` and `target_link_libraries(TARGET ${6502_LIBRARIES})`.