#include <6502.hpp>
#include <iostream>
#include <chrono>

unsigned char rb(unsigned short addr)
{
    return 0xEA;
}
void wb(unsigned short addr, unsigned char value)
{

}
int main()
{
    CPU::CPU processor(rb, wb);
    
    std::chrono::time_point<std::chrono::system_clock,std::chrono::nanoseconds> time_point;
    unsigned long ccount = 0;
    time_point = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() - time_point < std::chrono::seconds(2)) {
        processor.cycle();
        ++ccount;
    }
    std::cout << ccount << std::endl;

}