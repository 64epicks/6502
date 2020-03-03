#ifndef HPP_6502
#define HPP_6502
#ifdef __cplusplus
namespace CPU {
enum raise_modes
{
    NMI,
    RESET,
    IRQ
};
class CPU
{
    public:
    CPU();
    CPU(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char));

    void setRW(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char));

    void cycle();
    void cycles(unsigned int);
    void step();

    void run(bool thread = true);
    void run(unsigned long, bool thread = true);

    void raise(enum raise_modes);

    struct {
        // All values in pinout are type char but is in fact, a boolean value: all values of 0 or below is treated as FALSE and everything above 0 as TRUE
        char a[16];
        char d[8];
        char be;
        char rwb;
        char mlb;
        char irqb;
        char nmib;
        char resb;
        char rdy;
        char sow;
        char sync;
        char vpb;
    }pinout;

    private:
    unsigned short A, X, Y, P, PC, SP;

    unsigned char (*readByte)(unsigned short);
    void (*writeByte)(unsigned short, unsigned char);

    void variablesInit();
};
} // namespace CPU
#endif
#endif