/*
6502.hpp
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
#ifndef HPP_6502
#define HPP_6502
#ifdef __cplusplus
#ifndef NO_NAMESPACE
namespace CPU {
#endif
enum raise_modes
{
    NMI,
    RESET,
    IRQ
};
enum status_flags
{
    CARRY,
    ZERO,
    INTERRUPT,
    DECIMAL,
    B0,
    B1,
    OVERFLOW,
    NEGATIVE,
};
enum instruction_state
{
    FETCH,
    LOAD,
    EXECUTE,
    SRESET,
    SNMI,
    SIRQ,
};
enum addressing_mode
{
    IMM,
    ZP,
    ZPX,
    ABS,
    ABSX,
    ABSY,
    IND,
    INDX,
    INDY,
};
#define CPU_DEFAULT_SPEED 1789773 /* ~1.79 MHz */
class CPU
{
    public:
    CPU();
    CPU(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char));

    void setRW(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char));

    void cycle();
    #ifndef NO_INLINE
    inline void cycles(unsigned int c) { while (c--) { cycle(); } }
    inline void step() { do { cycle(); } while (state != FETCH); }
    inline void steps(unsigned int c) { while (c--) { step(); }}
    #else
    #define cycles(c) while (c--) { cycle(); }
    #define step() do { cycle(); } while (state != FETCH);
    #define steps(c) while (c--) { step(); }
    #endif

    void run(unsigned long speed = CPU_DEFAULT_SPEED);
    void run(unsigned long long ms, unsigned long speed = CPU_DEFAULT_SPEED);

    void raise(enum raise_modes);

    #ifdef PROTECT_REGISTERS
    #ifndef NO_INLINE
    private:
    #else
    #error PROTECT_REGISTERS and NO_INLINE can't be enabled at the same time!
    #endif
    #endif
    unsigned short PC;
    unsigned char A, X, Y, S;
    bool P[8];

    enum instruction_state state;

    #ifndef PROTECT_REGISTERS
    private:
    #endif

    unsigned char (*readByte)(unsigned short);
    void (*writeByte)(unsigned short, unsigned char);

    void variablesInit();

    enum addressing_mode addm;
    unsigned char cycle_count;
    unsigned char opcode, wv0, wv1;
    unsigned short av;
    void fetch();
    void load();
    void exec();

    /* Addressing mode functions */
    void zp();
    void zpx();
    void abs();
    void absx();
    void absy();
    void ind();
    void indx();
    void indy();

    /* Instructions */
    void o_adc();
    void o_and();
    void o_asl();
    void o_bit();
    void o_brk();
    void o_dec();
    void o_inc();
    void o_eor();
    void o_jmp();
    void o_jsr();
    void o_lsr();
    void o_nop();
    void o_ora();
    void o_rol();
    void o_ror();
    void o_rti();
    void o_rts();
    void o_sbc();
    void o_php();
    void o_plp();

    void o_br(bool&, bool);
    void o_fl(bool&, bool);
    void o_cp(unsigned char&);
    void o_de(unsigned char&);
    void o_in(unsigned char&);
    void o_ld(unsigned char&);
    void o_st(unsigned char&);
    void o_tr(unsigned char&, unsigned char);
    void o_ph(unsigned char);
    void o_pl(unsigned char&);

    bool irqc;
    void irq();

    #define RD_DEFAULT_RETURN_DATA 0xEA // 6502 instruction NOP
    #ifndef NO_INLINE
    inline unsigned char rd(unsigned short adr)
    {
        if (readByte != nullptr) {
            return readByte(adr);
        }
        return RD_DEFAULT_RETURN_DATA;
    }
    inline void wr(unsigned short adr, unsigned char value)
    {
        if (writeByte != nullptr)
        {
            writeByte(adr, value);
        }
    }
    inline void push(unsigned char val) { wr(0x100 + S--, val); }
    inline unsigned char pop()          { return rd(0x100 + (++S)); }
    inline void flUpdate(unsigned char v)
    {
        P[NEGATIVE] = (v >> 7) & 1;
        P[ZERO] = v == 0;
    }
    #else
    unsigned char rd(unsigned short adr);
    void wr(unsigned short adr, unsigned char value);
    void push(unsigned char val);
    unsigned char pop();
    void flUpdate(unsigned char v);
    #endif
};
#ifndef NO_NAMESPACE
} // namespace CPU
#endif
#else
#error The 6502 library is C++ based! Unable to compile with C code
#endif
#endif