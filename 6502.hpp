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
namespace CPU {
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
class CPU
{
    public:
    CPU();
    CPU(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char));

    void setRW(unsigned char (*readByte)(unsigned short), void (*writeByte)(unsigned short, unsigned char));

    void cycle();
    inline void cycles(unsigned int c) { while (c--) { cycle(); } }
    inline void step() { do { cycle(); } while (state != FETCH); }

    void run(bool thread = true);
    void run(unsigned long, bool thread = true);

    void raise(enum raise_modes);

    private:
    unsigned short PC, S;
    unsigned char A, X, Y;
    bool P[8];

    unsigned char (*readByte)(unsigned short);
    void (*writeByte)(unsigned short, unsigned char);

    void variablesInit();

    enum instruction_state state;
    enum raise_modes mode;
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

    void o_br();
    void o_fl();
    void o_cp(unsigned char&);
    void o_de();
    void o_in();
    void o_ld(unsigned char&);
    void o_ph();
    void o_st(unsigned char&);
    void o_stk();

    inline void flUpdate(unsigned char v)
    {
        P[NEGATIVE] = (v >> 7) & 1;
        P[ZERO] = v == 0;
    }

    bool vectorFetch;
    void vectorPull(enum raise_modes mode);

    #define RD_DEFAULT_RETURN_DATA 0xEA // 6502 instruction NOP
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
};
} // namespace CPU
#else
#error The 6502 library is C++ based! Unable to compile with C code
#endif
#endif