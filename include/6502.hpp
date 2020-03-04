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

    // Pinout only works when mode is set to PINOUT and using run(true).
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
    void indx();
    void indy();

    /* Instructions */
    void o_adc();
    void o_and();
    void o_asl();
    void o_bit();
    void o_brk();
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
    void o_cp();
    void o_de();
    void o_in();
    void o_ld();
    void o_ph();
    void o_st();
    void o_stk();

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
};
} // namespace CPU
#else
#error The 6502 library is C++ based! Unable to compile with C code
#endif
#endif