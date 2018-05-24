/*
 * Copyright 2018, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_GPL)
 */

/*
 * Copyright (c) 2018, Hesham Almatary <Hesham.Almatary@cl.cam.ac.uk>
 * All rights reserved.
 *
 * This software was was developed in part by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
 */

/*
 *
 * Copyright 2016, 2017 Hesham Almatary, Data61/CSIRO <hesham.almatary@data61.csiro.au>
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 */

#include <stdint.h>
#include <util.h>
#include <machine/io.h>
#include <plat/machine/devices.h>
#include <arch/machine.h>
#include <arch/sbi.h>
#include <arch/encoding.h>
#include <arch/kernel/traps.h>

static inline void print_format_cause(int cause_num)
{
    switch (cause_num) {
    case RISCVInstructionMisaligned:
        printf("Instruction address misaligned\n");
        break;
    case RISCVInstructionAccessFault:
        printf("Instruction access fault\n");
        break;
    case RISCVInstructionIllegal:
        printf("Illegal instruction\n");
        break;
    case RISCVBreakpoint:
        printf("Breakpoint\n");
        break;
    case RISCVLoadAddressMisaligned:
        printf("Load address misaligned\n");
        break;
    case RISCVLoadAccessFault:
        printf("Load access fault\n");
        break;
    case RISCVAddressMisaligned:
        printf("AMO address misaligned\n");
        break;
    case RISCVStoreAccessFault:
        printf("Store/AMO access fault\n");
        break;
    case RISCVEnvCall:
        printf("Environment call\n");
        break;
    case RISCVInstructionPageFault:
        printf("Instruction page fault\n");
        break;
    case RISCVLoadPageFault:
        printf("Load page fault\n");
        break;
    case RISCVStorePageFault:
        printf("Store page fault\n");
        break;
    default:
        printf("Reserved cause %d\n", cause_num);
        break;
    }
}

#define SH_RD 7
#define SH_RS1 15
#define SH_RS2 20
#define SH_RS2C 2

#if __riscv_xlen == 64
# define SLL32    sllw
# define STORE    sd
# define LOAD     ld
# define LWU      lwu
# define LOG_REGBYTES 3
#else
# define SLL32    sll
# define STORE    sw
# define LOAD     lw
# define LWU      lw
# define LOG_REGBYTES 2
#endif
#define REGBYTES (1 << LOG_REGBYTES)

#define RV_X(x, s, n) (((x) >> (s)) & ((1 << (n)) - 1))
#define RVC_LW_IMM(x) ((RV_X(x, 6, 1) << 2) | (RV_X(x, 10, 3) << 3) | (RV_X(x, 5, 1) << 6))
#define RVC_LD_IMM(x) ((RV_X(x, 10, 3) << 3) | (RV_X(x, 5, 2) << 6))
#define RVC_LWSP_IMM(x) ((RV_X(x, 4, 3) << 2) | (RV_X(x, 12, 1) << 5) | (RV_X(x, 2, 2) << 6))
#define RVC_LDSP_IMM(x) ((RV_X(x, 5, 2) << 3) | (RV_X(x, 12, 1) << 5) | (RV_X(x, 2, 3) << 6))
#define RVC_SWSP_IMM(x) ((RV_X(x, 9, 4) << 2) | (RV_X(x, 7, 2) << 6))
#define RVC_SDSP_IMM(x) ((RV_X(x, 10, 3) << 3) | (RV_X(x, 7, 3) << 6))
#define RVC_RS1S(insn) (8 + RV_X(insn, SH_RD, 3))
#define RVC_RS2S(insn) (8 + RV_X(insn, SH_RS2C, 3))
#define RVC_RS2(insn) RV_X(insn, SH_RS2C, 5)

#define SHIFT_RIGHT(x, y) ((y) < 0 ? ((x) << -(y)) : ((x) >> (y)))
static inline int GET_REG(word_t insn, int pos);
static inline int GET_REG(word_t insn, int pos)
{
    int mask = (1 << (5 + LOG_REGBYTES)) - (1 << LOG_REGBYTES);
    int reg = ((SHIFT_RIGHT(insn, (pos) - LOG_REGBYTES) & (mask)));
    return reg;
}

#define ROUNDUP(a, b) ((((a)-1)/(b)+1)*(b))
#define ROUNDDOWN(a, b) ((a)/(b)*(b))

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CLAMP(a, lo, hi) MIN(MAX(a, lo), hi)

#define EXTRACT_FIELD(val, which) (((val) & (which)) / ((which) & ~((which)-1)))
#define INSERT_FIELD(val, which, fieldval) (((val) & ~(which)) | ((fieldval) * ((which) & ~((which)-1))))

#define STR(x) XSTR(x)
#define XSTR(x) #x

#define DECLARE_UNPRIVILEGED_LOAD_FUNCTION(type, insn)              \
  static inline type load_##type(const type* addr, word_t mepc)  \
  {                                                                 \
    register word_t __mepc asm ("a2") = mepc;                    \
    register word_t __mstatus asm ("a3");                        \
    type val;                                                       \
    asm ("csrrs %0, mstatus, %3\n"                                  \
         #insn " %1, %2\n"                                          \
         "csrw mstatus, %0"                                         \
         : "+&r" (__mstatus), "=&r" (val)                           \
         : "m" (*addr), "r" (MSTATUS_MPRV), "r" (__mepc));          \
    return val;                                                     \
  }

DECLARE_UNPRIVILEGED_LOAD_FUNCTION(uint8_t, lbu)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(uint16_t, lhu)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(int8_t, lb)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(int16_t, lh)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(int32_t, lw)
#if __riscv_xlen == 64
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(uint32_t, lwu)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(uint64_t, ld)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(word_t, ld)
#else
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(uint32_t, lw)
DECLARE_UNPRIVILEGED_LOAD_FUNCTION(word_t, lw)

static inline uint64_t load_uint64_t(const uint64_t* addr, word_t mepc)
{
    return load_uint32_t((uint32_t*)addr, mepc)
           + ((uint64_t)load_uint32_t((uint32_t*)addr + 1, mepc) << 32);
}
#endif

typedef word_t insn_t;

union byte_array {
    uint8_t bytes[8];
    word_t intx;
    uint64_t int64;
};

static word_t get_insn(word_t mepc, word_t* mstatus)
{
    register word_t __mepc asm ("a2") = mepc;
    register word_t __mstatus asm ("a3");
    word_t val;
    asm ("csrrs %[mstatus], mstatus, %[mprv]\n"
         STR(LWU) " %[insn], (%[addr])\n"
         "csrw mstatus, %[mstatus]"
         : [mstatus] "+&r" (__mstatus), [insn] "=&r" (val)
         : [mprv] "r" (MSTATUS_MPRV | MSTATUS_MXR), [addr] "r" (__mepc));
    *mstatus = __mstatus;
    return val;
}

void misaligned_load_trap(word_t mcause, word_t mepc);

void misaligned_load_trap(word_t mcause, word_t mepc)
{
    union byte_array val;
    word_t mstatus;
    insn_t insn = get_insn(mepc, &mstatus);
    word_t npc = mepc + 4;
    word_t addr = read_csr(mbadaddr);

    int shift = 0, fp = 0, len;
    if ((insn & MASK_LW) == MATCH_LW) {
        len = 4, shift = 8 * (sizeof(word_t) - len);
    }
#if __riscv_xlen == 64
    else if ((insn & MASK_LD) == MATCH_LD) {
        len = 8, shift = 8 * (sizeof(word_t) - len);
    } else if ((insn & MASK_LWU) == MATCH_LWU) {
        len = 4;
    }
#endif
    else if ((insn & MASK_LH) == MATCH_LH) {
        len = 2, shift = 8 * (sizeof(word_t) - len);
    } else if ((insn & MASK_LHU) == MATCH_LHU) {
        len = 2;
    }
#ifdef __riscv_compressed
# if __riscv_xlen >= 64
    else if ((insn & MASK_C_LD) == MATCH_C_LD) {
        len = 8, shift = 8 * (sizeof(word_t) - len), insn = RVC_RS2S(insn) << SH_RD;
    } else if ((insn & MASK_C_LDSP) == MATCH_C_LDSP && ((insn >> SH_RD) & 0x1f)) {
        len = 8, shift = 8 * (sizeof(word_t) - len);
    }
# endif
    else if ((insn & MASK_C_LW) == MATCH_C_LW) {
        len = 4, shift = 8 * (sizeof(word_t) - len), insn = RVC_RS2S(insn) << SH_RD;
    } else if ((insn & MASK_C_LWSP) == MATCH_C_LWSP && ((insn >> SH_RD) & 0x1f)) {
        len = 4, shift = 8 * (sizeof(word_t) - len);
    }
#endif
    else {
        printf("Misalign handler, illegal\n");
        return;
    }

    val.int64 = 0;
    for (word_t i = 0; i < len; i++) {
        val.bytes[i] = load_uint8_t((void *)(addr + i), mepc);
    }

    if (!fp)
        //SET_RD(insn, regs, (intptr_t)val.intx << shift >> shift);
    {
        printf("Des register = %d\n", GET_REG(insn, (word_t)val.intx << shift >> shift));
    }


    setNextPC(NODE_STATE(ksCurThread), npc);
}

void handle_exception(void)
{
    word_t cause = getRegister(NODE_STATE(ksCurThread), CAUSE);
    word_t epc = getRegister(NODE_STATE(ksCurThread), EPC);

    /* handleVMFaultEvent
     * */
    switch (cause) {
    case RISCVInstructionAccessFault:
    case RISCVLoadAccessFault:
    case RISCVStoreAccessFault:
    case RISCVInstructionPageFault:
    case RISCVLoadPageFault:
    case RISCVStorePageFault:
        handleVMFaultEvent(cause);
        break;
    case RISCVInstructionIllegal:
        handleUserLevelFault(0, 0);
        break;
    case RISCVLoadAddressMisaligned:
        misaligned_load_trap(cause, epc);
    default:
        print_format_cause(read_csr_env(cause));
        printf("epc = %p\n", (void*)(word_t)read_csr_env(epc));
        printf("badaddr = %p\n", (void*)(word_t)read_csr_env(badaddr));
        printf("status = %p\n", (void*)(word_t) getRegister(NODE_STATE(ksCurThread), STATUS));
        printf("Halt!\n");

        printf("Register Context Dump \n");

        register word_t thread_context asm("t0");
        for (int i = 0; i < 32; i++) {
            printf("x%d = %p\n", i, (void*) * (((word_t *) thread_context) + i));
        }

        halt();

    }
}

volatile uint8_t* uart16550;

#define UART_REG_QUEUE     0
#define UART_REG_LINESTAT  5
#define UART_REG_STATUS_RX 0x01
#define UART_REG_STATUS_TX 0x20

void uart16550_putchar(uint8_t ch);
void uart16550_putchar(uint8_t ch)
{
    while ((uart16550[UART_REG_LINESTAT] & UART_REG_STATUS_TX) == 0);
    uart16550[UART_REG_QUEUE] = ch;
}

#define UART_REG_TXFIFO         0
#define UART_REG_RXFIFO         1
#define UART_REG_TXCTRL         2
#define UART_REG_RXCTRL         3
#define UART_REG_DIV            4

#define UART_TXEN                0x1
#define UART_RXEN                0x1

void uart_putchar(uint8_t ch);
extern volatile uint32_t *uart;

void uart_putchar(uint8_t ch)
{
    volatile uint32_t *tx = uart + UART_REG_TXFIFO;
    while ((int32_t)(*tx) < 0);
    *tx = ch;
}

#ifdef CONFIG_PRINTING
void
putConsoleChar(unsigned char c)
{
    if (uart) {
        uart_putchar((uint8_t) c);
    }  else if (uart16550) {
        uart16550_putchar((uint8_t) c);
    } else {
        sbi_console_putchar(c);
    }
}
#endif
