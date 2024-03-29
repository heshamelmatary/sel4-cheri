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
 *
 * Copyright 2016, 2017 Hesham Almatary, Data61/CSIRO <hesham.almatary@data61.csiro.au>
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 */

#include <config.h>
#include <model/statedata.h>
#include <arch/fastpath/fastpath.h>
#include <arch/kernel/traps.h>
#include <machine/debug.h>
#include <api/syscall.h>
#include <util.h>
#include <arch/machine/hardware.h>

#include <benchmark/benchmark_track.h>
#include <benchmark/benchmark_utilisation.h>

/** DONT_TRANSLATE */
void VISIBLE NORETURN restore_user_context(void)
{
    word_t cur_thread_reg = (word_t) NODE_STATE(ksCurThread)->tcbArch.tcbContext.registers;

    c_exit_hook();

    NODE_UNLOCK_IF_HELD;

#ifdef CONFIG_ARCH_CHERI
    UNUSED register word_t cur_cheri_reg asm("t1") = (word_t) NODE_STATE(ksCurThread)->tcbArch.tcbContext.cheri_registers;
#endif

#ifdef CONFIG_ARCH_CHERI
    asm volatile(
        LOAD_CHERI "  c0, (0*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c1, (1*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c2, (2*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c3, (3*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c4, (4*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c5, (5*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c6, (6*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c7, (7*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c8, (8*%[CREGSIZE])(t1)  \n"
        LOAD_CHERI "  c9, (9*%[CREGSIZE])(t1) \n"
        LOAD_CHERI "  c10, (10*%[CREGSIZE])(t1) \n"
        LOAD_CHERI "  c11, (11*%[CREGSIZE])(t1) \n"
        LOAD_CHERI "  c12, (12*%[CREGSIZE])(t1) \n"
        LOAD_CHERI "  c13, (13*%[CREGSIZE])(t1) \n"
        LOAD_CHERI "  c14, (14*%[CREGSIZE])(t1) \n"
        LOAD_CHERI "  c15, (15*%[CREGSIZE])(t1) \n"
        : /* no output */
        :
        [CREGSIZE] "i" (sizeof(cheri_reg_t)),
        [cur_cheri] "r" (cur_cheri_reg)
    );
#endif

    asm volatile(
        "mv t0, %[cur_thread]       \n"
        LOAD_S " ra, (0*%[REGSIZE])(t0)  \n"
        LOAD_S "  sp, (1*%[REGSIZE])(t0)  \n"
        LOAD_S "  gp, (2*%[REGSIZE])(t0)  \n"
        /* skip tp */
        /* skip x5/t0 */
        LOAD_S "  t2, (6*%[REGSIZE])(t0)  \n"
        LOAD_S "  s0, (7*%[REGSIZE])(t0)  \n"
        LOAD_S "  s1, (8*%[REGSIZE])(t0)  \n"
        LOAD_S "  a0, (9*%[REGSIZE])(t0) \n"
        LOAD_S "  a1, (10*%[REGSIZE])(t0) \n"
        LOAD_S "  a2, (11*%[REGSIZE])(t0) \n"
        LOAD_S "  a3, (12*%[REGSIZE])(t0) \n"
        LOAD_S "  a4, (13*%[REGSIZE])(t0) \n"
        LOAD_S "  a5, (14*%[REGSIZE])(t0) \n"
        LOAD_S "  a6, (15*%[REGSIZE])(t0) \n"
        LOAD_S "  a7, (16*%[REGSIZE])(t0) \n"
        LOAD_S "  s2, (17*%[REGSIZE])(t0) \n"
        LOAD_S "  s3, (18*%[REGSIZE])(t0) \n"
        LOAD_S "  s4, (19*%[REGSIZE])(t0) \n"
        LOAD_S "  s5, (20*%[REGSIZE])(t0) \n"
        LOAD_S "  s6, (21*%[REGSIZE])(t0) \n"
        LOAD_S "  s7, (22*%[REGSIZE])(t0) \n"
        LOAD_S "  s8, (23*%[REGSIZE])(t0) \n"
        LOAD_S "  s9, (24*%[REGSIZE])(t0) \n"
        LOAD_S "  s10, (25*%[REGSIZE])(t0)\n"
        LOAD_S "  s11, (26*%[REGSIZE])(t0)\n"
        LOAD_S "  t3, (27*%[REGSIZE])(t0) \n"
        LOAD_S "  t4, (28*%[REGSIZE])(t0) \n"
        LOAD_S "  t5, (29*%[REGSIZE])(t0) \n"
        LOAD_S "  t6, (30*%[REGSIZE])(t0) \n"
        /* Get next restored tp */
        LOAD_S "  t1, (3*%[REGSIZE])(t0)  \n"
        /* get restored tp */
        "add tp, t1, x0  \n"
        /* get sepc */
        LOAD_S "  t1, (34*%[REGSIZE])(t0)\n"
        "csrw sepc, t1  \n"

        /* Write back sscratch with cur_thread_reg to get it back on the next trap entry */
        "csrw sscratch, t0         \n"

        LOAD_S "  t1, (32*%[REGSIZE])(t0) \n"
        "csrw sstatus, t1\n"

        LOAD_S "  t1, (5*%[REGSIZE])(t0) \n"
        LOAD_S "  t0, (4*%[REGSIZE])(t0) \n"
        "sret"
        : /* no output */
        : [REGSIZE] "i" (sizeof(word_t)),
        [cur_thread] "r" (cur_thread_reg)
        : "memory"
    );

    UNREACHABLE();
}

void VISIBLE NORETURN
c_handle_interrupt(void)
{
    NODE_LOCK_IRQ;

    c_entry_hook();

    handleInterruptEntry();

    restore_user_context();
    UNREACHABLE();
}

void VISIBLE NORETURN
c_handle_exception(void)
{
    NODE_LOCK_SYS;

    c_entry_hook();

    handle_exception();

    restore_user_context();
    UNREACHABLE();
}

void NORETURN
slowpath(syscall_t syscall)
{
    /* check for undefined syscall */
    if (unlikely(syscall < SYSCALL_MIN || syscall > SYSCALL_MAX)) {
        handleUnknownSyscall(syscall);
    } else {
        handleSyscall(syscall);
    }

    restore_user_context();
    UNREACHABLE();
}

void VISIBLE NORETURN
c_handle_syscall(word_t cptr, word_t msgInfo, word_t unused1, word_t unused2, word_t unused3, word_t unused4, word_t unused5, syscall_t syscall)
{
    NODE_LOCK_SYS;

    c_entry_hook();

#ifdef CONFIG_FASTPATH
    if (syscall == (syscall_t)SysCall) {
        fastpath_call(cptr, msgInfo);
        UNREACHABLE();
    } else if (syscall == (syscall_t)SysReplyRecv) {
        fastpath_reply_recv(cptr, msgInfo);
        UNREACHABLE();
    }
#endif /* CONFIG_FASTPATH */
    slowpath(syscall);
    UNREACHABLE();
}
