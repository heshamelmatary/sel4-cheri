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

#include <config.h>
#include <machine/assembler.h>
#include <arch/machine/hardware.h>
#include <arch/api/syscall.h>
#include <arch/machine/registerset.h>
#include <util.h>
#include <arch/encoding.h>

#define REGBYTES (CONFIG_WORD_SIZE / 8)

#ifdef CONFIG_ARCH_CHERI128
#define CREGSIZE (REGBYTES * 2)
#else
#define CREGSIZE (REGBYTES * 4)
#endif

.section .text

.global trap_entry
.extern c_handle_syscall
.extern c_handle_interrupt
.extern c_handle_exception
.extern restore_user_context

trap_entry:

  csrrw t0, sscratch, t0

#ifdef CONFIG_ARCH_CHERI
  cspecialrw c3, c0, mscratchc
  cspecialrw c0, c3, ddc

  csetoffset c3, c3, t0
  cincoffsetimmediate c3, c3, REGBYTES*36

  SC   c1, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c2, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c3, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c4, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c5, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c6, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c7, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c8, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c9, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c10, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c11, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c12, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c13, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c14, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c15, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c16, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c17, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c18, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c19, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c20, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c21, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c22, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c23, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c24, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c25, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c26, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c27, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c28, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c29, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c30, c3
  cincoffsetimmediate c3, c3, CREGSIZE
  SC   c31, c3
#endif

#ifndef CONFIG_CHERI_MERGED_RF
  STORE ra, (0*REGBYTES)(t0)
  STORE sp, (1*REGBYTES)(t0)
  STORE gp, (2*REGBYTES)(t0)
  STORE tp, (3*REGBYTES)(t0)
  STORE t1, (5*REGBYTES)(t0)
  STORE t2, (6*REGBYTES)(t0)
  STORE s0, (7*REGBYTES)(t0)
  STORE s1, (8*REGBYTES)(t0)
  STORE a0, (9*REGBYTES)(t0)
  STORE a1, (10*REGBYTES)(t0)
  STORE a2, (11*REGBYTES)(t0)
  STORE a3, (12*REGBYTES)(t0)
  STORE a4, (13*REGBYTES)(t0)
  STORE a5, (14*REGBYTES)(t0)
  STORE a6, (15*REGBYTES)(t0)
  STORE a7, (16*REGBYTES)(t0)
  STORE s2, (17*REGBYTES)(t0)
  STORE s3, (18*REGBYTES)(t0)
  STORE s4, (19*REGBYTES)(t0)
  STORE s5, (20*REGBYTES)(t0)
  STORE s6, (21*REGBYTES)(t0)
  STORE s7, (22*REGBYTES)(t0)
  STORE s8, (23*REGBYTES)(t0)
  STORE s9, (24*REGBYTES)(t0)
  STORE s10, (25*REGBYTES)(t0)
  STORE s11, (26*REGBYTES)(t0)
  STORE t3, (27*REGBYTES)(t0)
  STORE t4, (28*REGBYTES)(t0)
  STORE t5, (29*REGBYTES)(t0)
  STORE t6, (30*REGBYTES)(t0)
#endif

  /* save t0 value */
  csrr  x1, sscratch
  STORE    x1, (4*REGBYTES)(t0)

  csrr x1, STATUS_REG
  STORE x1, (32*REGBYTES)(t0)

  csrr s0, CAUSE_REG
  STORE s0, (31*REGBYTES)(t0)

  /* Save exception PC */
  csrr x1, EPC_REG
  STORE   x1, (33*REGBYTES)(t0)

  la gp, __global_pointer$

  /* Load kernel's stack address */
  la sp, (kernel_stack_alloc + BIT(CONFIG_KERNEL_STACK_BITS))

  /* Check if it's an interrupt */
  srli s2, s0, (CONFIG_WORD_SIZE - 1)
  li   s1, 0x1
  beq  s2, s1, interrupt

  andi s0, s0, 0xf /* priv 1.10 defines up to 15 exceptions/interrupts */
#if CONFIG_MMULESS
  li   s4, 11      /* M-mode ecalls have value 11 */
#else
  li   s4, 8       /* priv 1.10 has value 8 for ecall exception */
#endif
  bne  s0, s4, exception

syscall:
  /* Set the return address to sepc + 4 in the case of a system/environment call */
  addi x1, x1, 4
  /* Save NEXTPC */
  STORE   x1, (34*REGBYTES)(t0)

  j c_handle_syscall

/* Not an interrupt or a syscall */
exception:
  /* Save NEXTPC */
  STORE   x1, (34*REGBYTES)(t0)
  j c_handle_exception

interrupt:
  /* Save NEXTPC */
  STORE   x1, (34*REGBYTES)(t0)
  j c_handle_interrupt
