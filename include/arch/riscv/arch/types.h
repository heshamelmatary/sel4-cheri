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
 * Copyright 2018, Hesham Almatary, University of Cambridge <Hesham.Almatary@cl.cam.ac.uk>
 * Copyright 2016, 2017 Hesham Almatary, Data61/CSIRO <hesham.almatary@data61.csiro.au>
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 */

#ifndef __ARCH_TYPES_H
#define __ARCH_TYPES_H

#include <config.h>
#include <assert.h>
#include <stdint.h>
#include <mode/types.h>

typedef unsigned long word_t;
typedef signed long sword_t;
typedef word_t vptr_t;
typedef word_t paddr_t;
typedef word_t pptr_t;
typedef word_t cptr_t;
typedef word_t dev_id_t;
typedef word_t cpu_id_t;
typedef word_t node_id_t;
typedef word_t dom_t;

/* for libsel4 headers that the kernel shares */
typedef word_t seL4_Word;
typedef cptr_t seL4_CPtr;
typedef uint32_t seL4_Uint32;
typedef uint8_t seL4_Uint8;
typedef node_id_t seL4_NodeId;
typedef paddr_t seL4_PAddr;
typedef dom_t seL4_Domain;

#ifdef CONFIG_ARCH_CHERI
struct cheri_reg {
    uint64_t base;
    uint64_t length;
    uint64_t offset;

    uint32_t uperms : 20;
    uint32_t perms  : 11;
    uint32_t sealed : 1;

        uint32_t otype    : 24;
        uint32_t reserved : 7;
        uint32_t tag      : 1;
    };

    typedef struct cheri_reg cheri_reg_t;
#endif /* CONFIG_ARCH_CHERI */

#define wordBits BIT(wordRadix)

#endif
