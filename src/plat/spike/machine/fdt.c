/* Copyright (c) 2010-2017, The Regents of the University of California
 * (Regents).  All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Regents nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
 * OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
 * HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
 * MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
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


/* This file is copied from RISC-V tools. It's modified to work with seL4 */

#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include <string.h>
#include <plat/machine/fdt.h>
#include <config.h>
#include <util.h>

#define FDT_MAGIC   0xd00dfeed
#define FDT_VERSION 17

#define FDT_BEGIN_NODE  1
#define FDT_END_NODE    2
#define FDT_PROP    3
#define FDT_NOP     4
#define FDT_END     9

/* workaround because string literals are not supported by the C parser */
const char fdt_address_cells[] = {'#', 'a', 'd', 'd', 'r', 'e', 's', 's', '-', 'c', 'e', 'l', 'l', 's', 0};
const char fdt_size_cells[] = {'#', 's', 'i', 'z', 'e', '-', 'c', 'e', 'l', 'l', 's', 0};
const char fdt_reg[] = {'r', 'e', 'g', 0};
const char fdt_device_type[] = {'d', 'e', 'v', 'i', 'c', 'e', '_', 't', 'y', 'p', 'e', 0};
const char fdt_memory[] = {'m', 'e', 'm', 'o', 'r', 'y', 0};

static inline uint32_t bswap(uint32_t x)
{
    uint32_t y = (x & 0x00FF00FF) <<  8 | (x & 0xFF00FF00) >>  8;
    uint32_t z = (y & 0x0000FFFF) << 16 | (y & 0xFFFF0000) >> 16;
    return z;
}

struct scan_state {
    int found_memory;
    const uint32_t *reg_value;
    int reg_len;
};

static const uint32_t *fdt_get_address(const struct fdt_scan_node *node, const uint32_t *value, uint64_t *result)
{
    *result = 0;
    for (int cells = node->address_cells; cells > 0; --cells) {
        *result = (*result << 32) + bswap(*value++);
    }
    return value;
}

static const uint32_t *fdt_get_size(const struct fdt_scan_node *node, const uint32_t *value, uint64_t *result)
{
    *result = 0;
    for (int cells = node->size_cells; cells > 0; --cells) {
        *result = (*result << 32) + bswap(*value++);
    }
    return value;
}

static uint32_t *fdt_scan_helper(
    uint32_t *lex,
    const char *strings,
    struct fdt_scan_node *node,
    struct scan_state *state)
{
    struct fdt_scan_node child;
    struct fdt_scan_prop prop;
    int last = 0;

    child.parent = node;
    // these are the default cell counts, as per the FDT spec
    child.address_cells = 2;
    child.size_cells = 1;
    prop.node = node;

    while (1) {
        switch (bswap(lex[0])) {
        case FDT_NOP: {
            lex += 1;
            break;
        }
        case FDT_PROP: {
            assert (!last);
            prop.name  = strings + bswap(lex[2]);
            prop.len   = bswap(lex[1]);
            prop.value = lex + 3;
            if (node && !strncmp(prop.name, fdt_address_cells, 14)) {
                node->address_cells = bswap(lex[3]);
            }
            if (node && !strncmp(prop.name, fdt_size_cells, 11))    {
                node->size_cells    = bswap(lex[3]);
            }
            lex += 3 + (prop.len + 3) / 4;
            if (state->found_memory && strncmp(prop.name, fdt_reg, 3) == 0) {
                state->reg_value = prop.value;
                state->reg_len = prop.len;
            }
            if (strncmp(prop.name, fdt_device_type, 11) == 0 && strncmp((const char*)prop.value, fdt_memory, 6) == 0) {
                state->found_memory = 1;
            }
            break;
        }
        case FDT_BEGIN_NODE: {
            uint32_t *lex_next;
            last = 1;
            child.name = (const char *)(lex + 1);
            lex_next = fdt_scan_helper(
                           lex + 2 + strnlen(child.name, 1024) / 4,
                           strings, &child, state);
            lex = lex_next;
            break;
        }
        case FDT_END_NODE: {
            if (state->found_memory) {
                const uint32_t *value = state->reg_value;
                const uint32_t *end = value + state->reg_len / 4;

                assert (state->reg_value && state->reg_len % 4 == 0);

                while (end - value > 0) {
                    uint64_t base, size;
                    value = fdt_get_address(node->parent, value, &base);
                    value = fdt_get_size   (node->parent, value, &size);
                    if (!add_avail_p_reg((p_region_t) {
                    base, base + size
                })) {
                        printf("Failed to add physical memory region %llu-%llu\n", (unsigned long long)base, (unsigned long long)(base + size));
                    }
                }
                state->found_memory = 0;
            }
            return lex + 1;
        }
        default: { // FDT_END
            return lex;
        }
        }
    }
}

void parseFDT(void *fdt)
{
    struct fdt_header *header = (struct fdt_header *)fdt;

    // Only process FDT that we understand
    if (bswap(header->magic) != FDT_MAGIC ||
            bswap(header->last_comp_version) > FDT_VERSION) {
        return;
    }

    const char *strings = (const char *)((word_t)fdt + bswap(header->off_dt_strings));
    uint32_t *lex = (uint32_t *)((word_t)fdt + bswap(header->off_dt_struct));

    struct scan_state state;
    state.found_memory = 0;

    fdt_scan_helper(lex, strings, 0, &state);
}

uint32_t fdt_size(void *fdt)
{
    struct fdt_header *header = (struct fdt_header *)fdt;

    // Only process FDT that we understand
    if (bswap(header->magic) != FDT_MAGIC ||
            bswap(header->last_comp_version) > FDT_VERSION) {
        return 0;
    }
    return bswap(header->totalsize);
}

#define UART_REG_TXFIFO         0
#define UART_REG_RXFIFO         1
#define UART_REG_TXCTRL         2
#define UART_REG_RXCTRL         3
#define UART_REG_DIV            4

#define UART_TXEN                0x1
#define UART_RXEN                0x1

static uint32_t *fdt_scan_helper_pk(
    uint32_t *lex,
    const char *strings,
    struct fdt_scan_node *node,
    const struct fdt_cb *cb)
{
    struct fdt_scan_node child;
    struct fdt_scan_prop prop;
    int last = 0;

    child.parent = node;
    // these are the default cell counts, as per the FDT spec
    child.address_cells = 2;
    child.size_cells = 1;
    prop.node = node;

    while (1) {
        switch (bswap(lex[0])) {
        case FDT_NOP: {
            lex += 1;
            break;
        }
        case FDT_PROP: {
            assert (!last);
            prop.name  = strings + bswap(lex[2]);
            prop.len   = bswap(lex[1]);
            prop.value = lex + 3;
            if (node && !strncmp(prop.name, "#address-cells", 14)) {
                node->address_cells = bswap(lex[3]);
            }
            if (node && !strncmp(prop.name, "#size-cells", 11))    {
                node->size_cells    = bswap(lex[3]);
            }
            lex += 3 + (prop.len + 3) / 4;
            cb->prop(&prop, cb->extra);
            break;
        }
        case FDT_BEGIN_NODE: {
            uint32_t *lex_next;
            if (!last && node && cb->done) {
                cb->done(node, cb->extra);
            }
            last = 1;
            child.name = (const char *)(lex + 1);
            if (cb->open) {
                cb->open(&child, cb->extra);
            }
            lex_next = fdt_scan_helper_pk(
                           lex + 2 + strnlen(child.name, 100) / 4,
                           strings, &child, cb);
            if (cb->close && cb->close(&child, cb->extra) == -1)
                while (lex != lex_next) {
                    *lex++ = bswap(FDT_NOP);
                }
            lex = lex_next;
            break;
        }
        case FDT_END_NODE: {
            if (!last && node && cb->done) {
                cb->done(node, cb->extra);
            }
            return lex + 1;
        }
        default: { // FDT_END
            if (!last && node && cb->done) {
                cb->done(node, cb->extra);
            }
            return lex;
        }
        }
    }
}

void fdt_scan(word_t fdt, const struct fdt_cb *cb)
{
    struct fdt_header *header = (struct fdt_header *)fdt;

    // Only process FDT that we understand
    if (bswap(header->magic) != FDT_MAGIC ||
            bswap(header->last_comp_version) > FDT_VERSION) {
        return;
    }

    const char *strings = (const char *)(fdt + bswap(header->off_dt_strings));
    uint32_t *lex = (uint32_t *)(fdt + bswap(header->off_dt_struct));

    fdt_scan_helper_pk(lex, strings, 0, cb);
}

volatile uint32_t *uart;
struct uart_scan {
    int compat;
    uint64_t reg;
};

static void uart_open(const struct fdt_scan_node *node, void *extra)
{
    struct uart_scan *scan = (struct uart_scan *)extra;
    memset(scan, 0, sizeof(*scan));
}

static void uart_prop(const struct fdt_scan_prop *prop, void *extra)
{
    struct uart_scan *scan = (struct uart_scan *)extra;
    if (!strncmp(prop->name, "compatible", 10) && !strncmp((const char*)prop->value, "sifive,uart0", 12)) {
        scan->compat = 1;
    } else if (!strncmp(prop->name, "reg", 3)) {
        fdt_get_address(prop->node->parent, prop->value, &scan->reg);
    }
}

static void uart_done(const struct fdt_scan_node *node, void *extra)
{
    struct uart_scan *scan = (struct uart_scan *)extra;
    if (!scan->compat || !scan->reg || uart) {
        return;
    }

    // Enable Rx/Tx channels
    uart = (void*)(word_t)scan->reg;
#ifdef KERNEL_UART_BASE
    uart = (void*) ((word_t) uart | KERNEL_UART_BASE);
#endif
    uart[UART_REG_TXCTRL] = UART_TXEN;
    uart[UART_REG_RXCTRL] = UART_RXEN;
}

void query_uart(void *fdt)
{
    struct fdt_cb cb;
    struct uart_scan scan;

    memset(&cb, 0, sizeof(cb));
    cb.open = uart_open;
    cb.prop = uart_prop;
    cb.done = uart_done;
    cb.extra = &scan;

    fdt_scan((word_t)fdt, &cb);
}

volatile uint8_t* uart16550;

#define UART_REG_QUEUE     0
#define UART_REG_LINESTAT  5
#define UART_REG_STATUS_RX 0x01
#define UART_REG_STATUS_TX 0x20

struct uart16550_scan {
    int compat;
    uint64_t reg;
};

static void uart16550_open(const struct fdt_scan_node *node, void *extra)
{
    struct uart16550_scan *scan = (struct uart16550_scan *)extra;
    memset(scan, 0, sizeof(*scan));
}

static void uart16550_prop(const struct fdt_scan_prop *prop, void *extra)
{
    struct uart16550_scan *scan = (struct uart16550_scan *)extra;
    if (!strncmp(prop->name, "compatible", 10) && !strncmp((const char*)prop->value, "ns16550a", 8)) {
        scan->compat = 1;
    } else if (!strncmp(prop->name, "reg", 3)) {
        fdt_get_address(prop->node->parent, prop->value, &scan->reg);
    }
}

static void uart16550_done(const struct fdt_scan_node *node, void *extra)
{
    struct uart16550_scan *scan = (struct uart16550_scan *)extra;
    if (!scan->compat || !scan->reg || uart16550) {
        return;
    }

    uart16550 = (void*)(word_t)scan->reg;
#ifdef KERNEL_UART_BASE
    uart16550 = (void*) ((word_t) uart16550 | KERNEL_UART_BASE);
#endif
    // http://wiki.osdev.org/Serial_Ports
    uart16550[1] = 0x00;    // Disable all interrupts
    uart16550[3] = 0x80;    // Enable DLAB (set baud rate divisor)
    uart16550[0] = 0x03;    // Set divisor to 3 (lo byte) 38400 baud
    uart16550[1] = 0x00;    //                  (hi byte)
    uart16550[3] = 0x03;    // 8 bits, no parity, one stop bit
    uart16550[2] = 0xC7;    // Enable FIFO, clear them, with 14-byte threshold
}

void query_uart16550(void *fdt)
{
    struct fdt_cb cb;
    struct uart16550_scan scan;

    memset(&cb, 0, sizeof(cb));
    cb.open = uart16550_open;
    cb.prop = uart16550_prop;
    cb.done = uart16550_done;
    cb.extra = &scan;

    fdt_scan((word_t)fdt, &cb);
}

