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

#include <assert.h>
#include <kernel/boot.h>
#include <machine/io.h>
#include <model/statedata.h>
#include <object/interrupt.h>
#include <arch/machine.h>
#include <arch/kernel/boot.h>
#include <arch/kernel/vspace.h>
#include <arch/benchmark.h>
#include <linker.h>
#include <plat/machine/hardware.h>
#include <plat/machine/fdt.h>
#include <machine.h>
#include <stdarg.h>

/* pointer to the end of boot code/data in kernel image */
/* need a fake array to get the pointer from the linker script */
extern char ki_boot_end[1];
/* pointer to end of kernel image */
extern char ki_end[1];

BOOT_CODE static bool_t
create_untypeds(cap_t root_cnode_cap, region_t boot_mem_reuse_reg)
{
    seL4_SlotPos   slot_pos_before;
    seL4_SlotPos   slot_pos_after;

    slot_pos_before = ndks_boot.slot_pos_cur;
    bool_t res = create_kernel_untypeds(root_cnode_cap, boot_mem_reuse_reg, slot_pos_before);

    slot_pos_after = ndks_boot.slot_pos_cur;
    ndks_boot.bi_frame->untyped = (seL4_SlotRegion) {
        slot_pos_before, slot_pos_after
    };
    return res;

}

BOOT_CODE cap_t
create_mapped_it_frame_cap(cap_t pd_cap, pptr_t pptr, vptr_t vptr, asid_t asid, bool_t
                           use_large, bool_t executable)
{
    cap_t cap;
    vm_page_size_t frame_size;

    if (use_large) {
        frame_size = RISCV_Mega_Page;
    } else {
        frame_size = RISCV_4K_Page;
    }

    cap = cap_frame_cap_new(
              asid,                            /* capFMappedASID       */
              pptr,                            /* capFBasePtr          */
              frame_size,                      /* capFSize             */
              wordFromVMRights(VMReadWrite),   /* capFVMRights         */
              0,                               /* capFIsDevice         */
              vptr                             /* capFMappedAddress    */
          );

    if (!config_set(CONFIG_MMULESS)) {
        map_it_frame_cap(pd_cap, cap);
    }

    return cap;
}

/**
 * Split mem_reg about reserved_reg. If memory exists in the lower
 * segment, insert it. If memory exists in the upper segment, return it.
 */
BOOT_CODE static region_t
insert_region_excluded(region_t mem_reg, region_t reserved_reg)
{
    region_t residual_reg = mem_reg;
    bool_t result UNUSED;

    if (reserved_reg.start < mem_reg.start) {
        /* Reserved region is below the provided mem_reg. */
        mem_reg.end = 0;
        mem_reg.start = 0;
        /* Fit the residual around the reserved region */
        if (reserved_reg.end > residual_reg.start) {
            residual_reg.start = reserved_reg.end;
        }
    } else if (mem_reg.end > reserved_reg.start) {
        /* Split mem_reg around reserved_reg */
        mem_reg.end = reserved_reg.start;
        residual_reg.start = reserved_reg.end;
    } else {
        /* reserved_reg is completely above mem_reg */
        residual_reg.start = 0;
        residual_reg.end = 0;
    }
    /* Add the lower region if it exists */
    if (mem_reg.start < mem_reg.end) {
        result = insert_region(mem_reg);
        assert(result);
    }
    /* Validate the upper region */
    if (residual_reg.start > residual_reg.end) {
        residual_reg.start = residual_reg.end;
    }

    return residual_reg;
}

BOOT_CODE static void
init_freemem(region_t ui_reg, region_t dtb_reg)
{
    unsigned int i;
    bool_t result UNUSED;
    region_t cur_reg;
    region_t res_reg[] = {
        {
            // We ignore all physical memory before the dtb as the current riscv-pk (proxy kernel)
            // that we use for loading is broken and provides an incorrect memory map where
            // it claims that the memory that is used to provide the m-mode services are
            // free physical memory. As there is no interface to determine what the memory
            // reserved for this is we simply hope it placed the dtb after itself and exclude
            // all memory up until then.
            .start = 0,
            .end = dtb_reg.end
        },
        {
            // This looks a bit awkward as our symbols are a reference in the kernel image window, but
            // we want to do all allocations in terms of the main kernel window, so we do some translation
            .start = (pptr_t)paddr_to_pptr(kpptr_to_paddr((void*)kernelBase)),
            .end   = (pptr_t)paddr_to_pptr(kpptr_to_paddr((void*)ki_end))
        },
        {
            .start = ui_reg.start,
            .end = ui_reg.end
        }
    };

    for (i = 0; i < MAX_NUM_FREEMEM_REG; i++) {
        ndks_boot.freemem[i] = REG_EMPTY;
    }

    /* Force ordering and exclusivity of reserved regions. */
    assert(res_reg[0].start < res_reg[0].end);
    assert(res_reg[1].start < res_reg[1].end);
    assert(res_reg[2].start < res_reg[2].end);

    assert(res_reg[0].end <= res_reg[1].start);
    assert(res_reg[1].end <= res_reg[2].start);

    for (i = 0; i < get_num_avail_p_regs(); i++) {
        cur_reg = paddr_to_pptr_reg(get_avail_p_reg(i));
        /* Adjust region if it exceeds the kernel window
         * Note that we compare physical address in case of overflow.
         */
        if (pptr_to_paddr((void*)cur_reg.end) > PADDR_TOP) {
            cur_reg.end = PPTR_TOP;
        }
        if (pptr_to_paddr((void*)cur_reg.start) > PADDR_TOP) {
            cur_reg.start = PPTR_TOP;
        }

        cur_reg = insert_region_excluded(cur_reg, res_reg[0]);
        cur_reg = insert_region_excluded(cur_reg, res_reg[1]);
        cur_reg = insert_region_excluded(cur_reg, res_reg[2]);

        if (cur_reg.start != cur_reg.end) {
            result = insert_region(cur_reg);
            assert(result);
        }
    }
}

BOOT_CODE static void
init_irqs(cap_t root_cnode_cap)
{
    irq_t i;

    for (i = 0; i <= maxIRQ; i++) {
        setIRQState(IRQInactive, i);
    }
    setIRQState(IRQTimer, KERNEL_TIMER_IRQ);

    /* provide the IRQ control cap */
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapIRQControl), cap_irq_control_cap_new());
}

/* This and only this function initialises the CPU. It does NOT initialise any kernel state. */
extern char trap_entry[];

BOOT_CODE static void
init_cpu(void)
{
    if (!config_set(CONFIG_SEL4_RV_MACHINE)) {
        activate_kernel_vspace();
    } else {
        /* Save the trap address entry of riscv-pk before overwritting it */
        pk_trap_addr = read_csr(mtvec);
    }

    /* Write trap entry address to stvec */
    write_csr_env(tvec, trap_entry);
}

/* This and only this function initialises the platform. It does NOT initialise any kernel state. */
BOOT_CODE static void
init_plat(region_t dtb)
{
    parseFDT((void*)dtb.start);
    query_uart((void *)dtb.start);
    query_uart16550((void *)dtb.start);

    initIRQController();
    initTimer();
}

/* Main kernel initialisation function. */

static BOOT_CODE bool_t
try_init_kernel(
    paddr_t ui_p_reg_start,
    paddr_t ui_p_reg_end,
    paddr_t dtb_p_reg_start,
    paddr_t dtb_p_reg_end,
    uint32_t pv_offset,
    vptr_t  v_entry
)
{
    cap_t root_cnode_cap;
    cap_t it_pd_cap;
    cap_t it_ap_cap;
    cap_t ipcbuf_cap;
    p_region_t boot_mem_reuse_p_reg = ((p_region_t) {
        kpptr_to_paddr((void*)KERNEL_BASE), kpptr_to_paddr(ki_boot_end)
    });
    region_t boot_mem_reuse_reg = paddr_to_pptr_reg(boot_mem_reuse_p_reg);
    region_t ui_reg = paddr_to_pptr_reg((p_region_t) {
        ui_p_reg_start, ui_p_reg_end
    });
    region_t dtb_reg = paddr_to_pptr_reg((p_region_t) {
        dtb_p_reg_start, dtb_p_reg_end
    });
    pptr_t bi_frame_pptr;
    vptr_t bi_frame_vptr;
    pptr_t cap_frame_pptr;
    vptr_t cap_frame_vptr;
    vptr_t ipcbuf_vptr;
    create_frames_of_region_ret_t create_frames_ret;

    /* convert from physical addresses to userland vptrs */
    v_region_t ui_v_reg;
    v_region_t it_v_reg;
    ui_v_reg.start = (uint32_t) (ui_p_reg_start - pv_offset);
    ui_v_reg.end   = (uint32_t) (ui_p_reg_end   - pv_offset);

    ipcbuf_vptr = ui_v_reg.end;
    bi_frame_vptr = ipcbuf_vptr + BIT(PAGE_BITS);
    cap_frame_vptr = bi_frame_vptr + BIT(PAGE_BITS);

    /* The region of the initial thread is the user image + ipcbuf and boot info */
    it_v_reg.start = ui_v_reg.start;
    it_v_reg.end = bi_frame_vptr + BIT(PAGE_BITS);

    if (!config_set(CONFIG_SEL4_RV_MACHINE)) {
        map_kernel_window();
    }

    /* initialise the CPU */
    init_cpu();

    /* initialize the platform */
    init_plat(dtb_reg);

    /* make the free memory available to alloc_region() */
    init_freemem(ui_reg, dtb_reg);

    printf("pv_offset = %x\n", pv_offset);
    //cap_frame_pptr = (pptr_t) alloc_region(sizeof(cap_t) * (ndks_boot.slot_pos_cur + 1));
    cap_frame_pptr = (pptr_t) alloc_region(PAGE_BITS);

    /* create the root cnode */
    root_cnode_cap = create_root_cnode();
    if (cap_get_capType(root_cnode_cap) == cap_null_cap) {
        return false;
    }

    /* create the cap for managing thread domains */
    create_domain_cap(root_cnode_cap);

    /* create the IRQ CNode */
    if (!create_irq_cnode()) {
        return false;
    }

    /* initialise the IRQ states and provide the IRQ control cap */
    init_irqs(root_cnode_cap);

    /* create the bootinfo frame */
    bi_frame_pptr = allocate_bi_frame(0, CONFIG_MAX_NUM_NODES, ipcbuf_vptr);
    if (!bi_frame_pptr) {
        return false;
    }

    /* Construct an initial address space with enough virtual addresses
     * to cover the user image + ipc buffer and bootinfo frames */
    it_pd_cap = create_it_address_space(root_cnode_cap, it_v_reg);
    if (cap_get_capType(it_pd_cap) == cap_null_cap) {
        return false;
    }

    /* Create and map bootinfo frame cap */
    create_bi_frame_cap(
        root_cnode_cap,
        it_pd_cap,
        bi_frame_pptr,
        bi_frame_vptr
    );

    /* create the initial thread's IPC buffer */
    ipcbuf_cap = create_ipcbuf_frame(root_cnode_cap, it_pd_cap, ipcbuf_vptr);
    if (cap_get_capType(ipcbuf_cap) == cap_null_cap) {
        return false;
    }

    /* create all userland image frames */
    create_frames_ret =
        create_frames_of_region(
            root_cnode_cap,
            it_pd_cap,
            ui_reg,
            true,
            pv_offset
        );
    if (!create_frames_ret.success) {
        return false;
    }
    ndks_boot.bi_frame->userImageFrames = create_frames_ret.region;

    /* create the initial thread's ASID pool */
    it_ap_cap = create_it_asid_pool(root_cnode_cap);
    if (cap_get_capType(it_ap_cap) == cap_null_cap) {
        return false;
    }
    write_it_asid_pool(it_ap_cap, it_pd_cap);

    /* create the idle thread */
    if (!create_idle_thread()) {
        return false;
    }

    /* create the initial thread */
    tcb_t *initial = create_initial_thread(
                         root_cnode_cap,
                         it_pd_cap,
                         v_entry,
                         bi_frame_vptr,
                         ipcbuf_vptr,
                         ipcbuf_cap
                     );

    if (initial == NULL) {
        return false;
    }

    init_core_state(initial);

    /* convert the remaining free memory into UT objects and provide the caps */
    if (!create_untypeds(
                root_cnode_cap,
                boot_mem_reuse_reg)) {
        return false;
    }

    /* no shared-frame caps (RISCV has no multikernel support) */
    ndks_boot.bi_frame->sharedFrames = S_REG_EMPTY;

    /* finalise the bootinfo frame */
    bi_finalise();

#ifdef CONFIG_ARCH_CHERI
    // Allocate an area of memory to hold caps for the user
    cap_t *clist = (cap_t *) cap_frame_pptr;

    // Map it to the user (Read Only? Or no Access?)
    cap_t cap = create_mapped_it_frame_cap(it_pd_cap, cap_frame_pptr, cap_frame_vptr, IT_ASID, false, false);

    printf("There are %lu caps that need %lu bytes\n", ndks_boot.slot_pos_cur + 1, ndks_boot.slot_pos_cur * 16);
    //cap = *((cap_t *) SLOT_PTR(cap_cnode_cap_get_capCNodePtr(root_cnode_cap) , i));
    unsigned long long base = (unsigned long long) pv_offset;

#if 1
    pptr_t kernel_clist_pptr = (pptr_t) SLOT_PTR(cap_cnode_cap_get_capCNodePtr(root_cnode_cap), 0);
    asm volatile("cspecialrw c21, c0, pcc\n"
                 /* First save almighty pcc to mtcc so that the kernel has
                 access to full address space */
                 "cspecialrw c0, c21, mtcc\n"

                 /* Create arch->pcc address cap */
                 "cspecialrw c23, c0, ddc\n"
                 "csetoffset c23, c23, %0\n"

                 /* Create arch->ddc address cap */
                 "cspecialrw c24, c0, ddc\n"
                 "csetoffset c24, c24, %3\n"

                 "cspecialrw c22, c0, ddc\n"
                 "cspecialrw c0, c22, mscratchc\n"

                 /* Save root cnode to c31 */
                 "cspecialrw c25, c0, ddc\n"
                 "csetoffset c25, c24, %4\n"

                 "cspecialrw c27, c0, ddc\n"
                 "csetoffset c27, c27, %5\n"
                 "csetbounds c27, c27, %6\n"

                 /* Store caps */
                 "sqcap  c27, c25\n"
                 :
#ifdef CONFIG_CHERI_MERGED_RF
                 : "r" (&(initial->tcbArch.tcbContext.registers[pcc])),
#else
                 : "r" (&(initial->tcbArch.tcbContext.cheri_registers[pcc])),
#endif
                 "r" (base),
                 "r" (ui_v_reg.end - ui_v_reg.start),
#ifdef CONFIG_CHERI_MERGED_RF
                 "r" (&(initial->tcbArch.tcbContext.registers[ddc]))
#else
                 "r" (&(initial->tcbArch.tcbContext.cheri_registers[ddc])),
                 "r" (&(initial->tcbArch.tcbContext.cheri_registers[c27])),
                 "r" (kernel_clist_pptr),
                 "r" (BIT(PAGE_BITS))
#endif
                 : "memory"
                );
#endif

    // Copy all caps to it
    for (int i = 0; i <= ndks_boot.slot_pos_cur; i++) {
        cap = *((cap_t *) SLOT_PTR(cap_cnode_cap_get_capCNodePtr(root_cnode_cap) , i));
        clist[i] = cap;
        asm volatile("ctag %0" :: "r" (&clist[i]));
    }

    // Pin the root cap to the init thread in a CHERI register

    for (int i = 0; i <= ndks_boot.slot_pos_cur; i++) {
        //seL4_Cap cap = *((seL4_Cap *) SLOT_PTR(cap_cnode_cap_get_capCNodePtr(root_cnode_cap) , i));
        //printf("#%d: 0x%lx%lx\n", i, cap.words[1], cap.words[0]);

        /* Copy all root cnoide caps to user's bootinfo */
        //ndks_boot.bi_frame->user_cnode_root[i] = cap;

        /* Tag it */
        //asm volatile("ctag %0" :: "r" (&(ndks_boot.bi_frame->user_cnode_root[i])));
    }
#endif

    ksNumCPUs = 1;

    printf("Booting all finished, dropped to user space\n");
    return true;
}


void
init_bi_frame_mmuless(
    pptr_t   pptr,
    node_id_t  node_id,
    word_t   num_nodes,
    vptr_t   ipcbuf_vptr
);

BOOT_CODE void
init_bi_frame_mmuless(
    pptr_t   pptr,
    node_id_t  node_id,
    word_t   num_nodes,
    vptr_t   ipcbuf_vptr
)
{
    if (!pptr) {
        printf("Kernel init failed: could not allocate bootinfo frame\n");
    }
    clearMemory((void*)pptr, BI_FRAME_SIZE_BITS);

    /* initialise bootinfo-related global state */
    ndks_boot.bi_frame = BI_PTR(pptr);
    ndks_boot.slot_pos_cur = seL4_NumInitialCaps;

    BI_PTR(pptr)->nodeID = node_id;
    BI_PTR(pptr)->numNodes = num_nodes;
    BI_PTR(pptr)->numIOPTLevels = 0;
    BI_PTR(pptr)->ipcBuffer = (seL4_IPCBuffer *) ipcbuf_vptr;
    BI_PTR(pptr)->initThreadCNodeSizeBits = CONFIG_ROOT_CNODE_SIZE_BITS;
    BI_PTR(pptr)->initThreadDomain = ksDomSchedule[ksDomScheduleIdx].domain;
    BI_PTR(pptr)->extraLen = 0;
    BI_PTR(pptr)->extraBIPages.start = 0;
    BI_PTR(pptr)->extraBIPages.end = 0;
}

cap_t
create_ipcbuf_frame_mmuless(cap_t root_cnode_cap, cap_t pd_cap, pptr_t pptr, vptr_t vptr);

BOOT_CODE cap_t
create_ipcbuf_frame_mmuless(cap_t root_cnode_cap, cap_t pd_cap, pptr_t pptr, vptr_t vptr)
{
    cap_t cap;

    if (!pptr) {
        printf("Kernel init failing: could not create ipc buffer frame\n");
        return cap_null_cap_new();
    }
    clearMemory((void*)pptr, PAGE_BITS);

    /* create a cap of it and write it into the root CNode */
    cap = create_mapped_it_frame_cap(pd_cap, pptr, vptr, IT_ASID, false, false);
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadIPCBuffer), cap);

    return cap;
}

static BOOT_CODE bool_t
try_init_kernel_mmuless(
    paddr_t ui_p_reg_start,
    paddr_t ui_p_reg_end,
    paddr_t dtb_p_reg_start,
    paddr_t dtb_p_reg_end,
    uint32_t pv_offset,
    vptr_t  v_entry
)
{
    cap_t root_cnode_cap;
    cap_t it_pd_cap;
    cap_t it_ap_cap;
    cap_t ipcbuf_cap;
    p_region_t boot_mem_reuse_p_reg = ((p_region_t) {
        kpptr_to_paddr((void*)KERNEL_BASE), kpptr_to_paddr(ki_boot_end)
    });
    region_t boot_mem_reuse_reg = paddr_to_pptr_reg(boot_mem_reuse_p_reg);
    region_t dtb_reg = paddr_to_pptr_reg((p_region_t) {
        dtb_p_reg_start, dtb_p_reg_end
    });
    pptr_t bi_frame_pptr;
    vptr_t bi_frame_vptr;
    vptr_t ipcbuf_vptr;
    pptr_t ipcbuf_pptr;
    create_frames_of_region_ret_t create_frames_ret;

    /* Add two pages to physical address for IPC and bootinfo */
    region_t ui_reg = paddr_to_pptr_reg((p_region_t) {
        ui_p_reg_start, ui_p_reg_end + 2 * BIT(PAGE_BITS)
    });

    /* convert from physical addresses to userland vptrs */
    v_region_t ui_v_reg;
    v_region_t it_v_reg;
    ui_v_reg.start = (uint32_t) (ui_p_reg_start - pv_offset);
    ui_v_reg.end   = (uint32_t) (ui_p_reg_end   - pv_offset);

    ipcbuf_vptr = ui_v_reg.end;
    bi_frame_vptr = ipcbuf_vptr + BIT(PAGE_BITS);
    ipcbuf_pptr = ui_p_reg_end;
    bi_frame_pptr = ui_p_reg_end + BIT(PAGE_BITS);

    /* The region of the initial thread is the user image + ipcbuf and boot info */
    it_v_reg.start = ui_v_reg.start;
    it_v_reg.end = bi_frame_vptr + BIT(PAGE_BITS);

#if 0

    paddr_t bi_frame_ptr;
    paddr_t ipcbuf_ptr;
    create_frames_of_region_ret_t create_frames_ret;

    /* convert from physical addresses to userland vptrs */
    //v_region_t ui_v_reg;
    ui_v_reg.start = (uint32_t) (ui_p_reg_start - pv_offset);
    ui_v_reg.end   = (uint32_t) (ui_p_reg_end   - pv_offset);

    ipcbuf_vptr = ui_v_reg.end;
    bi_frame_vptr = ipcbuf_vptr + BIT(PAGE_BITS);

    region_t ui_p_reg;
    region_t it_p_reg;

    ui_p_reg.start = (uint32_t) (ui_p_reg_start);
    ui_p_reg.end   = (uint32_t) (ui_p_reg_end);

    ipcbuf_ptr = ui_p_reg.end;
    bi_frame_ptr = ipcbuf_ptr + BIT(PAGE_BITS);

    /* The region of the initial thread is the user image + ipcbuf and boot info */
    it_p_reg.start = ui_p_reg.start;
    it_p_reg.end = bi_frame_ptr + BIT(PAGE_BITS);
#endif

    /* initialise the CPU */
    init_cpu();

    /* initialize the platform */
    init_plat(dtb_reg);

    /* make the free memory available to alloc_region() */
    init_freemem(ui_reg, dtb_reg);

    /* create the root cnode */
    root_cnode_cap = create_root_cnode();
    if (cap_get_capType(root_cnode_cap) == cap_null_cap) {
        return false;
    }

    /* create the cap for managing thread domains */
    create_domain_cap(root_cnode_cap);

    /* create the IRQ CNode */
    if (!create_irq_cnode()) {
        return false;
    }

    /* initialise the IRQ states and provide the IRQ control cap */
    init_irqs(root_cnode_cap);

    /* create the bootinfo frame */
    init_bi_frame_mmuless(bi_frame_pptr, 0, CONFIG_MAX_NUM_NODES, ipcbuf_vptr);
    if (!bi_frame_pptr) {
        return false;
    }

#if 0
    /* Construct an initial address space with enough virtual addresses
     * to cover the user image + ipc buffer and bootinfo frames */
    it_pd_cap = create_it_address_space(root_cnode_cap, it_v_reg);
    if (cap_get_capType(it_pd_cap) == cap_null_cap) {
        return false;
    }
#endif

    /* Create and map bootinfo frame cap */
    create_bi_frame_cap(
        root_cnode_cap,
        it_pd_cap,
        bi_frame_pptr,
        bi_frame_vptr
    );

    /* create the initial thread's IPC buffer */
    //ipcbuf_cap = create_ipcbuf_frame(root_cnode_cap, it_pd_cap, ipcbuf_vptr);
    ipcbuf_cap = create_ipcbuf_frame_mmuless(root_cnode_cap, it_pd_cap, ipcbuf_pptr,  ipcbuf_vptr);
    if (cap_get_capType(ipcbuf_cap) == cap_null_cap) {
        return false;
    }

    /* create all userland image frames */
    create_frames_ret =
        create_frames_of_region(
            root_cnode_cap,
            it_pd_cap,
            ui_reg,
            true,
            pv_offset
        );
    if (!create_frames_ret.success) {
        return false;
    }
    ndks_boot.bi_frame->userImageFrames = create_frames_ret.region;

#if 0
    /* create the initial thread's ASID pool */
    it_ap_cap = create_it_asid_pool(root_cnode_cap);
    if (cap_get_capType(it_ap_cap) == cap_null_cap) {
        return false;
    }
    write_it_asid_pool(it_ap_cap, it_pd_cap);
#endif

    /* create the idle thread */
    if (!create_idle_thread()) {
        return false;
    }


    //v_entry = v_entry + pv_offset;

    /* create the initial thread */
    tcb_t *initial = create_initial_thread(
                         root_cnode_cap,
                         it_pd_cap,
                         v_entry,
                         bi_frame_vptr,
                         ipcbuf_vptr,
                         ipcbuf_cap
                     );

    printf("bi_frame_vptr, = %p\n", (void *) bi_frame_vptr);

    if (initial == NULL) {
        return false;
    }

    init_core_state(initial);
    unsigned long long base = (unsigned long long) pv_offset;

    // Write PCC and DCC and MTCC

#ifdef CONFIG_ARCH_CHERI
    asm volatile("cspecialrw c21, c0, pcc\n"
                 /* First save almighty pcc to mtcc so that the kernel has
                 access to full address space */
                 "cspecialrw c0, c21, mtcc\n"

                 /* Mint PCC */
                 "csetoffset c21, c21, %1\n"
                 "csetbounds c21, c21, %2\n"

                 /* Create arch->pcc address cap */
                 "cspecialrw c23, c0, ddc\n"
                 "csetoffset c23, c23, %0\n"

                 /* Create arch->ddc address cap */
                 "cspecialrw c24, c0, ddc\n"
                 "csetoffset c24, c24, %3\n"

                 "cspecialrw c22, c0, ddc\n"
                 "cspecialrw c0, c22, mscratchc\n"

                 /* Mint DDC */
                 "csetoffset c22, c22, %1\n"
                 "csetbounds c22, c22, %2\n"

                 /* Store caps */
                 "sqcap  c21, c23\n"
                 "sqcap  c22, c24\n"
                 :
#ifdef CONFIG_CHERI_MERGED_RF
                 : "r" (&(initial->tcbArch.tcbContext.registers[pcc])),
#else
                 : "r" (&(initial->tcbArch.tcbContext.cheri_registers[pcc])),
#endif
                 "r" (base),
                 "r" (ui_v_reg.end - ui_v_reg.start),
#ifdef CONFIG_CHERI_MERGED_RF
                 "r" (&(initial->tcbArch.tcbContext.registers[ddc]))
#else
                 "r" (&(initial->tcbArch.tcbContext.cheri_registers[ddc]))
#endif
                 : "memory"
                );

#endif
    /* convert the remaining free memory into UT objects and provide the caps */
    if (!create_untypeds(
                root_cnode_cap,
                boot_mem_reuse_reg)) {
        return false;
    }

    /* no shared-frame caps (RISCV has no multikernel support) */
    ndks_boot.bi_frame->sharedFrames = S_REG_EMPTY;

    /* finalise the bootinfo frame */
    bi_finalise();

    ksNumCPUs = 1;
    //printf("Booting all finished, dropped to user space\n");
    return true;
}

BOOT_CODE VISIBLE void
init_kernel(
    paddr_t ui_p_reg_start,
    paddr_t ui_p_reg_end,
    sword_t pv_offset,
    vptr_t  v_entry,
    word_t hartid,
    paddr_t dtb_output_p
)
{
    pptr_t dtb_output = (pptr_t)paddr_to_pptr(dtb_output_p);

#ifndef CONFIG_MMULESS
    bool_t result = try_init_kernel(ui_p_reg_start,
                                    ui_p_reg_end,
                                    dtb_output_p,
                                    dtb_output_p + fdt_size((void*)dtb_output),
                                    pv_offset,
                                    v_entry
                                   );

#else
    bool_t result = try_init_kernel_mmuless(ui_p_reg_start,
                                            ui_p_reg_end,
                                            dtb_output_p,
                                            dtb_output_p + fdt_size((void*)dtb_output),
                                            pv_offset,
                                            v_entry
                                           );

#endif
    if (!result) {
        fail ("Kernel init failed for some reason :(");
    }

    schedule();
    activateThread();
}
