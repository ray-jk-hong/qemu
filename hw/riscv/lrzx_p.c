/*
 * QEMU RISC-V LRZX-P Board
 *
 * Copyright (c) 2026 Lrzx, Inc.
 *
 * Provides a board compatible with the LRZX_P SDK:
 *
 * 0) UART
 *
 * The Mask ROM reset vector jumps to the flash payload at 0x2040_0000.
 * The OTP ROM and Flash boot code will be emulated in a future version.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/units.h"

#include "qemu/qemu-print.h"

#include "qapi/error.h"

#include "target/riscv/cpu.h"

#include "system/device_tree.h"
#include "system/address-spaces.h"
#include "system/system.h"

#include <libfdt.h>

#include "hw/riscv/numa.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/riscv_aplic.h"
#include "hw/char/serial-mm.h"
#include "chardev/char.h"

#include "hw/riscv/lrzx_p.h"

static const MemMapEntry lrzx_p_memmap[] = {
    [LRZX_P_DEV_MROM] =         { LRZX_P_RSTVEC,            0x2000 },

    /* uart */
    [LRZX_P_DEV_UART0] =        { 0x1003002000,             0x1000 },

    /* irqchip */
    [LRZX_P_DEV_CLINT] =        { 0x1100000000,             0x10000 },
    [LRZX_P_DEV_ACLINT_SSWI] =  { 0x10F0000000,             0x4000 },

    [LRZX_P_DEV_APLIC_M] =      { 0x1400000000, APLIC_SIZE(LRZX_P_CPUS_MAX) },
    [LRZX_P_DEV_APLIC_S] =      { 0x1500000000, APLIC_SIZE(LRZX_P_CPUS_MAX) },

    /* DRAM size is cmdline specified, so set size to 0. */
    [LRZX_P_DEV_DRAM] =         { 0x0,                      0x0 }
};

/* Add boot memory */
static inline void lrzx_p_rom_add(const MemMapEntry *memmap)
{
    const MemMapEntry *rom_map = &memmap[LRZX_P_DEV_MROM];
    MemoryRegion *rom_mr = g_new(MemoryRegion, 1);

    memory_region_init_rom(rom_mr, NULL, "riscv.lrzx.p.mrom",
        rom_map->size, &error_fatal);
    /* Add rom to memregion */
    memory_region_add_subregion(get_system_memory(), rom_map->base, rom_mr);
#ifdef LRZX_P_DEBUG
    qemu_printf("Add rom to memregion: base=0x%llx, size=0x%llx\n",
        (unsigned long long)rom_map->base, (unsigned long long)rom_map->size);
#endif
}

/* Fill rom with reset vector */
static void lrzx_p_rom_fill(RISCVLrzxPState *s)
{
    hwaddr kernel_entry = s->boot_info.kernel_entry;
    hwaddr fdt_load_addr = s->boot_info.dts_start;
    MachineState *ms = MACHINE(s);

    /* load the reset vector */
    riscv_setup_rom_reset_vec(ms, &s->soc[0], s->boot_info.sbi_start,
        s->memmap[LRZX_P_DEV_MROM].base,
        s->memmap[LRZX_P_DEV_MROM].size, kernel_entry,
        fdt_load_addr);
#ifdef LRZX_P_DEBUG
    qemu_printf("Fill rom: base=0x%llx, size=0x%llx, kernel_entry=0x%llx, fdt_load_addr=0x%llx\n",
        (unsigned long long)s->memmap[LRZX_P_DEV_MROM].base,
        (unsigned long long)s->memmap[LRZX_P_DEV_MROM].size,
        (unsigned long long)kernel_entry,
        (unsigned long long)fdt_load_addr);
#endif
}

/* Add main memory */
static inline void lrzx_p_ram_add(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);
    memory_region_add_subregion(get_system_memory(),
        s->memmap[LRZX_P_DEV_DRAM].base, ms->ram);

#ifdef LRZX_P_DEBUG
    qemu_printf("Add ram to memregion: base=0x%llx, size=0x%llx\n",
        (unsigned long long)s->memmap[LRZX_P_DEV_DRAM].base,
        (unsigned long long)s->memmap[LRZX_P_DEV_DRAM].size);
#endif
}

static void lrzx_p_sbi_load(RISCVLrzxPState *s)
{
    hwaddr sbi_start = s->memmap[LRZX_P_DEV_DRAM].base;
    MachineState *ms = MACHINE(s);
    hwaddr sbi_end;

    g_assert(ms->firmware != NULL);
    sbi_end = riscv_load_firmware(ms->firmware, &sbi_start, NULL);
    s->boot_info.sbi_start = sbi_start;
    s->boot_info.sbi_end = sbi_end;

#ifdef LRZX_P_DEBUG
    qemu_printf("Load sbi: base=0x%llx, size=0x%llx\n",
        (unsigned long long)sbi_start, (unsigned long long)sbi_end);
#endif
}

/*
 * Load dts file to qemu memory and save to ms->fdt
 * Must be called before riscv_load_kernel, because riscv_load_kernel will add
 * /chosen properties and guest RAM must receive the blob after that.
 */
static void lrzx_p_fdt_pre_load(MachineState *ms)
{
    int fdt_size;

    g_assert(ms->dtb != NULL);
    ms->fdt = load_device_tree(ms->dtb, &fdt_size);
    if (!ms->fdt) {
        error_report("load_device_tree() failed");
        exit(1);
    }
}

static void lrzx_p_kernel_load(RISCVLrzxPState *s, RISCVBootInfo *boot_info)
{
    MachineState *ms = MACHINE(s);
    hwaddr kernel_start;

    g_assert(ms->kernel_filename != NULL);
    kernel_start = QEMU_ALIGN_UP(s->boot_info.sbi_end, 2 * MiB);
    riscv_load_kernel(ms, boot_info, kernel_start, true, NULL);

    s->boot_info.kernel_start = kernel_start;
    s->boot_info.kernel_end = boot_info->image_high_addr;
    s->boot_info.kernel_entry = boot_info->image_low_addr;
#ifdef LRZX_P_DEBUG
    qemu_printf("Load kernel: start=0x%llx, end=0x%llx, entry=0x%llx\n",
        (unsigned long long)s->boot_info.kernel_start,
        (unsigned long long)s->boot_info.kernel_end,
        (unsigned long long)s->boot_info.kernel_entry);
#endif
}

/* Load fdt to real memory */
static void lrzx_p_fdt_load(RISCVLrzxPState *s, RISCVBootInfo *boot_info)
{
    hwaddr dram_base = s->memmap[LRZX_P_DEV_DRAM].base;
    hwaddr dram_size = s->memmap[LRZX_P_DEV_DRAM].size;
    MachineState *ms = MACHINE(s);
    hwaddr fdt_load_addr;

    fdt_load_addr = riscv_compute_fdt_addr(dram_base, dram_size, ms,
                                             boot_info);
    riscv_load_fdt(fdt_load_addr, ms->fdt);
    s->boot_info.dts_start = fdt_load_addr;
    s->boot_info.dts_end = fdt_load_addr + fdt_totalsize(ms->fdt);

#ifdef LRZX_P_DEBUG
    qemu_printf("Load fdt: start=0x%llx, end=0x%llx\n",
        (unsigned long long)s->boot_info.dts_start,
        (unsigned long long)s->boot_info.dts_end);
#endif
}

static void lrzx_p_firmware_load(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);
    RISCVBootInfo boot_info;

    riscv_boot_info_init(&boot_info, &s->soc[0]);

    lrzx_p_sbi_load(s);
    /* Preload fdt to qemu memory, must be called before kernel load */
    lrzx_p_fdt_pre_load(ms);

    lrzx_p_kernel_load(s, &boot_info);
    lrzx_p_fdt_load(s, &boot_info);
}

static void lrzx_p_machine_done(Notifier *notifier, void *data)
{
    RISCVLrzxPState *s = container_of(notifier, RISCVLrzxPState, notifier);
    lrzx_p_firmware_load(s);
    lrzx_p_rom_fill(s);
}

/* Init socket and related properties */
static void lrzx_p_soc_init(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);
    uint32_t soc_num, soc_id;

    soc_num = (uint32_t)riscv_socket_count(ms);
    if (soc_num > LRZX_P_SOC_NUM_MAX) {
        error_report("Get socket num fail. (soc_num=%u)\n", soc_num);
        exit(1);
    }

    for (soc_id = 0; soc_id < soc_num; soc_id++) {
        g_autofree char *soc_name = g_strdup_printf("soc%d", soc_id);

        object_initialize_child(OBJECT(ms), soc_name, &s->soc[soc_id],
            TYPE_RISCV_HART_ARRAY);
        s->soc_info[soc_id].hart_num = riscv_socket_hart_count(ms, soc_id);
        s->soc_info[soc_id].hart_base = riscv_socket_first_hartid(ms, soc_id);
        object_property_set_str(OBJECT(&s->soc[soc_id]), "cpu-type",
            ms->cpu_type, &error_abort);
        object_property_set_int(OBJECT(&s->soc[soc_id]), "num-harts",
            s->soc_info[soc_id].hart_num, &error_abort);
        object_property_set_int(OBJECT(&s->soc[soc_id]), "resetvec",
            s->rstvec, &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->soc[soc_id]), &error_fatal);
    }
    s->soc_num = soc_num;
}

static void lrzx_p_state_init(RISCVLrzxPState *s)
{
    s->memmap = lrzx_p_memmap;
    s->rstvec = LRZX_P_RSTVEC;
    lrzx_p_soc_init(s);
}

static void lrzx_p_uart_init(RISCVLrzxPState *s)
{
    serial_mm_init(get_system_memory(),
                   s->memmap[LRZX_P_DEV_UART0].base,
                   2, /* reg-shift= 2  */
                   qdev_get_gpio_in(s->irqchip[0], LYNX_UART0_IRQ),
                   10000000,
                   serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);
}

static void lrzx_p_aclint_init(RISCVLrzxPState *s, int soc_id)
{
    int base_hartid = s->soc_info[soc_id].hart_base;
    int hart_count = s->soc_info[soc_id].hart_num;

    /* Per-socket ACLINT MSWI, MTIMER, and SSWI */
    riscv_aclint_swi_create(s->memmap[LRZX_P_DEV_CLINT].base +
        soc_id * s->memmap[LRZX_P_DEV_CLINT].size,
        base_hartid, hart_count, false);

    /* For access MTIMER/STIMER CRS */
    riscv_aclint_mtimer_create(s->memmap[LRZX_P_DEV_CLINT].base +
        soc_id * s->memmap[LRZX_P_DEV_CLINT].size +
        RISCV_ACLINT_SWI_SIZE,
        RISCV_ACLINT_DEFAULT_MTIMER_SIZE,
        base_hartid, hart_count,
        RISCV_ACLINT_DEFAULT_MTIMECMP,
        RISCV_ACLINT_DEFAULT_MTIME,
        RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, true);

    riscv_aclint_swi_create(s->memmap[LRZX_P_DEV_ACLINT_SSWI].base +
        soc_id * s->memmap[LRZX_P_DEV_ACLINT_SSWI].size,
        base_hartid, hart_count, true);

#ifdef LRZX_P_DEBUG
    qemu_printf("Init aclint: base=0x%llx, size=0x%llx, hart_count=%u\n",
        (unsigned long long)s->memmap[LRZX_P_DEV_CLINT].base +
        soc_id * s->memmap[LRZX_P_DEV_CLINT].size,
        (unsigned long long)s->memmap[LRZX_P_DEV_CLINT].size,
        hart_count);
    qemu_printf("Init mtimer-aclint: base=0x%llx, size=0x%llx, hart_count=%u\n",
        (unsigned long long)s->memmap[LRZX_P_DEV_CLINT].base +
        soc_id * s->memmap[LRZX_P_DEV_CLINT].size + RISCV_ACLINT_SWI_SIZE,
        (unsigned long long)RISCV_ACLINT_DEFAULT_MTIMER_SIZE, hart_count);
    qemu_printf("Init sswi-aclint: base=0x%llx, size=0x%llx, hart_count=%u\n",
        (unsigned long long)s->memmap[LRZX_P_DEV_ACLINT_SSWI].base +
        soc_id * s->memmap[LRZX_P_DEV_ACLINT_SSWI].size,
        (unsigned long long)s->memmap[LRZX_P_DEV_ACLINT_SSWI].size,
        hart_count);
#endif
}

static DeviceState *lrzx_p_create_aia(RISCVLrzxPState *s, int soc_id)
{
    DeviceState *aplic_s = NULL, *aplic_m = NULL;
    const MemMapEntry *memmap = s->memmap;

    aplic_m = riscv_aplic_create(memmap[LRZX_P_DEV_APLIC_M].base +
        soc_id * memmap[LRZX_P_DEV_APLIC_M].size,
        memmap[LRZX_P_DEV_APLIC_M].size,
        s->soc_info[soc_id].hart_base,
        s->soc_info[soc_id].hart_num,
        LRZX_P_IRQCHIP_NUM_SOURCES,
        LRZX_P_IRQCHIP_NUM_PRIO_BITS,
        false, true, NULL);

    /* Per-socket S-level APLIC */
    aplic_s = riscv_aplic_create(memmap[LRZX_P_DEV_APLIC_S].base +
        soc_id * memmap[LRZX_P_DEV_APLIC_S].size,
        memmap[LRZX_P_DEV_APLIC_S].size,
        s->soc_info[soc_id].hart_base,
        s->soc_info[soc_id].hart_num,
        LRZX_P_IRQCHIP_NUM_SOURCES,
        LRZX_P_IRQCHIP_NUM_PRIO_BITS,
        false, false, aplic_m);

    UNUSED(aplic_s);

    /*
     * Only the root (M-mode) APLIC gets qdev GPIO inputs; child S-APLIC does
     * not. Wired devices must use aplic_m here.
     */
    return aplic_m;
}

static void lrzx_p_irqchip_init(RISCVLrzxPState *s)
{
    uint32_t soc_num = s->soc_num;
    uint32_t soc_id;

    for (soc_id = 0; soc_id < soc_num; soc_id++) {
        lrzx_p_aclint_init(s, soc_id);
        s->irqchip[soc_id] = lrzx_p_create_aia(s, soc_id);
    }
}

static void lrzx_p_machine_init(MachineState *ms)
{
    RISCVLrzxPState *s = RISCV_LRZX_P_MACHINE(ms);

    lrzx_p_state_init(s);

    /* Add rom to memregion */
    lrzx_p_rom_add(s->memmap);
    /* Add main memory to memregion */
    lrzx_p_ram_add(s);

    /* Initialize IRQ chip */
    lrzx_p_irqchip_init(s);

    /* Initialize UART */
    lrzx_p_uart_init(s);

    s->notifier.notify = lrzx_p_machine_done;
    qemu_add_machine_init_done_notifier(&s->notifier);
}

static void lrzx_p_instance_init(Object *obj)
{
}

static void lrzx_p_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V LRZX-P board";
    mc->init = lrzx_p_machine_init;
    mc->max_cpus = LRZX_P_CPUS_MAX;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->default_ram_id = "riscv_lrzx_p_board.ram";
}

static const TypeInfo lrzx_p_typeinfo = {
    .name       = MACHINE_TYPE_NAME("lrzx_p"),
    .parent     = TYPE_MACHINE,
    .class_init = lrzx_p_class_init,
    .instance_init = lrzx_p_instance_init,
    .instance_size = sizeof(RISCVLrzxPState),
    .interfaces = (const InterfaceInfo[]) {
         { TYPE_HOTPLUG_HANDLER },
         { }
    },
};

static void lrzx_p_type_init(void)
{
    type_register_static(&lrzx_p_typeinfo);
}
type_init(lrzx_p_type_init)
