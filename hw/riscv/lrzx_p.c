/*
 * QEMU RISC-V VirtIO Board
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * RISC-V machine with 16550a UART and VirtIO MMIO
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
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/qemu-print.h"

#include "qapi/error.h"

#include <libfdt.h>

#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/char/serial.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/numa.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/misc/sifive_test.h"

#include "target/riscv/cpu.h"

#include "chardev/char.h"
#include "sysemu/device_tree.h"
#include "sysemu/sysemu.h"

#include "hw/riscv/lrzx_p.h"

static const MemMapEntry lrzx_p_memmap[] = {
    [LRZX_P_MROM] =        {     0x1000,        0xf000 },

    /* ACLINT MSWI base addr and size */
    [LRZX_P_CLINT] =       {  0x2000000,       0x10000 },

    /* ACLINT SSWI*/
    [LRZX_P_ACLINT_SSWI] = {  0x2F00000,        0x4000 },

    [LRZX_P_PLIC] =        {  0xc000000, LRZX_P_PLIC_SIZE(LRZX_P_CPUS_MAX * 2) },
    [LRZX_P_UART0] =       { 0x10000000,         0x100 },
    [LRZX_P_DRAM] =        { 0x80000000,           0x0 },
};

/*
 * Load dts file to qemu memory and save to ms->fdt
 * Must be called before riscv_load_kernel, because riscv_load_kernel will add
 * /chosen properties and guest RAM must receive the blob after that.
 */
static void lrzx_load_pre_fdt(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);

    g_assert(ms->dtb != NULL);
    ms->fdt = load_device_tree(ms->dtb, &s->fdt_size);
    if (!ms->fdt) {
        error_report("load_device_tree() failed");
        exit(1);
    }
    if (ms->kernel_cmdline) {
        qemu_fdt_setprop_string(ms->fdt, "/chosen", "bootargs", ms->kernel_cmdline);
    }
}

static void lrzx_p_sbi_load(RISCVLrzxPState *s)
{
    hwaddr sbi_start = s->memmap[LRZX_P_DRAM].base;
    MachineState *ms = MACHINE(s);
    hwaddr sbi_end;

    g_assert(ms->firmware != NULL);

    sbi_end = riscv_load_firmware(ms->firmware, sbi_start, NULL);
    s->boot_info.sbi_start = sbi_start;
    s->boot_info.sbi_end = sbi_end;
}

static void lrzx_p_kernel_load(RISCVLrzxPState *s)
{
    target_ulong kernel_start_addr, kernel_entry;
    MachineState *ms = MACHINE(s);

    g_assert(ms->kernel_filename != NULL);
    kernel_start_addr = riscv_calc_kernel_start_addr(&s->soc[0],
                                                        s->boot_info.sbi_end);

    kernel_entry = riscv_load_kernel(ms->kernel_filename,
                                        kernel_start_addr, NULL);

    s->boot_info.kernel_start = kernel_start_addr;
    s->boot_info.kernel_entry = kernel_entry;
}

static void lrzx_p_initrd_load(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);
    hwaddr start, end;

    g_assert(ms->initrd_filename != NULL);
    end = riscv_load_initrd(ms->initrd_filename,
                                    ms->ram_size, s->boot_info.kernel_entry,
                                    &start);
    qemu_fdt_setprop_cell(ms->fdt, "/chosen",
                            "linux,initrd-start", start);
    qemu_fdt_setprop_cell(ms->fdt, "/chosen", "linux,initrd-end",
                            end);
}

static void lrzx_p_fdt_load(RISCVLrzxPState *s)
{
    target_ulong start_addr = s->memmap[LRZX_P_DRAM].base;
    MachineState *ms = MACHINE(s);
    uint32_t fdt_load_addr;

    /* Compute the fdt load address in dram */
    fdt_load_addr = riscv_load_fdt(start_addr,
        ms->ram_size, ms->fdt);
    s->boot_info.dts_start = fdt_load_addr;
    s->boot_info.dts_end = fdt_load_addr + fdt_totalsize(ms->fdt);
}

static void lrzx_p_rom_fill(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);

    /* load the reset vector */
    riscv_setup_rom_reset_vec(ms, &s->soc[0],
        s->memmap[LRZX_P_DRAM].base,
        s->memmap[LRZX_P_MROM].base,
        s->memmap[LRZX_P_MROM].size,
        s->boot_info.kernel_entry,
        s->boot_info.dts_start,
        ms->fdt);
}

static void lrzx_p_firmware_load(RISCVLrzxPState *s)
{
    /* Preload fdt to qemu memory, must be called before kernel load */
    lrzx_load_pre_fdt(s);
    lrzx_p_sbi_load(s);
    lrzx_p_kernel_load(s);
    lrzx_p_initrd_load(s);
    lrzx_p_fdt_load(s);
}

static void lrzx_p_machine_done(Notifier *notifier, void *data)
{
    RISCVLrzxPState *s = container_of(notifier, RISCVLrzxPState, notifier);
    lrzx_p_firmware_load(s);
    lrzx_p_rom_fill(s);
}

static void lrzx_p_soc_init(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);
    uint32_t soc_num, soc_id;

    soc_num = (uint32_t)riscv_socket_count(ms);
    if (soc_num > LRZX_P_SOCKETS_MAX) {
        error_report("Invalid socket. (soc_num=%u)\n", soc_num);
        exit(1);
    }

    for (soc_id = 0; soc_id < soc_num; soc_id++) {
        g_autofree char *soc_name = g_strdup_printf("soc%d", soc_id);
        int base_hartid, hart_count;

        if (!riscv_socket_check_hartids(ms, soc_id)) {
            error_report("discontinuous hartids in socket%d", soc_id);
            exit(1);
        }

        base_hartid = riscv_socket_first_hartid(ms, soc_id);
        if (base_hartid < 0) {
            error_report("can't find hartid base for socket%d", soc_id);
            exit(1);
        }

        hart_count = riscv_socket_hart_count(ms, soc_id);
        if (hart_count < 0) {
            error_report("can't find hart count for socket%d", soc_id);
            exit(1);
        }

        object_initialize_child(OBJECT(ms), soc_name, &s->soc[soc_id],
                                TYPE_RISCV_HART_ARRAY);
        object_property_set_str(OBJECT(&s->soc[soc_id]), "cpu-type",
                                ms->cpu_type, &error_abort);
        object_property_set_int(OBJECT(&s->soc[soc_id]), "hartid-base",
                                base_hartid, &error_abort);
        object_property_set_int(OBJECT(&s->soc[soc_id]), "num-harts",
                                hart_count, &error_abort);
        object_property_set_int(OBJECT(&s->soc[soc_id]), "resetvec",
                            LRZX_P_RSTVEC, &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->soc[soc_id]), &error_abort);
        s->soc_info[soc_id].hart_num = hart_count;
        s->soc_info[soc_id].hart_base = base_hartid;
    }

    s->soc_num = soc_num;
    s->memmap = lrzx_p_memmap;
    s->rstvec = LRZX_P_RSTVEC;
}

static void lrzx_p_aclint_init(RISCVLrzxPState *s, int soc_id)
{
    const MemMapEntry *memmap = s->memmap;

    /* Per-socket ACLINT MSWI */
    riscv_aclint_swi_create(
        memmap[LRZX_P_CLINT].base + soc_id * memmap[LRZX_P_CLINT].size,
        s->soc_info[soc_id].hart_base,
        s->soc_info[soc_id].hart_num,
        false);

    /* Per-socket ACLINT MTIMER */
    riscv_aclint_mtimer_create(
        memmap[LRZX_P_CLINT].base + soc_id * memmap[LRZX_P_CLINT].size +
            RISCV_ACLINT_SWI_SIZE,
        RISCV_ACLINT_DEFAULT_MTIMER_SIZE,
        s->soc_info[soc_id].hart_base,
        s->soc_info[soc_id].hart_num,
        RISCV_ACLINT_DEFAULT_MTIMECMP,
        RISCV_ACLINT_DEFAULT_MTIME,
        RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ,
        true);

    /* Per-socket ACLINT SSWI */
    riscv_aclint_swi_create(
        memmap[LRZX_P_ACLINT_SSWI].base +
            soc_id * memmap[LRZX_P_ACLINT_SSWI].size,
        s->soc_info[soc_id].hart_base,
        s->soc_info[soc_id].hart_num,
        true);
}

static DeviceState *lrzx_p_plic_init(RISCVLrzxPState *s, int soc_id)
{
    const MemMapEntry *memmap = s->memmap;
    char *plic_hart_config;
    DeviceState *plic;

    /* Per-socket PLIC */
    plic_hart_config = riscv_plic_hart_config_string(s->soc_info[soc_id].hart_num);
    plic = sifive_plic_create(
        memmap[LRZX_P_PLIC].base + soc_id * memmap[LRZX_P_PLIC].size,
        plic_hart_config, s->soc_info[soc_id].hart_num, s->soc_info[soc_id].hart_base,
        LRZX_P_PLIC_NUM_SOURCES,
        LRZX_P_PLIC_NUM_PRIORITIES,
        LRZX_P_PLIC_PRIORITY_BASE,
        LRZX_P_PLIC_PENDING_BASE,
        LRZX_P_PLIC_ENABLE_BASE,
        LRZX_P_PLIC_ENABLE_STRIDE,
        LRZX_P_PLIC_CONTEXT_BASE,
        LRZX_P_PLIC_CONTEXT_STRIDE,
        memmap[LRZX_P_PLIC].size);
    g_free(plic_hart_config);

    return plic;
}

static void lrzx_p_irqchip_init(RISCVLrzxPState *s)
{
    int soc_id;

    for (soc_id = 0; soc_id < s->soc_num; soc_id++) {
        lrzx_p_aclint_init(s, soc_id);
        s->plic[soc_id] = lrzx_p_plic_init(s, soc_id);
    }
}

static void lrzx_p_uart_init(RISCVLrzxPState *s)
{
    DeviceState *plic = s->plic[0];

    serial_mm_init(get_system_memory(),
        s->memmap[LRZX_P_UART0].base,
        0,
        qdev_get_gpio_in(plic, UART0_IRQ),
        399193,
        serial_hd(0),
        DEVICE_LITTLE_ENDIAN
    );
}

static void lrzx_p_rom_add(RISCVLrzxPState *s)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);
    /* boot rom */
    memory_region_init_rom(mr, NULL, "riscv_lrzx_p_board.mrom",
        s->memmap[LRZX_P_MROM].size, &error_fatal);
    memory_region_add_subregion(get_system_memory(),
        s->memmap[LRZX_P_MROM].base, mr);
}

static void lrzx_p_ram_add(RISCVLrzxPState *s)
{
    MachineState *ms = MACHINE(s);

    /* register system main memory (actual RAM) */
    memory_region_add_subregion(get_system_memory(),
        s->memmap[LRZX_P_DRAM].base,
        ms->ram);
}

static void lrzx_p_machine_init(MachineState *machine)
{
    RISCVLrzxPState *s = LRZX_P_MACHINE(machine);

    lrzx_p_soc_init(s);

    lrzx_p_rom_add(s);
    lrzx_p_ram_add(s);

    lrzx_p_irqchip_init(s);
    lrzx_p_uart_init(s);

    s->notifier.notify = lrzx_p_machine_done;
    qemu_add_machine_init_done_notifier(&s->notifier);
}

static void lrzx_p_inst_init(Object *obj)
{
}

static void lrzx_p_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V LRZX-P100 board";
    mc->init = lrzx_p_machine_init;
    mc->max_cpus = LRZX_P_CPUS_MAX;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->default_ram_id = "riscv_lrzx_p_board.ram";
}

static const TypeInfo lrzx_p_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("lrzx_p"),
    .parent     = TYPE_MACHINE,
    .class_init = lrzx_p_class_init,
    .instance_init = lrzx_p_inst_init,
    .instance_size = sizeof(RISCVLrzxPState),
};

static void lrzx_p_machine_init_register_types(void)
{
    type_register_static(&lrzx_p_machine_typeinfo);
}

type_init(lrzx_p_machine_init_register_types)
