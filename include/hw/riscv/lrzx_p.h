/*
 * QEMU RISC-V LRZX-P Board
 *
 * Copyright (c) 2026 Lrzx, Inc.
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
#ifndef HW_LRZX_P_H
#define HW_LRZX_P_H

#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define LRZX_P_CPUS_MAX 8
#define LRZX_P_SOCKETS_MAX 8

#define TYPE_RISCV_LRZX_P_MACHINE MACHINE_TYPE_NAME("lrzx_p")
typedef struct RISCVLrzxPState RISCVLrzxPState;
DECLARE_INSTANCE_CHECKER(RISCVLrzxPState, LRZX_P_MACHINE,
                         TYPE_RISCV_LRZX_P_MACHINE)

struct RISCVLrzxPSocInfo {
int hart_num; /* number of hart */
    int hart_base; /* base hartid */
};

struct RISCVLrzxPBootInfo {
    hwaddr sbi_start;
    hwaddr sbi_end;
    hwaddr kernel_start;
    hwaddr kernel_entry; /* kernel entry address */
    hwaddr kernel_end;
    hwaddr dts_start;
    hwaddr dts_end;
};

struct RISCVLrzxPState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    const MemMapEntry *memmap;
    uint64_t rstvec; /* reset base address */

    uint32_t soc_num;
    RISCVHartArrayState soc[LRZX_P_SOCKETS_MAX];
    struct RISCVLrzxPSocInfo soc_info[LRZX_P_SOCKETS_MAX];

    DeviceState *plic[LRZX_P_SOCKETS_MAX];

    int fdt_size;

    struct RISCVLrzxPBootInfo boot_info;

    Notifier notifier; /* machine done notifier */
};

enum {
    LRZX_P_MROM,
    LRZX_P_CLINT,
    LRZX_P_ACLINT_SSWI,
    LRZX_P_PLIC,
    LRZX_P_UART0,
    LRZX_P_DRAM,
};

enum {
    UART0_IRQ = 10,
};

#define LRZX_P_PLIC_NUM_SOURCES 127
#define LRZX_P_PLIC_NUM_PRIORITIES 7
#define LRZX_P_PLIC_PRIORITY_BASE 0x04
#define LRZX_P_PLIC_PENDING_BASE 0x1000
#define LRZX_P_PLIC_ENABLE_BASE 0x2000
#define LRZX_P_PLIC_ENABLE_STRIDE 0x80
#define LRZX_P_PLIC_CONTEXT_BASE 0x200000
#define LRZX_P_PLIC_CONTEXT_STRIDE 0x1000
#define LRZX_P_PLIC_SIZE(__num_context) \
    (LRZX_P_PLIC_CONTEXT_BASE + (__num_context) * LRZX_P_PLIC_CONTEXT_STRIDE)

#define LRZX_P_RSTVEC    0x1090000000

#endif
