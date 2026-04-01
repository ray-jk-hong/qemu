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

#include "hw/boards.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/riscv_hart.h"

#define LRZX_P_CPUS_MAX         16
#define LRZX_P_SOC_NUM_MAX      1

#define TYPE_RISCV_LRZX_P_MACHINE MACHINE_TYPE_NAME("lrzx_p")
typedef struct RISCVLrzxPState RISCVLrzxPState;
DECLARE_INSTANCE_CHECKER(RISCVLrzxPState, RISCV_LRZX_P_MACHINE,
                         TYPE_RISCV_LRZX_P_MACHINE)

struct LrzxPSocInfo {
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

    /* socket array */
    uint32_t soc_num;
    RISCVHartArrayState soc[LRZX_P_SOC_NUM_MAX];
    struct LrzxPSocInfo soc_info[LRZX_P_SOC_NUM_MAX];

    DeviceState *irqchip[LRZX_P_SOC_NUM_MAX];

    struct RISCVLrzxPBootInfo boot_info;

    Notifier notifier; /* machine done notifier */
};

enum {
    LRZX_P_DEV_MROM = 0,
    LRZX_P_DEV_CLINT,
    LRZX_P_DEV_ACLINT_SSWI,
    LRZX_P_DEV_APLIC_M,
    LRZX_P_DEV_APLIC_S,
    LRZX_P_DEV_UART0,
    LRZX_P_DEV_DRAM
};

enum {
    LYNX_UART0_IRQ = 10,
};

#define LRZX_P_RSTVEC    0x1090000000

#define LRZX_P_IRQCHIP_NUM_SOURCES      96
#define LRZX_P_IRQCHIP_NUM_PRIO_BITS    3

#define LRZX_P_DEBUG

#define UNUSED(__x) (void)__x

#endif