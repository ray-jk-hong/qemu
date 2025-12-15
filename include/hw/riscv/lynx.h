/*
 * QEMU RISC-V VirtIO machine interface
 *
 * Copyright (c) 2017 SiFive, Inc.
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

#ifndef HW_LYNX_H
#define HW_LYNX_H

#include "hw/boards.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "hw/intc/riscv_imsic.h"

#define LYNX_CPUS_MAX_BITS             9
#define LYNX_CPUS_MAX                  (1 << LYNX_CPUS_MAX_BITS)
#define LYNX_SOCKETS_MAX_BITS          2
#define LYNX_SOCKETS_MAX               (1 << LYNX_SOCKETS_MAX_BITS)

#define TYPE_RISCV_LYNX_MACHINE MACHINE_TYPE_NAME("lynx")
typedef struct RISCVVirtState RISCVVirtState;
DECLARE_INSTANCE_CHECKER(RISCVVirtState, RISCV_LYNX_MACHINE,
                         TYPE_RISCV_LYNX_MACHINE)

typedef enum RISCVVirtAIAType {
    VIRT_AIA_TYPE_NONE = 0,
    VIRT_AIA_TYPE_APLIC,
    VIRT_AIA_TYPE_APLIC_IMSIC,
} RISCVVirtAIAType;

struct RISCVVirtState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    Notifier machine_done;
    DeviceState *platform_bus_dev;
    RISCVHartArrayState soc[LYNX_SOCKETS_MAX];
    DeviceState *irqchip[LYNX_SOCKETS_MAX];
    PFlashCFI01 *flash[2];
    FWCfgState *fw_cfg;

    int fdt_size;
    bool have_aclint;
    RISCVVirtAIAType aia_type;
    int aia_guests;
    char *oem_id;
    char *oem_table_id;
    OnOffAuto acpi;
    const MemMapEntry *memmap;
    struct GPEXHost *gpex_host;
    OnOffAuto iommu_sys;
    uint16_t pci_iommu_bdf;
};

enum {
    LYNX_DEBUG,
    LYNX_MROM,
    LYNX_TEST,
    LYNX_RTC,
    LYNX_CLINT,
    LYNX_ACLINT_SSWI,
    LYNX_PLIC,
    LYNX_APLIC_M,
    LYNX_APLIC_S,
    LYNX_UART0,
    LYNX_VIRTIO,
    LYNX_FW_CFG,
    LYNX_IMSIC_M,
    LYNX_IMSIC_S,
    LYNX_FLASH,
    LYNX_DRAM,
    LYNX_PCIE_MMIO,
    LYNX_PCIE_PIO,
    LYNX_PLATFORM_BUS,
    LYNX_PCIE_ECAM,
    LYNX_IOMMU_SYS,
};

enum {
    LYNX_UART0_IRQ = 10,
    LYNX_RTC_IRQ = 11,
    LYNX_VIRTIO_IRQ = 1, /* 1 to 8 */
    LYNX_VIRTIO_COUNT = 8,
    LYNX_PCIE_IRQ = 0x20, /* 32 to 35 */
    LYNX_IOMMU_SYS_IRQ = 0x24, /* 36-39 */
    VIRT_PLATFORM_BUS_IRQ = 64, /* 64 to 95 */
};

#define VIRT_PLATFORM_BUS_NUM_IRQS 32

#define VIRT_IRQCHIP_NUM_MSIS 255
#define VIRT_IRQCHIP_NUM_SOURCES 96
#define VIRT_IRQCHIP_NUM_PRIO_BITS 3
#define VIRT_IRQCHIP_MAX_GUESTS_BITS 3
#define VIRT_IRQCHIP_MAX_GUESTS ((1U << VIRT_IRQCHIP_MAX_GUESTS_BITS) - 1U)

#define VIRT_PLIC_PRIORITY_BASE 0x00
#define VIRT_PLIC_PENDING_BASE 0x1000
#define VIRT_PLIC_ENABLE_BASE 0x2000
#define VIRT_PLIC_ENABLE_STRIDE 0x80
#define VIRT_PLIC_CONTEXT_BASE 0x200000
#define VIRT_PLIC_CONTEXT_STRIDE 0x1000
#define VIRT_PLIC_SIZE(__num_context) \
    (VIRT_PLIC_CONTEXT_BASE + (__num_context) * VIRT_PLIC_CONTEXT_STRIDE)

#define FDT_PCI_ADDR_CELLS    3
#define FDT_PCI_INT_CELLS     1
#define FDT_PLIC_ADDR_CELLS   0
#define FDT_PLIC_INT_CELLS    1
#define FDT_APLIC_INT_CELLS   2
#define FDT_APLIC_ADDR_CELLS  0
#define FDT_IMSIC_INT_CELLS   0
#define FDT_MAX_INT_CELLS     2
#define FDT_MAX_INT_MAP_WIDTH (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_MAX_INT_CELLS)
#define FDT_PLIC_INT_MAP_WIDTH  (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_PLIC_INT_CELLS)
#define FDT_APLIC_INT_MAP_WIDTH (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_APLIC_INT_CELLS)

bool lynx_is_acpi_enabled(RISCVVirtState *s);
bool lynx_is_iommu_sys_enabled(RISCVVirtState *s);
void lynx_acpi_setup(RISCVVirtState *vms);
uint32_t lynx_imsic_num_bits(uint32_t count);

/*
 * The lynx machine physical address space used by some of the devices
 * namely ACLINT, PLIC, APLIC, and IMSIC depend on number of Sockets,
 * number of CPUs, and number of IMSIC guest files.
 *
 * Various limits defined by LYNX_SOCKETS_MAX_BITS, LYNX_CPUS_MAX_BITS,
 * and VIRT_IRQCHIP_MAX_GUESTS_BITS are tuned for maximum utilization
 * of lynx machine physical address space.
 */

#define LYNX_IMSIC_GROUP_MAX_SIZE      (1U << IMSIC_MMIO_GROUP_MIN_SHIFT)
#if LYNX_IMSIC_GROUP_MAX_SIZE < \
    IMSIC_GROUP_SIZE(LYNX_CPUS_MAX_BITS, VIRT_IRQCHIP_MAX_GUESTS_BITS)
#error "Can't accommodate single IMSIC group in address space"
#endif

#define LYNX_IMSIC_MAX_SIZE            (LYNX_SOCKETS_MAX * \
                                        LYNX_IMSIC_GROUP_MAX_SIZE)
#if 0x4000000 < LYNX_IMSIC_MAX_SIZE
#error "Can't accommodate all IMSIC groups in address space"
#endif

#endif
