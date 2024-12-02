/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2022 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_platform.h"
#include "gc_hal_kernel_platform_emulator.h"
#include <linux/delay.h>

static gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    );

gceSTATUS
_GetGPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T * GPUPhysical
    );

gceSTATUS
_GetCPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T GPUPhysical,
    OUT gctPHYS_ADDR_T * CPUPhysical
    );

#if USE_LINUX_PCIE
#define USE_MSI         1
#define MAX_PCIE_BAR    6

struct _gcsPCIEBarInfo
{
    resource_size_t io_base;
    iomem_t virt_base;
    size_t size;
};

struct _gcsPCIEInfo
{
    struct pci_dev *pdev;
    struct _gcsPCIEBarInfo bar[MAX_PCIE_BAR];
};

struct _gcsPLATFORM_PCIE
{
    struct _gcsPLATFORM base;
    struct _gcsPCIEInfo pcie;
};

static struct _gcsPLATFORM_OPERATIONS pcie_ops =
{
    .adjustParam = _AdjustParam,
    .getGPUPhysical = _GetGPUPhysical,
    .getCPUPhysical = _GetCPUPhysical,
};

static struct _gcsPLATFORM_PCIE emulator_platform =
{
    .base =
    {
        .name = __FILE__,
        .ops  = &pcie_ops,
    },
};

#else

static struct _gcsPLATFORM emulator_platform =
{
    .name = __FILE__,
    .ops  = &pcie_ops,
};
#endif

static gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    )
{
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    struct pci_dev *pdev = pcie_platform->pcie.pdev;

    if (!pdev)
    {
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    Args->irqs[gcvCORE_MAJOR] = pdev->irq;

    Args->contiguousBase = 0;
    Args->contiguousSize = 0;

    Args->registerBases[gcvCORE_MAJOR] = pcie_platform->pcie.bar[PCI_BAR_2].io_base
                                + PCIE_CCM_DDR_NOC_SIZE + DDR0_PHY_CON_SIZE;
    Args->registerSizes[gcvCORE_MAJOR] = 0x10000;
    Args->registerBasesMapped[gcvCORE_MAJOR] = pcie_platform->pcie.bar[PCI_BAR_2].virt_base
                                + PCIE_CCM_DDR_NOC_SIZE + DDR0_PHY_CON_SIZE;

    Args->externalBase = pcie_platform->pcie.bar[PCI_BAR_4].io_base;
    Args->externalSize = pcie_platform->pcie.bar[PCI_BAR_4].size;

    Args->mmuPageTablePool = 0;
    Args->mmuDynamicMap = 0;
#endif
    return gcvSTATUS_OK;
}

gceSTATUS
_GetGPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T * GPUPhysical
    )
{
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    gctPHYS_ADDR_T CPUPhysBaseAddr = pcie_platform->pcie.bar[PCI_BAR_4].io_base;
    gctPHYS_ADDR_T GPUPhysBaseAddr = 0;
    size_t gpuMemSize = pcie_platform->pcie.bar[PCI_BAR_4].size;

    if (CPUPhysical >= CPUPhysBaseAddr && CPUPhysical < (CPUPhysBaseAddr + gpuMemSize))
    {
        *GPUPhysical = CPUPhysical - CPUPhysBaseAddr + GPUPhysBaseAddr;
    }
    else
    {
        *GPUPhysical = CPUPhysical;
    }
#endif
    return gcvSTATUS_OK;
}

gceSTATUS
_GetCPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T GPUPhysical,
    OUT gctPHYS_ADDR_T * CPUPhysical
    )
{
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    gctPHYS_ADDR_T CPUPhysBaseAddr = pcie_platform->pcie.bar[PCI_BAR_4].io_base;
    gctPHYS_ADDR_T GPUPhysBaseAddr = 0;
    size_t gpuMemSize = pcie_platform->pcie.bar[PCI_BAR_4].size;

    if (GPUPhysical >= GPUPhysBaseAddr && GPUPhysical < (GPUPhysBaseAddr + gpuMemSize))
    {
        *CPUPhysical = GPUPhysical - GPUPhysBaseAddr + CPUPhysBaseAddr;
    }
    else
    {
        *CPUPhysical = GPUPhysical;
    }
#endif
    return gcvSTATUS_OK;
}

#if USE_LINUX_PCIE
static int32_t pcie_ep_iatu_init(iomem_t reg_base,pcie_iatu_cfg *iatu_cfg)
{
    uint32_t index;

    if (!reg_base || !iatu_cfg)
    {
        return -EINVAL;
    }
    index = iatu_cfg->iatu_dir_index & 0x0000000F;

    write_mreg32(reg_base, IATU_REGION_CTRL_1_OFF_INBOUND(index), iatu_cfg->control1);
    write_mreg32(reg_base, IATU_REGION_CTRL_2_OFF_INBOUND(index), 0);
    write_mreg32(reg_base, IATU_LWR_BASE_ADDR_OFF_INBOUND(index), iatu_cfg->src_base_low);
    write_mreg32(reg_base, IATU_UPPER_BASE_ADDR_OFF_INBOUND(index), iatu_cfg->src_base_high);
    write_mreg32(reg_base, IATU_LIMIT_ADDR_OFF_INBOUND(index), iatu_cfg->src_base_limit);
    write_mreg32(reg_base, IATU_LWR_TARGET_ADDR_OFF_INBOUND(index), iatu_cfg->dst_base_low);
    write_mreg32(reg_base, IATU_UPPER_TARGET_ADDR_OFF_INBOUND(index), iatu_cfg->dst_base_high);
    write_mreg32(reg_base, IATU_REGION_CTRL_2_OFF_INBOUND(index), iatu_cfg->control2);

    return 0;
}

static int32_t pcie_ep_inbound_init(struct _gcsPCIEInfo *pcie)
{
    pcie_iatu_cfg iatu_inbound;

    /* map pcie controller, ccm and ddr_noc */
    iatu_inbound.iatu_dir_index = IATU_TYPE_INBOUND | IATU_BOUND_INDEX(0);
    iatu_inbound.control1 = 0x00000000L;

    iatu_inbound.src_base_low = (uint32_t)DWORD_LO(pcie->bar[PCI_BAR_2].io_base);
    iatu_inbound.src_base_high = (uint32_t)DWORD_HI(pcie->bar[PCI_BAR_2].io_base);

    iatu_inbound.src_base_limit = iatu_inbound.src_base_low + PCIE_CCM_DDR_NOC_SIZE - 1;

    iatu_inbound.dst_base_low = PCIE_CCM_DDR_NOC_CON_BASE_L_ADDR;
    iatu_inbound.dst_base_high = IPS_H_ADDR;
    iatu_inbound.control2 = IATU_REGION_ENABLE;

    pcie_ep_iatu_init(pcie->bar[PCI_BAR_0].virt_base, &iatu_inbound);

    /* map ddr phy controller */
    iatu_inbound.iatu_dir_index = IATU_TYPE_INBOUND | IATU_BOUND_INDEX(1);
    iatu_inbound.control1 = 0x00000000L;

    iatu_inbound.src_base_low = (uint32_t)DWORD_LO(pcie->bar[PCI_BAR_2].io_base) + PCIE_CCM_DDR_NOC_SIZE;
    iatu_inbound.src_base_high = (uint32_t)DWORD_HI(pcie->bar[PCI_BAR_2].io_base);

    iatu_inbound.src_base_limit = iatu_inbound.src_base_low + DDR0_PHY_CON_SIZE - 1;

    iatu_inbound.dst_base_low = DDR0_CON_BASE_L_ADDR;
    iatu_inbound.dst_base_high = IPS_H_ADDR;
    iatu_inbound.control2 = IATU_REGION_ENABLE;

    pcie_ep_iatu_init(pcie->bar[PCI_BAR_0].virt_base, &iatu_inbound);

    /* map system controllers */
    iatu_inbound.iatu_dir_index = IATU_TYPE_INBOUND | IATU_BOUND_INDEX(2);
    iatu_inbound.control1 = 0x00000000L;
    iatu_inbound.src_base_low = (uint32_t)DWORD_LO(pcie->bar[PCI_BAR_2].io_base)
                                + PCIE_CCM_DDR_NOC_SIZE + DDR0_PHY_CON_SIZE;
    iatu_inbound.src_base_high = (uint32_t)DWORD_HI(pcie->bar[PCI_BAR_2].io_base);

    iatu_inbound.src_base_limit = iatu_inbound.src_base_low + THS_CON_SIZE - 1;

    iatu_inbound.dst_base_low = THS_SYS_CON_BASE_L_ADDR;
    iatu_inbound.dst_base_high = IPS_H_ADDR;
    iatu_inbound.control2 = IATU_REGION_ENABLE;

    pcie_ep_iatu_init(pcie->bar[PCI_BAR_0].virt_base, &iatu_inbound);

    /* map ddr0 */
    iatu_inbound.iatu_dir_index = IATU_TYPE_INBOUND | IATU_BOUND_INDEX(3);
    iatu_inbound.control1 = 0x00000000L;
    iatu_inbound.src_base_low = (uint32_t)DWORD_LO(pcie->bar[PCI_BAR_4].io_base);
    iatu_inbound.src_base_high = (uint32_t)DWORD_HI(pcie->bar[PCI_BAR_4].io_base);

    iatu_inbound.src_base_limit = iatu_inbound.src_base_low + pcie->bar[PCI_BAR_4].size - 1;

    iatu_inbound.dst_base_low = S0_DDR_L_ADDR;
    iatu_inbound.dst_base_high = S0_DDR_H_ADDR;
    iatu_inbound.control2 = IATU_REGION_ENABLE;

    pcie_ep_iatu_init(pcie->bar[PCI_BAR_0].virt_base, &iatu_inbound);

    return 0;
}

static const struct pci_device_id vivpci_ids[] = {
  {
    .class = 0x000000,
    .class_mask = 0x000000,
    .vendor = VIV_PCI_VENDOR_ID,
    .device = VIV_PCI_DEVICE_ID,
    .subvendor = 0,
    .subdevice = 0,
    .driver_data = 0
  }, { /* End: all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, vivpci_ids);

static int get_bar_resource(struct _gcsPCIEInfo *pcie, uint32_t id, bool virtual)
{
    int ret = 0;

    if (!pcie || !pcie->pdev || id > 5)
    {
        return -EINVAL;
    }

    pcie->bar[id].io_base = pci_resource_start(pcie->pdev, id);
    pcie->bar[id].size = pci_resource_len(pcie->pdev, id);
    if (virtual)
    {
        pcie->bar[id].virt_base = ioremap_nocache(pcie->bar[id].io_base,
                                                pcie->bar[id].size);
        if (!pcie->bar[id].virt_base)
        {
            printk("map bar %d failed.\n", id);
            ret = -ENODEV;
        }
    }

    return ret;
}

static void put_bar_resource(struct _gcsPCIEBarInfo *barInfo)
{
    if (!barInfo)
    {
        return;
    }

    if (barInfo->virt_base)
    {
        iounmap(barInfo->virt_base);
    }

    barInfo->io_base = 0;
    barInfo->size = 0;
    barInfo->virt_base = 0;
}

static int gpu_pci_subdriver_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int ret = 0;
    void* ccm_base;
    static u64 dma_mask;

    gcmkPRINT("PCIE DRIVER PROBED");

    ret = pci_enable_device(pdev);
    if (ret) {
        printk("galcore: pci_enable_device() failed.\n");
        return ret;
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
    dma_mask = DMA_BIT_MASK(64);
#else
    dma_mask = DMA_64BIT_MASK;
#endif
    ret = pci_set_dma_mask(pdev, dma_mask);
    if (ret) {
        ret = pci_set_consistent_dma_mask(pdev, dma_mask);
        if (ret) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
            dma_mask = DMA_BIT_MASK(32);
#else
            dma_mask = DMA_32BIT_MASK;
#endif
            printk("galcore: Failed to set 64 bit dma mask.\n");
            ret = pci_set_dma_mask(pdev, dma_mask);
            if (ret) {
                ret = pci_set_consistent_dma_mask(pdev, dma_mask);
                if (ret) {
                    printk("galcore: Failed to set DMA mask.\n");
                    goto error_out;
                }
            }
        }
    }

    pci_set_master(pdev);

    ret = pci_request_regions(pdev, "galcore");
    if (ret) {
        printk("galcore: Failed to get ownership of BAR region.\n");
        goto error_request_regions;
    }

    emulator_platform.pcie.pdev = pdev;
    ret = get_bar_resource(&emulator_platform.pcie, PCI_BAR_0, true);
    if (ret)
    {
        goto error_iomap_bar0;
    }

    ret = get_bar_resource(&emulator_platform.pcie, PCI_BAR_2, true);
    if (ret)
    {
        goto error_iomap_bar2;
    }

    ret = get_bar_resource(&emulator_platform.pcie, PCI_BAR_4, true);
    if (ret)
    {
        goto error_iomap_bar4;
    }

#if USE_MSI
    ret = pci_alloc_irq_vectors(pdev, 1, MSI_NUM, PCI_IRQ_MSI);
    if (ret < 1) {
        printk("galcore: Failed to allocate irq vectors.\n");
        goto error_enable_msi;
    }
#endif
    pcie_ep_inbound_init(&emulator_platform.pcie);

    ccm_base = emulator_platform.pcie.bar[PCI_BAR_2].virt_base + CCM_OFFSET_ADDR;
    /* enable interrupt path except edma */
    write_mreg32(ccm_base, INT_POL_REF_CFG, ~INT_FLAG_EDMA);
    /* eanble zsp_sft, vcd, vce, bigsea */
    write_mreg32(ccm_base, MAIL_BOX_INTERRUPT_CONTROLLER, GLUE_LGIC_INTE);

    return ret;
#if USE_MSI
error_enable_msi:
#endif
    put_bar_resource(&emulator_platform.pcie.bar[PCI_BAR_4]);
error_iomap_bar4:
    put_bar_resource(&emulator_platform.pcie.bar[PCI_BAR_2]);
error_iomap_bar2:
    put_bar_resource(&emulator_platform.pcie.bar[PCI_BAR_0]);
error_iomap_bar0:
    pci_release_regions(pdev);
error_request_regions:
    pci_clear_master(pdev);
error_out:
    pci_disable_device(pdev);

    return ret;
}

static void gpu_pci_subdriver_remove(struct pci_dev *pdev)
{
    put_bar_resource(&emulator_platform.pcie.bar[PCI_BAR_0]);
    put_bar_resource(&emulator_platform.pcie.bar[PCI_BAR_2]);
    put_bar_resource(&emulator_platform.pcie.bar[PCI_BAR_4]);

    pci_set_drvdata(pdev, NULL);
#if USE_MSI
    pci_free_irq_vectors(pdev);
#endif
    pci_clear_master(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    return;
}

static struct pci_driver gpu_pci_subdriver = {
    .name = DEVICE_NAME,
    .id_table = vivpci_ids,
    .probe = gpu_pci_subdriver_probe,
    .remove = gpu_pci_subdriver_remove
};

#endif

static struct platform_device *emulator_dev;

int gckPLATFORM_Init(struct platform_driver *pdrv,
            struct _gcsPLATFORM **platform)
{
    int ret;
    emulator_dev = platform_device_alloc(pdrv->driver.name, -1);

    if (!emulator_dev) {
        printk("galcore: platform_device_alloc failed.\n");
        return -ENOMEM;
    }

    /* Add device */
    ret = platform_device_add(emulator_dev);
    if (ret) {
        printk("galcore: platform_device_add failed.\n");
        goto put_dev;
    }

    *platform = (gcsPLATFORM *)&emulator_platform;

#if USE_LINUX_PCIE
    ret = pci_register_driver(&gpu_pci_subdriver);
    if (ret)
    {
        printk("galcore: pci_register_driver failed.\n");
        goto put_dev;
    }
#endif

    return 0;

put_dev:
    platform_device_put(emulator_dev);

    return ret;
}

int gckPLATFORM_Terminate(struct _gcsPLATFORM *platform)
{
    if (emulator_dev) {
        platform_device_unregister(emulator_dev);
        emulator_dev = NULL;
    }

#if USE_LINUX_PCIE
    pci_unregister_driver(&gpu_pci_subdriver);
#endif

    return 0;
}

