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

#ifndef __gc_hal_kernel_platform_emulator_h_
#define __gc_hal_kernel_platform_emulator_h_
#include <linux/io.h>

#define VIV_PCI_VENDOR_ID           0x1d9b
#define VIV_PCI_DEVICE_ID           0xface

#define PCI_BAR_0                   0x0
#define PCI_BAR_2                   0x2
#define PCI_BAR_4                   0x4

#define IATU_TYPE_INBOUND                   (0x80000000UL)
#define IATU_TYPE_OUTBOUND                  (0x00000000UL)
#define IATU_BOUND_INDEX(x)                 (x)
#define IATU_REGION_ENABLE                  (0x80000000UL)

/* ATU_CAP registers */
#define IATU_BASE                               0x8100
#define IATU_REGION_CTRL_1_OFF_INBOUND(i)       (IATU_BASE + 0x200 * (i) + 0x00)
#define IATU_REGION_CTRL_2_OFF_INBOUND(i)       (IATU_BASE + 0x200 * (i) + 0x04)
#define IATU_LWR_BASE_ADDR_OFF_INBOUND(i)       (IATU_BASE + 0x200 * (i) + 0x08)
#define IATU_UPPER_BASE_ADDR_OFF_INBOUND(i)     (IATU_BASE + 0x200 * (i) + 0x0c)
#define IATU_LIMIT_ADDR_OFF_INBOUND(i)          (IATU_BASE + 0x200 * (i) + 0x10)
#define IATU_LWR_TARGET_ADDR_OFF_INBOUND(i)     (IATU_BASE + 0x200 * (i) + 0x14)
#define IATU_UPPER_TARGET_ADDR_OFF_INBOUND(i)   (IATU_BASE + 0x200 * (i) + 0x18)

#define BAR_PCIE_MAP_SIZE           0x00080000
#define IPS_H_ADDR                  0x00000001

/* PCIe AXi master view two silics DDR address */
#define S0_DDR_L_ADDR               0x04000000
#define S0_DDR_H_ADDR               0x00000000
#define S1_DDR_L_ADDR               0x84000000
#define S1_DDR_H_ADDR               0x00000000
/* PCIe AXI master view pcie, addr and other ips controller offset */
#define PCIE_CCM_DDR_NOC_CON_BASE_L_ADDR 0x00000000
#define DDR0_CON_BASE_L_ADDR        0x02000000
#define DDR1_CON_BASE_L_ADDR        0x04000000
#define THS_SYS_CON_BASE_L_ADDR     0x06000000

#define S0_DDR_SIZE                 0x80000000
#define S1_DDR_SIZE                 0x80000000
#define S0_DDR_TCACHE_SIZE          0x04000000
#define S1_DDR_TCACHE_SIZE          0x04000000

/* pcie, ccm and ddr_noc size */
#define PCIE_CCM_DDR_NOC_SIZE       0x01100000
/* ddr_a and ddr_b size */
#define DDR0_PHY_CON_SIZE           0x02000000
/* ddr_a and ddr_b size */
#define DDR1_PHY_CON_SIZE           0x02000000
/* ths and sys con size */
#define THS_CON_SIZE                0x00e00000

#define CCM_OFFSET_ADDR             0x00400000
#define CCM_MAP_SIZE                0x00030000
#define MSI_NUM                     32

/* global int flag */
#define INT_FLAG_EDMA               (1 << 0)
#define INT_FLAG_HW_MONITOR         (1 << 2)
#define INT_FLAG_THS0_VCD_A         (1 << 7)
#define INT_FLAG_THS0_VCD_B         (1 << 9)
#define INT_FLAG_THS1_VCD_A         (1 << 8)
#define INT_FLAG_THS1_VCD_B         (1 << 10)
#define INT_FLAG_THS0_VCE           (1 << 11)
#define INT_FLAG_THS1_VCE           (1 << 12)
#define INT_FLAG_THS0_BIGSEA        (1 << 15)
#define INT_FLAG_THS1_BIGSEA        (1 << 16)
#define GLUE_LGIC_INTE              (\
        INT_FLAG_HW_MONITOR | INT_FLAG_THS0_VCD_A | INT_FLAG_THS0_VCD_B |\
        INT_FLAG_THS1_VCD_A | INT_FLAG_THS1_VCD_B | INT_FLAG_THS0_VCE |\
        INT_FLAG_THS1_VCE | INT_FLAG_THS0_BIGSEA | INT_FLAG_THS1_BIGSEA)

#define MAIL_BOX_INTERRUPT_CONTROLLER   0x00020300
/* pcie glue logic registers */
#define INT_POL_REF_CFG             0x000200ac

#define DWORD_LO(a) ((unsigned long long)(a) & 0xffffffff)
#define DWORD_HI(a) ((unsigned long long)(a) >> 32)

typedef void __iomem *      iomem_t;

#define write_mreg32(_base, _reg, _val) writel((_val), (volatile iomem_t)(_base + _reg))
#define read_mreg32(_base, _reg) readl((volatile iomem_t)(_base + _reg))

typedef struct __pcie_iatu_cfg
{
    uint32_t iatu_dir_index;
    uint32_t control1;
    uint32_t control2;
    uint32_t src_base_low;
    uint32_t src_base_high;
    uint32_t src_base_limit;
    uint32_t dst_base_low;
    uint32_t dst_base_high;
}pcie_iatu_cfg;
#endif

