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

//#include <linux/device.h>
//#include <linux/slab.h>
//#include <linux/miscdevice.h>
//#include <linux/uaccess.h>

#include "gc_hal_kernel_wcos.h"
#include "gc_hal_driver.h"
#include "gc_hal_kernel_driver.h"

/* Zone used for header/footer. */
#define _GC_OBJ_ZONE    gcvZONE_DRIVER

// static struct class* gpuClass = NULL;

gckGALDEVICE galDevice;
static BOOLEAN galDeviceIsLocked = FALSE;

static gctUINT major = 199; // major device number for GC device
static gctINT irqLine = 59; // IRQ number of GC core

static gctUINT32 registerMemBase = 0x38000000; // Base of bus address of GC core AHB register

static gctUINT32 registerMemSize = 4 << 16; // Size of bus address range of GC core AHB registe

static gctINT irqLine2D = -1; //IRQ number of G2D core if irqLine is used for a G3D core

static gctUINT32 registerMemBase2D = 0x00000000; // Base of bus address of G2D core if registerMemBase2D is used for a G3D core

static gctUINT32 registerMemSize2D = 2 << 16; // Size of bus address range of G2D core if registerMemSize is used for a G3D core

static gctINT irqLineVG = -1; // IRQ number of VG core

static gctUINT32 registerMemBaseVG = 0x00000000; // Base of bus address of VG core

static gctUINT32 registerMemSizeVG = 2 << 10; // Size of bus address range of VG core

#if gcdDEC_ENABLE_AHB
static ulong registerMemBaseDEC300 = 0x00000000;
module_param(registerMemBaseDEC300, ulong, 0644);

static ulong registerMemSizeDEC300 = 2 << 10;
module_param(registerMemSizeDEC300, ulong, 0644);
#endif

#ifndef gcdDEFAULT_CONTIGUOUS_SIZE
#define gcdDEFAULT_CONTIGUOUS_SIZE (4 << 20)
#endif
static gctUINT32 contiguousSize = gcdDEFAULT_CONTIGUOUS_SIZE; // Size of memory reserved for GC

static gctPHYS_ADDR_T contiguousBase = 0; // Base address of memory reserved for GC, if it is 0, GC driver will try to allocate a buffer whose size defined by contiguousSize

static gctSIZE_T externalSize = 0; // Size of external memory, if it is 0, means there is no external pool

static gctPHYS_ADDR_T externalBase = 0; // Base address of external memory


static gctINT fastClear = -1; // Disable fast clear if set it to 0, enabled by default

static gctINT compression = -1; // Disable compression if set it to 0, enabled by default

static gctINT powerManagement = 1; // Disable auto power saving if set it to 0, enabled by default

static gctINT gpuProfiler = 0; // Enable profiling support, disabled by default

static gctUINT32 baseAddress = 0x40000000; // Only used for old MMU, set it to 0 if memory which can be accessed by GPU falls into 0 - 2G, otherwise set it to 0x80000000

static gctSIZE_T physSize = 0xC0000000; // Obsolete

static gctUINT recovery = 1; // Recover GPU from stuck (1: Enable, 0: Disable)

/* Middle needs about 40KB buffer, Maximal may need more than 200KB buffer. */
static gctUINT stuckDump = 0; // Level of stuck dump content (1: Minimal, 2: Middle, 3: Maximal)

static gctINT showArgs = 1; // Display parameters value when driver loaded

static gctINT mmu = 1; // Disable MMU if set it to 0, enabled by default [Obsolete]

static gctINT irqs[gcvCORE_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // Array of IRQ numbers of multi-GPU

static gctUINT registerBases[gcvCORE_COUNT]; // "Array of bases of bus address of register of multi-GPU

static gctUINT registerSizes[gcvCORE_COUNT] = {2 << 16, 2 << 16, 2 << 16, 2 << 16, 2 << 16, 2 << 16, 2 << 16, 2 << 16, 2 << 16, 2 << 16}; //Array of sizes of bus address range of register of multi-GPU

static gctUINT chipIDs[gcvCORE_COUNT] = {gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT, gcvCHIP_ID_DEFAULT};
 // Array of chipIDs of multi-GPU

static gctUINT type = 0; // 0 - Char Driver (Default), 1 - Misc Driver

static gctINT userClusterMask = 0; // User defined cluster enable mask

static gctINT smallBatch = 1; // Enable/disable small batch

static gctINT allMapInOne = 1; // Mapping kernel video memory to user, 0 means mapping every time, otherwise only mapping one time

/*******************************************************************************
***************************** SRAM description *********************************
*******************************************************************************/

/* Per-core SRAM physical address base, the order of configuration is according to access speed, gcvINVALID_PHYSICAL_ADDRESS means no bus address. */
// Array of base of bus address of SRAM,INTERNAL, EXTERNAL0, EXTERNAL1..., gcvINVALID_PHYSICAL_ADDRESS means no bus address
static gctPHYS_ADDR_T sRAMBases[gcvSRAM_INTER_COUNT * gcvCORE_COUNT] = {
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS,
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS};

/* Per-core SRAM size. */
// Array of size of per-core SRAMs, 0 means no SRAM
static gctUINT sRAMSizes[gcvSRAM_INTER_COUNT * gcvCORE_COUNT] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Shared SRAM physical address bases. */
// Shared SRAM physical address bases.
static gctPHYS_ADDR_T extSRAMBases[gcvSRAM_EXT_COUNT] = {
    gcvINVALID_PHYSICAL_ADDRESS, gcvINVALID_PHYSICAL_ADDRESS};

/* Shared SRAM sizes. */
static gctUINT extSRAMSizes[gcvSRAM_EXT_COUNT] = {0, 0};

// Default 1 means AXI-SRAM is already reserved for GPU, 0 means GPU driver need request the memory region.
static gctUINT sRAMRequested = 1;

// Default 1 means alloc mmu page table in virsual memory, 0 means auto select memory pool.
static gctUINT mmuPageTablePool = 1;

// Default 0 means SRAM pool must be specified when allocating SRAM memory, 1 means SRAM memory will be looped as default pool.
static gctUINT sRAMLoopMode = 0;

// Default 1 means enable mmu dynamic mapping in virsual memory, 0 means disable dynnamic mapping.
static gctUINT mmuDynamicMap = 1;

#if USE_LINUX_PCIE
static int bar = 1;
module_param(bar, int, 0644);
MODULE_PARM_DESC(bar, "PCIE Bar index of GC core");

static int bars[gcvCORE_COUNT] = {[0 ... gcvCORE_COUNT - 1] = -1};
module_param_array(bars, int, NULL, 0644);
MODULE_PARM_DESC(bars, "Array of bar index of PCIE platform for multi-GPU");

static uint regOffsets[gcvCORE_COUNT] = {[0 ... gcvCORE_COUNT - 1] = 0};
module_param_array(regOffsets, uint, NULL, 0644);
MODULE_PARM_DESC(regOffsets, "Array of register offsets in corresponding BAR space");

static int sRAMBars[gcvSRAM_EXT_COUNT] = {[0 ... gcvSRAM_EXT_COUNT - 1] = -1};
module_param_array(sRAMBars, int, NULL, 0644);
MODULE_PARM_DESC(sRAMBars, "Array of SRAM bar index of shared external SRAMs.");

static int sRAMOffsets[gcvSRAM_EXT_COUNT] = {[0 ... gcvSRAM_EXT_COUNT - 1] = -1};
module_param_array(sRAMOffsets, int, NULL, 0644);
MODULE_PARM_DESC(sRAMOffsets, "Array of SRAM offset inside bar of shared external SRAMs.");
#endif

static gctINT gpu3DMinClock = 1;
static gctINT contiguousRequested = 0;
static gctSIZE_T bankSize = 0;


static gcsMODULE_PARAMETERS moduleParam;

static void
_InitModuleParam(
    gcsMODULE_PARAMETERS * ModuleParam
    )
{
    gctUINT i, j;
    gcsMODULE_PARAMETERS *p = ModuleParam;

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        p->irqs[i] = irqs[i];

        if (irqs[i] != -1)
        {
            p->registerBases[i] = registerBases[i];
            p->registerSizes[i] = registerSizes[i];
        }
#if USE_LINUX_PCIE
        p->bars[i] = bars[i];
        p->regOffsets[i] = regOffsets[i];
#endif
    }

    /* Check legacy style. */
#if USE_LINUX_PCIE
    if (bar != -1)
    {
        p->bars[gcvCORE_MAJOR]         = bar;
    }
#endif

    if (irqLine != -1)
    {
        p->irqs[gcvCORE_MAJOR]          = irqLine;
        p->registerBases[gcvCORE_MAJOR] = registerMemBase;
        p->registerSizes[gcvCORE_MAJOR] = registerMemSize;
    }

    if (irqLine2D != -1)
    {
        p->irqs[gcvCORE_2D]          = irqLine2D;
        p->registerBases[gcvCORE_2D] = registerMemBase2D;
        p->registerSizes[gcvCORE_2D] = registerMemSize2D;
    }

    if (irqLineVG != -1)
    {
        p->irqs[gcvCORE_VG]          = irqLineVG;
        p->registerBases[gcvCORE_VG] = registerMemBaseVG;
        p->registerSizes[gcvCORE_VG] = registerMemSizeVG;
    }

#if gcdDEC_ENABLE_AHB
    if (registerMemBaseDEC300 && registerMemSizeDEC300)
    {
        p->registerBases[gcvCORE_DEC] = registerMemBaseDEC300;
        p->registerSizes[gcvCORE_DEC] = registerMemSizeDEC300;
    }
#endif

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        /* Not a module param. */
        p->registerBasesMapped[i] = gcvNULL;
    }

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        p->chipIDs[i] = chipIDs[i];
    }

    p->contiguousBase      = contiguousBase;
    p->contiguousSize      = contiguousSize;
    p->contiguousRequested = contiguousRequested;   /* not a module param. */

    p->externalBase = externalBase;
    p->externalSize = externalSize;

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        for (j = 0; j < gcvSRAM_INTER_COUNT; j++)
        {
            p->sRAMBases[i][j] = sRAMBases[i * gcvSRAM_INTER_COUNT + j];
            p->sRAMSizes[i][j] = sRAMSizes[i * gcvSRAM_INTER_COUNT + j];
        }
    }

    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        p->extSRAMBases[i] = extSRAMBases[i];
        p->extSRAMSizes[i] = extSRAMSizes[i];
#if USE_LINUX_PCIE
        p->sRAMBars[i]     = sRAMBars[i];
        p->sRAMOffsets[i]  = sRAMOffsets[i];
#endif
    }

    p->sRAMRequested = sRAMRequested;
    p->sRAMLoopMode = sRAMLoopMode;

    p->baseAddress = baseAddress;
    p->physSize    = physSize;
    p->bankSize    = bankSize;  /* not a module param. */

    p->recovery        = recovery;
    p->powerManagement = powerManagement;

    p->enableMmu = mmu;
    p->fastClear = fastClear;

    p->compression = (compression == -1) ? gcvCOMPRESSION_OPTION_DEFAULT
                   : (gceCOMPRESSION_OPTION)compression;
    p->gpu3DMinClock   = gpu3DMinClock; /* not a module param. */
    p->userClusterMask = userClusterMask;
    p->smallBatch      = smallBatch;

    p->stuckDump   = stuckDump;
    p->gpuProfiler = gpuProfiler;

    p->deviceType  = type;
    p->showArgs    = showArgs;

    p->mmuPageTablePool = mmuPageTablePool;

    p->mmuDynamicMap = mmuDynamicMap;
    p->allMapInOne = allMapInOne;
#if !gcdENABLE_3D
    p->irqs[gcvCORE_MAJOR]          = irqLine = -1;
    p->registerBases[gcvCORE_MAJOR] = registerMemBase = 0;
    p->registerSizes[gcvCORE_MAJOR] = registerMemSize = 0;
#endif

#if !gcdENABLE_2D
    p->irqs[gcvCORE_2D]          = irqLine2D = -1;
    p->registerBases[gcvCORE_2D] = registerMemBase2D = 0;
    p->registerSizes[gcvCORE_2D] = registerMemSize2D = 0;
#endif

#if !gcdENABLE_VG
    p->irqs[gcvCORE_VG]          = irqLineVG = -1;
    p->registerBases[gcvCORE_VG] = registerMemBaseVG = 0;
    p->registerSizes[gcvCORE_VG] = registerMemSizeVG = 0;
#endif
}

static void
_SyncModuleParam(
    const gcsMODULE_PARAMETERS * ModuleParam
    )
{
    gctUINT i, j;
    gcsMODULE_PARAMETERS *p = &moduleParam;

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        irqs[i]          = p->irqs[i];
        registerBases[i] = (gctUINT32)p->registerBases[i];
        registerSizes[i] = (gctUINT32)p->registerSizes[i];
 #if USE_LINUX_PCIE
        bars[i]          = p->bars[i];
        regOffsets[i]    = p->regOffsets[i];
 #endif
    }

    /* Sync to legacy style. */

#if USE_LINUX_PCIE
    bar               = p->bars[gcvCORE_MAJOR];
#endif
    irqLine           = p->irqs[gcvCORE_MAJOR];
    registerMemBase   = (gctUINT32)p->registerBases[gcvCORE_MAJOR];
    registerMemSize   = (gctUINT32)p->registerSizes[gcvCORE_MAJOR];

    irqLine2D         = p->irqs[gcvCORE_2D];
    registerMemBase2D = (gctUINT32)p->registerBases[gcvCORE_2D];
    registerMemSize2D = (gctUINT32)p->registerSizes[gcvCORE_2D];

    irqLineVG         = p->irqs[gcvCORE_VG];
    registerMemBaseVG = (gctUINT32)p->registerBases[gcvCORE_VG];
    registerMemSizeVG = (gctUINT32)p->registerSizes[gcvCORE_VG];

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        p->chipIDs[i] = chipIDs[i];
    }

    contiguousBase      = (gctUINT32)p->contiguousBase;
    contiguousSize      = (gctUINT32)p->contiguousSize;
    contiguousRequested = p->contiguousRequested;   /* not a module param. */

    externalBase = p->externalBase;
    externalSize = p->externalSize;

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        for (j = 0; j < gcvSRAM_INTER_COUNT; j++)
        {
            sRAMBases[i * gcvSRAM_INTER_COUNT + j] = p->sRAMBases[i][j];
            sRAMSizes[i * gcvSRAM_INTER_COUNT + j] = p->sRAMSizes[i][j];
        }
    }

    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        extSRAMBases[i] = p->extSRAMBases[i];
        extSRAMSizes[i] = p->extSRAMSizes[i];

#if USE_LINUX_PCIE
        sRAMBars[i]     = p->sRAMBars[i];
        sRAMOffsets[i]  = p->sRAMOffsets[i];
#endif
    }

    sRAMRequested = p->sRAMRequested;
    sRAMLoopMode  = p->sRAMLoopMode;

    baseAddress = (gctUINT32)p->baseAddress;
    physSize    = p->physSize;
    bankSize    = p->bankSize;  /* not a module param. */

    recovery        = p->recovery;
    powerManagement = p->powerManagement;

    mmu             = p->enableMmu;
    fastClear       = p->fastClear;
    compression     = p->compression;
    gpu3DMinClock   = p->gpu3DMinClock; /* not a module param. */
    userClusterMask = p->userClusterMask;
    smallBatch      = p->smallBatch;

    stuckDump   = p->stuckDump;
    gpuProfiler = p->gpuProfiler;

    type        = p->deviceType;
    showArgs    = p->showArgs;

    mmuPageTablePool = p->mmuDynamicMap;
    mmuDynamicMap = p->mmuDynamicMap;
    allMapInOne = p->allMapInOne;
}

void
gckDRIVER_SetParams(void)
{
    /* Gather module parameters. */
    _InitModuleParam(&moduleParam);

    if (gcPlatform->ops->adjustParam)
    {
        /* Override default module param. */
        gcPlatform->ops->adjustParam(gcPlatform, &moduleParam);
    }

    /* Update module param because drv_init() uses them directly. */
    _SyncModuleParam(&moduleParam);
}

void
gckOS_DumpParam(
    void
    )
{
    gctINT i, j;

    printk("Galcore options:\n");
    if (irqLine != -1)
    {
        printk("  irqLine           = %d\n", irqLine);
        printk("  registerMemBase   = 0x%08lX\n", registerMemBase);
        printk("  registerMemSize   = 0x%08lX\n", registerMemSize);
    }

    if (irqLine2D != -1)
    {
        printk("  irqLine2D         = %d\n", irqLine2D);
        printk("  registerMemBase2D = 0x%08lX\n", registerMemBase2D);
        printk("  registerMemSize2D = 0x%08lX\n", registerMemSize2D);
    }

    if (irqLineVG != -1)
    {
        printk("  irqLineVG         = %d\n", irqLineVG);
        printk("  registerMemBaseVG = 0x%08lX\n", registerMemBaseVG);
        printk("  registerMemSizeVG = 0x%08lX\n", registerMemSizeVG);
    }
#if USE_LINUX_PCIE
    if (bar != -1)
    {
        printk("  bar               = %d\n", bar);
    }
#endif
#if gcdDEC_ENABLE_AHB
    printk("  registerMemBaseDEC300 = 0x%08lX\n", registerMemBaseDEC300);
    printk("  registerMemSizeDEC300 = 0x%08lX\n", registerMemSizeDEC300);
#endif

    printk("  contiguousSize    = 0x%08lX\n", contiguousSize);
    printk("  contiguousBase    = 0x%llX\n", contiguousBase);
    printk("  externalSize      = 0x%08lX\n", externalSize);
    printk("  externalBase      = 0x%llX\n", externalBase);
    printk("  bankSize          = 0x%08lX\n", bankSize);
    printk("  fastClear         = %d\n", fastClear);
    printk("  compression       = %d\n", compression);
    printk("  powerManagement   = %d\n", powerManagement);
    printk("  baseAddress       = 0x%08lX\n", baseAddress);
    printk("  physSize          = 0x%08lX\n", physSize);
    printk("  recovery          = %d\n", recovery);
    printk("  stuckDump         = %d\n", stuckDump);
    printk("  gpuProfiler       = %d\n", gpuProfiler);
    printk("  userClusterMask   = 0x%x\n", userClusterMask);
    printk("  smallBatch        = %d\n", smallBatch);
    printk("  allMapInOne       = %d\n", allMapInOne);

    printk("  irqs              = ");
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("%d, ", irqs[i]);
    }
    printk("\n");

#if USE_LINUX_PCIE
    printk("  bars              = ");
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("%d, ", bars[i]);
    }
    printk("\n");

    printk("  regOffsets        = ");
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("%d, ", regOffsets[i]);
    }
    printk("\n");

#endif
    printk("  registerBases     = ");
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("0x%08X, ", registerBases[i]);
    }
    printk("\n");

    printk("  registerSizes     = ");
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("0x%08X, ", registerSizes[i]);
    }
    printk("\n");

    printk("  chipIDs           = ");
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("0x%08X, ", chipIDs[i]);
    }
    printk("\n");

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        printk("  core %d internal sRAMBases = ", i);

        for (j = 0; j < gcvSRAM_INTER_COUNT; j++)
        {
            printk("0x%llX, ", sRAMBases[i * gcvSRAM_INTER_COUNT + j]);
        }
        printk("\n");
    }

    printk("  External sRAMBases = ");
    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        printk("0x%llx, ", extSRAMBases[i]);
    }
    printk("\n");

    printk("  mmuPageTablePool  = %d\n", mmuPageTablePool);
    printk("  mmuDynamicMap     = %d\n", mmuDynamicMap);

    printk("Build options:\n");
    printk("  gcdGPU_TIMEOUT    = %d\n", gcdGPU_TIMEOUT);
    printk("  gcdGPU_2D_TIMEOUT = %d\n", gcdGPU_2D_TIMEOUT);
    printk("  gcdINTERRUPT_STATISTIC = %d\n", gcdINTERRUPT_STATISTIC);
}

/* process attaching to device */
gceSTATUS gckDRIVER_DeviceAttach(PDEVICE_OBJECT DeviceObject)
{
#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    gceSTATUS status = gcvSTATUS_OK;
    gctINT i;
    gctINT attached = 0;

    gcmkHEADER_ARG("DeviceObject=%p", DeviceObject);

    /* Attached the process. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (galDevice->kernels[i] != gcvNULL)
        {
            status = gckKERNEL_AttachProcess(galDevice->kernels[i], gcvTRUE);

            if (gcmIS_ERROR(status))
            {
                break;
            }

            attached = i;
        }
    }

    if (gcmIS_ERROR(status))
    {
        /* Error. */
        for (i = 0; i < attached; i++)
        {
            if (galDevice->kernels[i] != gcvNULL)
            {
                gcmkVERIFY_OK(gckKERNEL_AttachProcess(galDevice->kernels[i], gcvFALSE));
            }
        }
        gcmkFOOTER_ARG("status=%d", status);
        return status;
    }

    /* Success. */
    gcmkFOOTER_NO();

    return gcvSTATUS_OK;
}

/* when process detaching from device */
int gckDRIVER_DeviceDetach(PDEVICE_OBJECT DeviceObject)
{
    gcmkHEADER_ARG("DeviceObject=%p", DeviceObject);

#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    gceSTATUS status = gcvSTATUS_OK;

    gctINT i;

    if (galDevice == gcvNULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): device is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (galDeviceIsLocked)
    {
        /* Release the mutex. */
        gcmkONERROR(gckOS_ReleaseMutex(gcvNULL, galDevice->device->commitMutex));
        galDeviceIsLocked = gcvFALSE;
    }

    /* A process gets detached. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (galDevice->kernels[i] != gcvNULL)
        {
            gcmkVERIFY_OK(gckKERNEL_AttachProcessEx(
                galDevice->kernels[i],
                gcvFALSE,
                _GetProcessID()
                ));
        }
    }

OnError:
    gcmkFOOTER();

    return status;
}

/* drv_ioctl - driver ioctl handler */
gceSTATUS gckDRIVER_DeviceIoctl(
    PDEVICE_OBJECT DeviceObject,
    gcsHAL_INTERFACE * iface)
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("DeviceObject=%p",DeviceObject);

#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    if (galDevice == gcvNULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): device is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (iface->command == gcvHAL_DEVICE_MUTEX)
    {
        if (iface->u.DeviceMutex.isMutexLocked == gcvTRUE)
        {
            galDeviceIsLocked = gcvTRUE;
        }
        else
        {
           galDeviceIsLocked = gcvFALSE;
        }
    }

    status = gckDEVICE_Dispatch(galDevice->device, iface);

    /* Redo system call after pending signal is handled. */
    if (status == gcvSTATUS_INTERRUPTED)
    {
        gcmkONERROR(status);
    }

    /* Success. */

OnError:
    gcmkFOOTER();

    return status;
}

/* drv_init - called when the device is added to the driver */
gceSTATUS gckDRIVER_DeviceStart(void)
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER();

    printk(KERN_INFO "Galcore version %d.%d.%d.%d\n",
        gcvVERSION_MAJOR, gcvVERSION_MINOR, gcvVERSION_PATCH, gcvVERSION_BUILD);

    if (showArgs)
    {
        gckOS_DumpParam();
    }

    gckGALDEVICE device = gcvNULL;
    struct class* device_class = gcvNULL;
    int ret = 0;

    if (gcPlatform->registerMemBase.QuadPart != 0)
    {
        moduleParam.registerBases[gcvCORE_MAJOR] = gcPlatform->registerMemBase.QuadPart;
        moduleParam.registerSizes[gcvCORE_MAJOR] = gcPlatform->registerMemSize;
    }

    if (gcPlatform->baseAddress.QuadPart != 0)
    {
        moduleParam.baseAddress = gcPlatform->baseAddress.QuadPart;
        moduleParam.physSize = gcPlatform->physSize;
    }

    /* Create the GAL device. */
    status = gckGALDEVICE_Construct(gcPlatform, &moduleParam, &device);

    if (gcmIS_ERROR(status))
    {
        gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
                       "%s(%d): Failed to create the GAL device: status=%d\n",
                       __FUNCTION__, __LINE__, status);

        goto OnError;
    }

    /* Start the GAL device. */
    gcmkONERROR(gckGALDEVICE_Start(device));

    if ((physSize != 0)
       && (device->kernels[gcvCORE_MAJOR] != gcvNULL)
       && (device->kernels[gcvCORE_MAJOR]->hardware->mmuVersion != 0))
    {
        /* Reset the base address */
        device->baseAddress = 0;
    }

    /* Set global galDevice pointer. */
    galDevice = device;

OnError:
    if (ret)
    {
        if (device)
        {
            gcmkVERIFY_OK(gckGALDEVICE_Stop(device));
            gcmkVERIFY_OK(gckGALDEVICE_Destroy(device));
        }
    }

    gcmkFOOTER();

    return status;
}

void gckDRIVER_DeviceStop(void)
{
    gcmkHEADER();

    if (galDevice != NULL) {

        gcmkVERIFY_OK(gckGALDEVICE_Stop(galDevice));

        gcmkVERIFY_OK(gckGALDEVICE_Destroy(galDevice));
    }

    galDevice = NULL;

    gcmkFOOTER_NO();
}

 /* gpu_suspend - called to suspend the gpu */
//static int gpu_suspend(struct platform_device *dev, pm_message_t state)
static int gpu_suspend(struct platform_device *dev)
{
#if 0
    gceSTATUS status;
    gckGALDEVICE device;
    gctINT i;

    device = platform_get_drvdata(dev);

    if (!device)
    {
        return -1;
    }

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (device->kernels[i] != gcvNULL)
        {
            /* Store states. */
#if gcdENABLE_VG
            if (i == gcvCORE_VG)
            {
                status = gckVGHARDWARE_QueryPowerManagementState(device->kernels[i]->vg->hardware, &device->statesStored[i]);
            }
            else
#endif
            {
                status = gckHARDWARE_QueryPowerState(device->kernels[i]->hardware, &device->statesStored[i]);
            }

            if (gcmIS_ERROR(status))
            {
                return -1;
            }

#if gcdENABLE_VG
            if (i == gcvCORE_VG)
            {
                status = gckVGHARDWARE_SetPowerState(device->kernels[i]->vg->hardware, gcvPOWER_OFF);
            }
            else
#endif
            {
                status = gckHARDWARE_SetPowerState(device->kernels[i]->hardware, gcvPOWER_OFF);
            }

            if (gcmIS_ERROR(status))
            {
                return -1;
            }

        }
    }
#endif
    return 0;
}

/* gpu_resume - called to resume the gpu */
static int gpu_resume(struct platform_device *dev)
{
#if 0
    gceSTATUS status;
    gckGALDEVICE device;
    gctINT i;
    gceCHIPPOWERSTATE   statesStored;

    device = platform_get_drvdata(dev);

    if (!device)
    {
        return -1;
    }

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (device->kernels[i] != gcvNULL)
        {
#if gcdENABLE_VG
            if (i == gcvCORE_VG)
            {
                status = gckVGHARDWARE_SetPowerState(device->kernels[i]->vg->hardware, gcvPOWER_ON);
            }
            else
#endif
            {
                status = gckHARDWARE_SetPowerState(device->kernels[i]->hardware, gcvPOWER_ON);
            }

            if (gcmIS_ERROR(status))
            {
                return -1;
            }

            /* Convert global state to crossponding internal state. */
            switch(device->statesStored[i])
            {
            case gcvPOWER_ON:
                statesStored = gcvPOWER_ON_AUTO;
                break;
            case gcvPOWER_IDLE:
                statesStored = gcvPOWER_IDLE_BROADCAST;
                break;
            case gcvPOWER_SUSPEND:
                statesStored = gcvPOWER_SUSPEND_BROADCAST;
                break;
            case gcvPOWER_OFF:
                statesStored = gcvPOWER_OFF_BROADCAST;
                break;
            default:
                statesStored = device->statesStored[i];
                break;
            }

            /* Restore states. */
#if gcdENABLE_VG
            if (i == gcvCORE_VG)
            {
                status = gckVGHARDWARE_SetPowerState(device->kernels[i]->vg->hardware, statesStored);
            }
            else
#endif
            {
                status = gckHARDWARE_SetPowerState(device->kernels[i]->hardware, statesStored);
            }

            if (gcmIS_ERROR(status))
            {
                return -1;
            }
        }
    }
#endif
    return 0;
}

#ifdef CONFIG_PM_SLEEP
/* gpu_system_suspend - called when system is being suspended */
static int gpu_system_suspend(struct device *dev)
{
    pm_message_t state={0};
    return gpu_suspend(to_platform_device(dev), state);
}

/* gpu_system_resume - calle when system is being resumed */
static int gpu_system_resume(struct device *dev)
{
    return gpu_resume(to_platform_device(dev));
}
#endif

#if 0
static struct platform_driver gpu_driver = {
    .probe      = gpu_probe,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    .remove     = gpu_remove,
#else
    .remove     = __devexit_p(gpu_remove),
#endif

    .suspend    = gpu_suspend,
    .resume     = gpu_resume,

    .driver     = {
        .owner = THIS_MODULE,
        .name   = DEVICE_NAME,
#if defined(CONFIG_PM) && LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
        .pm     = &gpu_pm_ops,
#endif
    }
};
#endif


