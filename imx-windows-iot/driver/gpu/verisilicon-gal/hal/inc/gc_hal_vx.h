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

#ifndef __gc_hal_user_vx_h_
#define __gc_hal_user_vx_h_

#if gcdUSE_VX
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
****************************** Object Declarations *****************************
\******************************************************************************/

#if gcdVX_OPTIMIZER > 1
#define gcsVX_KERNEL_PARAMETERS     gcoVX_Hardware_Context

typedef gcoVX_Hardware_Context *    gcsVX_KERNEL_PARAMETERS_PTR;
#else
/* VX kernel parameters. */
typedef struct _gcsVX_KERNEL_PARAMETERS * gcsVX_KERNEL_PARAMETERS_PTR;

#ifdef VSIMULATOR_DEBUG
typedef enum
{
    VX_KERNEL_USR     = 0,
    VX_KERNEL_STD,
    VX_KERNEL_VXC,
    VX_KERNEL_NN,
    VX_KERNEL_CL,
    VX_KERNEL_TP,
}
vx_kernel_type;

#endif

typedef struct _gcsVX_KERNEL_PARAMETERS
{
    gctUINT32           kernel;

    gctUINT32           step;

    gctUINT32           xmin;
    gctUINT32           xmax;
    gctUINT32           xstep;

    gctUINT32           ymin;
    gctUINT32           ymax;
    gctUINT32           ystep;

    gctUINT32           groupSizeX;
    gctUINT32           groupSizeY;

    gctUINT32           threadcount;
    gctUINT32           policy;
    gctUINT32           rounding;
    gctFLOAT            scale;
    gctFLOAT            factor;
    gctUINT32           borders;
    gctUINT32           constant_value;
    gctUINT32           volume;
    gctUINT32           clamp;
    /*gctUINT32           inputhack;*/
    /*gctUINT32           outputhack;*/

    gctUINT32           inputMultipleWidth;
    gctUINT32           outputMultipleWidth;

    gctUINT32           order;

    gctUINT32           input_type[10];
    gctUINT32           output_type[10];
    gctUINT32           input_count;
    gctUINT32           output_count;

    gctINT16            *matrix;
    gctINT16            *matrix1;
    gctUINT32           col;
    gctUINT32           row;

    gcsSURF_NODE_PTR    node;

    gceSURF_FORMAT      inputFormat;
    gceSURF_FORMAT      outputFormat;

    gctUINT8            isUseInitialEstimate;
    gctINT32            maxLevel;
    gctINT32            winSize;

    gcoVX_Instructions  instructions;

    vx_evis_no_inst_s   evisNoInst;


    gctUINT32               optionalOutputs[3];
    gctBOOL                 hasBarrier;
    gctBOOL                 hasAtomic;
}
gcsVX_KERNEL_PARAMETERS;
#endif

#define MAX_GPU_CORE_COUNT 8

/******************************************************************************\
****************************** API Declarations *****************************
\******************************************************************************/
gceSTATUS
gcoVX_Initialize(vx_evis_no_inst_s *evisNoInst);

gceSTATUS
gcoVX_BindImage(
    IN gctUINT32            Index,
    IN gcsVX_IMAGE_INFO_PTR Info
    );

gceSTATUS
gcoVX_BindUniform(
    IN gctUINT32            RegAddress,
    IN gctUINT32            Index,
    IN gctUINT32            *Value,
    IN gctUINT32            Num
    );

gceSTATUS
    gcoVX_SetImageInfo(
    IN  gcUNIFORM,
    IN gcsVX_IMAGE_INFO_PTR Info
    );

gceSTATUS
gcoVX_Commit(
    IN gctBOOL Flush,
    IN gctBOOL Stall,
    INOUT gctPOINTER *pCmdBuffer,
    INOUT gctUINT32  *pCmdBytes
    );

gceSTATUS
gcoVX_Replay(
    IN gctPOINTER CmdBuffer,
    IN gctUINT32  CmdBytes
    );

gceSTATUS
gcoVX_InvokeKernel(
    IN gcsVX_KERNEL_PARAMETERS_PTR  Parameters
    );

gceSTATUS
gcoVX_AllocateMemory(
    IN gctUINT32        Size,
    OUT gctPOINTER*     Logical,
    OUT gctUINT32*      Physical,
    OUT gcsSURF_NODE_PTR* Node
    );

gceSTATUS
gcoVX_FreeMemory(
    IN gcsSURF_NODE_PTR Node
    );

gceSTATUS
gcoVX_DestroyNode(
    IN gcsSURF_NODE_PTR Node
    );

gceSTATUS
gcoVX_KernelConstruct(
    IN OUT gcoVX_Hardware_Context   *Context
    );

gceSTATUS
gcoVX_SetHardwareType(
    IN gceHARDWARE_TYPE Type
    );

#ifdef VSIMULATOR_DEBUG
gceSTATUS gcoVX_SetDebugShaderCount(IN gctPOINTER graph, IN gctUINT32 count);

gceSTATUS
gcoVX_addDebugShader(INOUT gctUINT32 *debugShaderIndex,
    IN gctCHAR* name,
    IN gctPOINTER graph,
    IN gctPOINTER node,
    IN gctPOINTER operation,
    IN gctPOINTER program,
    IN gctPOINTER parent,
    IN gctCHAR* source,
    IN gctPOINTER debugInfo,
    IN gctUINT32 border_mode,
    IN gctUINT32 type,
    IN gctINT32  index,
    IN gctUINT32 instcount,
    IN gctUINT32 nnLev,
    IN gcsSURF_NODE_PTR memnode,
    IN gctCHAR* opFunc,
    IN gctUINT32 cmdAddr,
    IN gctINT32 opCmdID,
    IN gctINT32 opID,
    IN gctINT32 layerID,
    IN gctINT32 graphID,
    IN gctINT32 uid,
    IN gctBOOL first
    );

gceSTATUS
gcoVX_QueryShader(IN gctUINT32 Physical,
    IN gctPOINTER Graph,
    IN gctINT32 Index,
    OUT gctPOINTER* Shader,
    OUT gctUINT32 *NextCmdAddr);

gceSTATUS
gcoVX_QueryShaderList(IN gctUINT32 Physical,
    IN gctPOINTER Graph,
    OUT VSIMULATOR_SHADER_PTR** List,
    OUT gctUINT32* Count);

gceSTATUS
gcoVX_cleanDebugShader(gctPOINTER graph);

#endif

gceSTATUS
gcoVX_LockKernel(
    IN OUT gcoVX_Hardware_Context   *Context
    );

gceSTATUS
gcoVX_BindKernel(
    IN OUT gcoVX_Hardware_Context   *Context
    );

gceSTATUS
gcoVX_LoadKernelShader(
    IN gcsPROGRAM_STATE *ProgramState
    );

gceSTATUS
gcoVX_InvokeKernelShader(
    IN gcSHADER            Kernel,
    IN gctUINT             WorkDim,
    IN size_t              GlobalWorkOffset[3],
    IN size_t              GlobalWorkScale[3],
    IN size_t              GlobalWorkSize[3],
    IN size_t              LocalWorkSize[3],
    IN gctUINT             ValueOrder,
    IN gctBOOL             BarrierUsed,
    IN gctUINT32           MemoryAccessFlag,
    IN gctBOOL             bDual16
    );

gceSTATUS
gcoVX_Flush(
    IN gctBOOL      Stall
    );

gceSTATUS
gcoVX_TriggerAccelerator(
    IN gctUINT32              CmdAddress,
    IN gceVX_ACCELERATOR_TYPE Type,
    IN gctUINT32              EventId,
    IN gctBOOL                waitEvent,
    IN gctUINT32              gpuId,
    IN gctBOOL                sync,
    IN gctUINT32              syncEventID
    );

gceSTATUS
gcoVX_ProgrammCrossEngine(
    IN gctPOINTER                Data,
    IN gceVX_ACCELERATOR_TYPE    Type,
    IN gctPOINTER                Options,
    IN OUT gctUINT32_PTR        *Instruction
    );

gceSTATUS
gcoVX_SetNNImage(
    IN gctPOINTER Data,
    IN OUT gctUINT32_PTR *Instruction
    );

gceSTATUS
gcoVX_QueryDeviceCount(
    OUT gctUINT32 * DeviceCount
    );

gceSTATUS
gcoVX_QueryCoreCount(
    IN gctUINT32  DeviceID,
    OUT gctUINT32 *CoreCount
    );

gceSTATUS
gcoVX_QueryMultiCore(
    OUT gctBOOL *IsMultiCore
    );

gceSTATUS
gcoVX_GetNNConfig(
    IN OUT gctPOINTER Config
    );

gceSTATUS
gcoVX_QueryHWChipInfo(
    IN OUT vx_hw_chip_info * HwChipInfo
    );

gceSTATUS
gcoVX_WaitNNEvent(
    gctUINT32 EventId
    );

gceSTATUS
gcoVX_FlushCache(
    IN gctBOOL      FlushICache,
    IN gctBOOL      FlushPSSHL1Cache,
    IN gctBOOL      FlushNNL1Cache,
    IN gctBOOL      FlushTPL1Cache,
    IN gctBOOL      FlushSHL1Cache,
    IN gctBOOL      Stall
    );

gceSTATUS
gcoVX_AllocateMemoryEx(
    IN OUT gctUINT *        Bytes,
    IN  gceSURF_TYPE        Type,
    IN  gcePOOL             Pool,
    IN  gctUINT32           alignment,
    OUT gctUINT32 *         Physical,
    OUT gctPOINTER *        Logical,
    OUT gcsSURF_NODE_PTR *  Node
    );

gceSTATUS
gcoVX_AllocateMemoryExAddAllocflag(
    IN OUT gctUINT *        Bytes,
    IN  gceSURF_TYPE        Type,
    IN  gctUINT32           alignment,
    IN  gctUINT32           allocflag,
    OUT gctUINT32 *         Physical,
    OUT gctPOINTER *        Logical,
    OUT gctUINT32 * CpuPhysicalAddress,
    OUT gcsSURF_NODE_PTR *  Node
    );

gceSTATUS
gcoVX_FreeMemoryEx(
    IN gcsSURF_NODE_PTR     Node,
    IN gceSURF_TYPE         Type
    );


gceSTATUS
gcoVX_GetMemorySize(
    OUT gctUINT32_PTR Size
    );

gceSTATUS
gcoVX_ZeroMemorySize();

gceSTATUS
gcoVX_SwitchContext(
    IN  gctUINT DeviceID,
    OUT gcoHARDWARE *SavedHardware,
    OUT gceHARDWARE_TYPE *SavedType,
    OUT gctUINT32    *SavedCoreIndex
    );

gceSTATUS
gcoVX_RestoreContext(
    IN gcoHARDWARE Hardware,
    gceHARDWARE_TYPE PreType,
    gctUINT32 PreCoreIndex
    );

gctBOOL gcoVX_VerifyHardware();


gceSTATUS
gcoVX_CaptureState(
    IN OUT gctUINT8 *CaptureBuffer,
    IN gctUINT32 InputSizeInByte,
    IN OUT gctUINT32 *OutputSizeInByte,
    IN gctBOOL Enabled,
    IN gctBOOL dropCommandEnabled
    );

gceSTATUS
gcoVX_SetRemapAddress(
    IN gctUINT32 remapStart,
    IN gctUINT32 remapEnd,
    IN gceVX_REMAP_TYPE remapType
    );

gceSTATUS
gcoVX_ProgrammYUV2RGBScale(
    IN gctPOINTER Data,
    IN gctUINT32  gpuId,
    IN gctBOOL    mGpuSync
    );

gceSTATUS
gcoVX_CreateHW(
    IN gctUINT32  DeviceID,
    IN gctUINT32  CoreCountPerDevice,
    IN gctUINT32  LocalCoreIndexs[],
    IN gctUINT32  GlobalCoreIndexs[],
    OUT gcoHARDWARE * Hardware
    );

gceSTATUS
gcoVX_DestroyHW(
    IN gcoHARDWARE Hardware
    );

gceSTATUS gcoVX_GetEvisNoInstFeatureCap(
    OUT vx_evis_no_inst_s *EvisNoInst
    );

gceSTATUS gcoVX_MultiGPUSync(
    OUT gctUINT32_PTR *Memory
    );

#ifdef __cplusplus
}
#endif
#endif
#endif /* __gc_hal_user_vx_h_ */

