/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "vpu_memory.tmh"
#endif
#include "imx8q_driver.h"
#include <svc/rpc.h>
#include <svc/pm/pm_api.h>
#include <svc/rm/rm_api.h>
#include "Device.h"
#include "imxblit_public.h"



extern NTSTATUS RegisterMuInterruptHandler(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr,
                                           const CM_PARTIAL_RESOURCE_DESCRIPTOR *InterruptRaw,
                                           const CM_PARTIAL_RESOURCE_DESCRIPTOR *InterruptTranslated); // FIXME Put declaration and definition to the right place.


NTSTATUS map_reg(const CM_PARTIAL_RESOURCE_DESCRIPTOR **MemResListPtr, UINT32 idx, UINT32 Size,
                 PHYSICAL_ADDRESS *RegistersPhyAddr, void **RegistersVirtAddr)
{
    DBG_DEV_PRINT_INFO("Mapping registers.");
    if (MemResListPtr == NULL) {
        return STATUS_INVALID_PARAMETER_1;
    }
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *memResourcePtr = MemResListPtr[idx];

    if (memResourcePtr->Type != CmResourceTypeMemory) {
        return STATUS_INVALID_PARAMETER_10;
    }
    UINT32 size = Size > 0 ? Size : memResourcePtr->u.Memory.Length;
    *RegistersPhyAddr = memResourcePtr->u.Memory.Start;
    *RegistersVirtAddr = MmMapIoSpaceEx(memResourcePtr->u.Memory.Start, size, PAGE_READWRITE | PAGE_NOCACHE);

    if (RegistersVirtAddr == NULL) {
        DBG_PRINT_ERROR("MmMapIoSpaceEx(...) failed. (memResourcePtr->u.Memory.Start = 0x%llx, sizeof(MALONE_MU_REGISTERS) = %lu)",
                        RegistersPhyAddr->QuadPart, size);

        return STATUS_INSUFFICIENT_RESOURCES;
    }
    return STATUS_SUCCESS;
}

/**
 * init_vpu_sys_memory()
 * @brief Retrieves memory and interrupt requirements from ACPI table & alloc
 * @param deviceContextPtr Pointer to device context
 * @param ResourcesTranslated
 * @return NTSTATUS
 */
_Use_decl_annotations_
NTSTATUS
init_vpu_sys_memory(
    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr,
    WDFCMRESLIST ResourcesRaw,
    WDFCMRESLIST ResourcesTranslated
)
{
    DBG_DEV_METHOD_BEG();
    UNREFERENCED_PARAMETER(ResourcesRaw);

    const CM_PARTIAL_RESOURCE_DESCRIPTOR *memResourcePtr[11] = { 0 };
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *scfwResourceIpcPtr = NULL;
    ULONG memoryResourceCount = 0;
    ULONG interruptResourceCount = 0;

    /* Look for single memory and interrupt resource. */
    DBG_DEV_PRINT_INFO("Look for single memory and interrupt resource in ACPI.");
    const ULONG resourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < resourceCount; i++) {
        const CM_PARTIAL_RESOURCE_DESCRIPTOR *resourcePtr =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);

        switch (resourcePtr->Type) {

            /* Memory */
            case CmResourceTypeMemory:
                if (memResourcePtr[memoryResourceCount] != NULL) {

                    DBG_PRINT_ERROR("Received unexpected memory resource. (resourcePtr = 0x%p)",
                                    resourcePtr);
                    DBG_DEV_METHOD_END();
                    return STATUS_DEVICE_CONFIGURATION_ERROR;
                }

                memResourcePtr[memoryResourceCount] = resourcePtr;
                ++memoryResourceCount;
                break;
            case CmResourceTypeInterrupt: /* Interrupt */
                if (interruptResourceCount > 2) {
                    DBG_PRINT_ERROR(
                        "Received unexpected interrupt resource. "
                        "(interruptResourceCount = %lu, resourcePtr = 0x%p)",
                        interruptResourceCount,
                        resourcePtr);
                    DBG_DEV_METHOD_END();
                    return STATUS_DEVICE_CONFIGURATION_ERROR;
                }
                ++interruptResourceCount;
                break;
            case CmResourceTypeConnection:
                if ((resourcePtr->u.Connection.Type != CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)) {
                    DBG_PRINT_ERROR("Received unexpected connection resource type.");

                    return STATUS_DEVICE_CONFIGURATION_ERROR;
                }
                if (scfwResourceIpcPtr == NULL) {
                    scfwResourceIpcPtr = resourcePtr;
                    deviceContextPtr->scfw_ipc_id.ipc_id.LowPart = scfwResourceIpcPtr->u.Connection.IdLowPart;
                    deviceContextPtr->scfw_ipc_id.ipc_id.HighPart = scfwResourceIpcPtr->u.Connection.IdHighPart;
                } else {
                    DBG_PRINT_ERROR("Received unexpected connection resource count.");
                    return STATUS_DEVICE_CONFIGURATION_ERROR;
                }
                break;
        }
    }


    /* Check received memory requirements */
    DBG_DEV_PRINT_INFO("Checking received memory requirements.");
    for (ULONG i = 0; i < memoryResourceCount;
         i++) { // FIXME: The memoryResourceCount is result of previous loop not hardcoded number. Thus this loop checks only that there isn't typo in '=' in the code above.
        if (memResourcePtr[i] == NULL) {
            DBG_PRINT_ERROR(
                "Did not receive required memory resource"
                "(memResourcePtr = 0x%p)",
                memResourcePtr[i]);
            DBG_DEV_METHOD_END();
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }
    }


    /* Check received interrupt requirements */
    DBG_DEV_PRINT_INFO("Checking received interrupt requirements.");
    if (interruptResourceCount != 2) {
        DBG_PRINT_ERROR(
            "Did not receive required interrupt resource. "
            "interruptResourceCount = %lu)",
            interruptResourceCount);
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }


    /* Check received memory sizes from ACPI */
    DBG_DEV_PRINT_INFO("Checking received memory sizes from ACPI.");
    for (ULONG i = 0; i < memoryResourceCount; i++) {
        size_t size = 0;
        if (i == VPU_REGS_BASE_res_idx) {
            size = sizeof(MALONE_VPU_REGISTERS);
        } else if (i == VPU_MU0_BASE_res_idx) {
            size = sizeof(MALONE_MU_REGISTERS);
        } else if (i == VPU_FW_BASE_res_idx) {
            size = sizeof(MALONE_VPU_FW);
        } else if (i == VPU_RPC_BASE_res_idx) {
            size = sizeof(MALONE_VPU_RPC);
        }

        // TODO add DPU checks
        // Peter Cach might pose size requirement on Memory in the if-else above. Default is greater than zero.
        if (memResourcePtr[i]->u.Memory.Length < size) {
            DBG_PRINT_ERROR(
                "Memory resource is too small. "
                "(memResourcePtr[%lu]->u.Memory.Length = %lu, "
                "sizeof ) = %lu)",
                i,
                memResourcePtr[i]->u.Memory.Length,
                (unsigned long)size);

            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }
    }


    /* Map VPU REGS */
    DBG_DEV_PRINT_INFO("Mapping VPU registers.");
    NT_ASSERT(memResourcePtr[VPU_REGS_BASE_res_idx]->Type == CmResourceTypeMemory);
    deviceContextPtr->RegistersPhy = memResourcePtr[VPU_REGS_BASE_res_idx]->u.Memory.Start;
    deviceContextPtr->RegistersPtr = (MALONE_VPU_REGISTERS *)
                                     MmMapIoSpaceEx(
                                         memResourcePtr[VPU_REGS_BASE_res_idx]->u.Memory.Start,
                                         sizeof(MALONE_VPU_REGISTERS),
                                         PAGE_READWRITE | PAGE_NOCACHE);

    if (deviceContextPtr->RegistersPtr == NULL) {
        DBG_PRINT_ERROR(
            "MmMapIoSpaceEx(...) failed. "
            "(memResourcePtr->u.Memory.Start = 0x%llx, "
            "sizeof(IMXVPU_REGISTERS) = %lu)",
            memResourcePtr[VPU_REGS_BASE_res_idx]->u.Memory.Start.QuadPart,
            sizeof(MALONE_VPU_REGISTERS));

        return STATUS_INSUFFICIENT_RESOURCES;
    }
#if 0 /* MDL for provide access registers from user space */
    deviceContextPtr->RegistersLen = vpuLength;

    deviceContextPtr->MdlRegisters = IoAllocateMdl(deviceContextPtr->RegistersPtr, vpuLength, FALSE, FALSE, NULL);
    if (deviceContextPtr->MdlRegisters == NULL) {
        IMXVPU_LOG_LOW_MEMORY(
            "IoAllocateMdl(...) failed. "
            "(RegistersPtr = 0x%p)",
            deviceContextPtr->RegistersPtr);

        return STATUS_INSUFFICIENT_RESOURCES;
    }
    MmBuildMdlForNonPagedPool(deviceContextPtr->MdlRegisters);
#endif

    /* Map MU REGS */
    DBG_DEV_PRINT_INFO("Mapping MU registers.");
    NT_ASSERT(memResourcePtr[VPU_MU0_BASE_res_idx]->Type == CmResourceTypeMemory);
    deviceContextPtr->RegistersMUPhy = memResourcePtr[VPU_MU0_BASE_res_idx]->u.Memory.Start;
    deviceContextPtr->RegistersMUPtr = (MALONE_MU_REGISTERS *)
                                       MmMapIoSpaceEx(
                                           memResourcePtr[VPU_MU0_BASE_res_idx]->u.Memory.Start,
                                           sizeof(MALONE_MU_REGISTERS),
                                           PAGE_READWRITE | PAGE_NOCACHE);

    if (deviceContextPtr->RegistersMUPtr == NULL) {
        DBG_PRINT_ERROR(
            "MmMapIoSpaceEx(...) failed. "
            "(memResourcePtr->u.Memory.Start = 0x%llx, "
            "sizeof(MALONE_MU_REGISTERS) = %lu)",
            memResourcePtr[VPU_MU0_BASE_res_idx]->u.Memory.Start.QuadPart,
            sizeof(MALONE_MU_REGISTERS));

        return STATUS_INSUFFICIENT_RESOURCES;
    }


    /* Map FW_BASE */
    DBG_DEV_PRINT_INFO("Mapping FW memory space.");
    NT_ASSERT(memResourcePtr[VPU_FW_BASE_res_idx]->Type == CmResourceTypeMemory);
    deviceContextPtr->FWBasePhy = memResourcePtr[VPU_FW_BASE_res_idx]->u.Memory.Start;
    deviceContextPtr->FWBasePtr = (MALONE_VPU_FW *)
                                  MmMapIoSpaceEx(
                                      memResourcePtr[VPU_FW_BASE_res_idx]->u.Memory.Start,
                                      sizeof(MALONE_VPU_FW),
                                      PAGE_READWRITE | PAGE_NOCACHE);

    if (deviceContextPtr->FWBasePtr == NULL) {
        DBG_PRINT_ERROR(
            "MmMapIoSpaceEx(...) failed. "
            "(memResourcePtr->u.Memory.Start = 0x%llx, "
            "sizeof(MALONE_VPU_FW) = %lu)",
            memResourcePtr[VPU_FW_BASE_res_idx]->u.Memory.Start.QuadPart,
            sizeof(MALONE_VPU_FW));

        return STATUS_INSUFFICIENT_RESOURCES;
    }

    /* Map RPC_BASE */
    DBG_DEV_PRINT_INFO("Mapping RPC memory space.");
    NT_ASSERT(memResourcePtr[VPU_RPC_BASE_res_idx]->Type == CmResourceTypeMemory);
    deviceContextPtr->RPCBasePhy = memResourcePtr[VPU_RPC_BASE_res_idx]->u.Memory.Start;
    deviceContextPtr->RPCBasePtr = (MALONE_VPU_RPC *)
                                   MmMapIoSpaceEx(
                                       memResourcePtr[VPU_RPC_BASE_res_idx]->u.Memory.Start,
                                       sizeof(MALONE_VPU_RPC),
                                       PAGE_READWRITE | PAGE_NOCACHE);

    if (deviceContextPtr->RPCBasePtr == NULL) {
        DBG_PRINT_ERROR(
            "MmMapIoSpaceEx(...) failed. "
            "(memResourcePtr->u.Memory.Start = 0x%llx, "
            "sizeof(MALONE_VPU_RPC) = %lu)",
            memResourcePtr[VPU_RPC_BASE_res_idx]->u.Memory.Start.QuadPart,
            sizeof(MALONE_VPU_RPC));

        return STATUS_INSUFFICIENT_RESOURCES;
    }
    memset((void *)deviceContextPtr->RPCBasePtr, 0, sizeof(MALONE_VPU_RPC));


    /* Map Control status Registers of M4 core */
    DBG_DEV_PRINT_INFO("Mapping Control status registers of VPU core.");
    PHYSICAL_ADDRESS phy = { 0 };
    phy.QuadPart = 0x2D040000;
    deviceContextPtr->csr_offset = (uintptr_t) MmMapIoSpaceEx(phy, 4, PAGE_READWRITE | PAGE_NOCACHE);

    phy.QuadPart = 0x2D040004;
    deviceContextPtr->csr_cpuwait = (uintptr_t) MmMapIoSpaceEx(phy, 4, PAGE_READWRITE | PAGE_NOCACHE);


    /* Map blitter registers and ISR routine. */
    NTSTATUS status;
    {
        BLITTER_DEVICE_CONTEXT *blitterCtxPtr = &deviceContextPtr->BlitterCtx;

        status = map_reg(memResourcePtr, VPU_BLIT_DC_res_idx, 0, &blitterCtxPtr->DcRegPhy, &blitterCtxPtr->DcRegPtr);
        if (!NT_SUCCESS(status)) {
            return status;
        }
        status = map_reg(memResourcePtr, VPU_BLIT_PRG0_res_idx, 0, &blitterCtxPtr->Prg0RegPhy, &blitterCtxPtr->Prg0RegPtr);
        if (!NT_SUCCESS(status)) {
            return status;
        }
        status = map_reg(memResourcePtr, VPU_BLIT_PRG1_res_idx, 0, &blitterCtxPtr->Prg1RegPhy, &blitterCtxPtr->Prg1RegPtr);
        if (!NT_SUCCESS(status)) {
            return status;
        }
        status = map_reg(memResourcePtr, VPU_BLIT_DPR_CH0_res_idx, 0, &blitterCtxPtr->DprCh0RegPhy,
                         &blitterCtxPtr->DprCh0RegPtr);
        if (!NT_SUCCESS(status)) {
            return status;
        }
        status = map_reg(memResourcePtr, VPU_BLIT_DPR_CH1_res_idx, 0, &blitterCtxPtr->DprCh1RegPhy,
                         &blitterCtxPtr->DprCh1RegPtr);
        if (!NT_SUCCESS(status)) {
            return status;
        }
        status = map_reg(memResourcePtr, VPU_BLIT_LTS_res_idx, 0, &blitterCtxPtr->LtsRegPhy, &blitterCtxPtr->LtsRegPtr);
        if (!NT_SUCCESS(status)) {
            return status;
        }
        status = map_reg(memResourcePtr, VPU_BLIT_STEER_res_idx, 0, &blitterCtxPtr->IrqSteerRegPhy,
                         &blitterCtxPtr->IrqSteerRegPtr); // IrqSteerRegPtr
        if (!NT_SUCCESS(status)) {
            return status;
        }
    }
    memset((void *)deviceContextPtr->RPCBasePtr, 0, sizeof(MALONE_VPU_RPC));

    DBG_DEV_METHOD_END();
    return STATUS_SUCCESS;
}

_Use_decl_annotations_

/**
 * deinit_vpu_sys_memory()
 * @brief Free VPU reserved memory.
 * @param dev
 */
void deinit_vpu_sys_memory(MALONE_VPU_DEVICE_CONTEXT *dev)
{

    DBG_DEV_METHOD_BEG();


    /* Unmap VPU registers */
    DBG_DEV_PRINT_INFO("Unmapping VPU registers.");
    if (dev->RegistersPtr != NULL) {
        MmUnmapIoSpace(dev->RegistersPtr, sizeof(MALONE_VPU_REGISTERS));
        dev->RegistersPtr = NULL;
    }


    /* Unmap MU registers */
    DBG_DEV_PRINT_INFO("Unmapping MU registers.");
    if (dev->RegistersMUPtr != NULL) {
        MmUnmapIoSpace(dev->RegistersMUPtr, sizeof(MALONE_MU_REGISTERS));
        dev->RegistersMUPtr = NULL;
    }


    /* Unmap FW memory space */
    DBG_DEV_PRINT_INFO("Unmapping FW memory space.");
    if (dev->FWBasePtr != NULL) {
        MmUnmapIoSpace(dev->FWBasePtr, sizeof(MALONE_VPU_FW));
        dev->FWBasePtr = NULL;
    }


    /* Unmap RPC memory space */
    DBG_DEV_PRINT_INFO("Unmapping RPC memory space.");
    if (dev->RPCBasePtr != NULL) {
        MmUnmapIoSpace(dev->RPCBasePtr, sizeof(MALONE_VPU_RPC));
        dev->RPCBasePtr = NULL;
    }


    /* Unmap CSR cpu_wait register */
    DBG_DEV_PRINT_INFO("Unmapping CSR cpu_wait register.");
    if (dev->csr_cpuwait != (intptr_t)NULL) {
        MmUnmapIoSpace((void *)dev->csr_cpuwait, 4);
        dev->csr_cpuwait = (intptr_t)NULL;
    }


    /* Unmap CSR cpu_offset register */
    DBG_DEV_PRINT_INFO("Unmapping CSR cpu_offset register.");
    if (dev->csr_offset != (intptr_t)NULL) {
        MmUnmapIoSpace((void *)dev->csr_offset, 4);
        dev->csr_offset = (intptr_t)NULL;
    }

    DBG_DEV_METHOD_END();

}


/**
 * alloc_entry()
 * @brief Allocated memory for struct VpuMemory
 * @param alloc Ptr to Ptr of VpuMemory struct
 * @return NTSTATUS
 */
static NTSTATUS alloc_entry(VpuMemory **alloc)
{
    NTSTATUS status = STATUS_SUCCESS;
    /*DBG_DEV_METHOD_BEG(); */

    /* Allocate internal memory for mem     * This is used only localy in km driver to keep info about granted memory.
     * VpuMemory is an elist.alloc entry!
       ntry in memList.
     */
    (*alloc) = (VpuMemory *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(VpuMemory), MALONE_VPU_POOL_TAG);
    if ((*alloc) == NULL) { /* check alloc status */
        status = STATUS_INSUFFICIENT_RESOURCES;
        DBG_PRINT_ERROR_WITH_STATUS(status, "Out of memory");
        /*DBG_DEV_METHOD_END(); */
        return status;
    }

    memset(*alloc, 0, sizeof(VpuMemory));
    /*DBG_DEV_METHOD_END(); */
    return status;
}

/**
 * alloc_MDL()
 * @brief Allocates pages for MDL
 * @param alloc Ptr to Ptr of VpuMemory struct
 * @param size Size of memory to be allocated
 * @param cacheType
 * @return NTSTATUS
 */
static NTSTATUS alloc_MDL(VpuMemory **alloc, ULONG size, ULONG cacheType)
{

    NTSTATUS status = STATUS_SUCCESS;
    static const PHYSICAL_ADDRESS zero = { 0, 0 };          /* this is required by MmAllocatePagesForMdlEx */
    static const PHYSICAL_ADDRESS high = { 0xFFFFFFFF /*0xC0C00000*/, 0 }; /* this is required by MmAllocatePagesForMdlEx */

    memset(*alloc, 0, sizeof(VpuMemory));

    /*DBG_DEV_METHOD_BEG(); */
    /* Allocate */
    /* Memory Description List */
    (*alloc)->mdl = MmAllocatePagesForMdlEx(zero, high, zero, size, cacheType,
                                            MM_ALLOCATE_FULLY_REQUIRED | MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS);
    if ((*alloc)->mdl == NULL) {
        status = STATUS_INSUFFICIENT_RESOURCES;
        DBG_PRINT_ERROR_WITH_STATUS(status, "Out of memory");
    }

    /*DBG_DEV_METHOD_END(); */
    return status;
}

/**
 * map_MDL()
 * @brief map MDL pages
 * @param Ptr to Ptr of VpuMemory struct
 * @param cacheType
 * @return NTSTATUS
 */
static NTSTATUS map_MDL(VpuMemory **alloc, ULONG cacheType)
{
    NTSTATUS status = STATUS_SUCCESS;
    /*DBG_DEV_METHOD_BEG(); */

    try {
        (*alloc)->virtAddr = MmMapLockedPagesSpecifyCache((*alloc)->mdl, UserMode, cacheType, NULL, FALSE,
                                                          NormalPagePriority | MdlMappingNoExecute);
    }
    except(EXCEPTION_EXECUTE_HANDLER) {
        status = STATUS_INSUFFICIENT_RESOURCES;
        DBG_PRINT_ERROR_WITH_STATUS(status, "MmMapLockedPagesSpecifyCache(...) failed. (vpuRam = 0x%p)", (*alloc)->virtAddr);
        DBG_DEV_METHOD_END();
        return status;
    }
    /*DBG_DEV_METHOD_END(); */
    return status;
}

/**
 * free_vpu_buffer()
 * @brief Unmaps and frees vpu buffer
 * @param buffer Ptr to VpuMemory buffer
 */
void free_vpu_buffer(VpuMemory *buffer)
{
    if (buffer == NULL) {
        return;
    }
    /* free allocated resources */
    if (buffer->virtAddr != NULL) {
        MmUnmapLockedPages(buffer->virtAddr, buffer->mdl);
    }

    if (buffer->mdl != NULL) {
        MmFreePagesFromMdl(buffer->mdl);
        ExFreePool(buffer->mdl);
    }
    buffer->mdl = NULL;
    buffer->next = NULL;
    buffer->virtAddr = NULL;
    buffer->virtAddrAligned = NULL;
    buffer->physAddr = 0;
    buffer->physAddrAligned = 0;
    buffer->alignOffset = 0;
    buffer->file = NULL;
}

/**
 * alloc_vpu_buffer()
 * @brief Reserves VPU memory buffer
 * @param alloc Ptr to VpuMemory buffer
 * @param size Size of memory to be allocated
 * @param cacheType
 * @return NTSTATUS
 */
NTSTATUS alloc_vpu_buffer(VpuMemory *alloc, ULONG size, ULONG cacheType)
{
    NTSTATUS status = STATUS_SUCCESS;

    status = alloc_MDL(&alloc, size, cacheType);
    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "Out of memory");
        DBG_DEV_METHOD_END_WITH_STATUS(status);
        return status;
    }
    return status;
}

/**
 * map_vpu_buffer()
 * @brief maps MDL pages
 * @param buffer Ptr to VpuMemory buffer
 * @param cacheType
 * @param file WDFFILEOBJECT
 * @return NTSTATUS
 */
NTSTATUS map_vpu_buffer(VpuMemory *buffer, ULONG cacheType, WDFFILEOBJECT file)
{
    NTSTATUS status = STATUS_SUCCESS;
    status = map_MDL(&buffer, cacheType);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    PHYSICAL_ADDRESS physAddr = MmGetPhysicalAddress(buffer->virtAddr); /* get PHY memory */
    buffer->physAddr = physAddr.QuadPart;
    buffer->file = file;

    buffer->physAddrAligned = (uintptr_t)((physAddr.QuadPart + MEM_ALIGN - 1) & ~(MEM_ALIGN - 1));
    buffer->alignOffset = buffer->physAddrAligned - buffer->physAddr;
    buffer->virtAddrAligned = buffer->virtAddr + buffer->alignOffset;

    return status;
}

/**
 * unmap_vpu_buffer()
 * @brief Unmap MDL pages
 * @param buffer Ptr to VpuMemory buffer
 */
void unmap_vpu_buffer(VpuMemory *buffer)
{
    if (buffer == NULL) {
        return;
    }
    /* free allocated resources */
    if (buffer->mdl != NULL) {
        if (buffer->virtAddr != NULL) {
            MmUnmapLockedPages(buffer->virtAddr, buffer->mdl);
            buffer->virtAddr = NULL;
            buffer->virtAddrAligned = NULL;
        }
    }

    buffer->physAddr = 0;
    buffer->physAddrAligned = 0;
    buffer->file = NULL;
}

/**
 * get_frame_size_from_cfg()
 * @brief Calculate frame size based on values given from decoder
 * @param CtxMemory *ctxMem Pointer to CtxMemory struct
 * @param VpuConfig *config Pointer to config struct
 */
static void get_frame_size_from_cfg(CtxMemory *ctxMem, VpuConfig *config)
{
    u_int32 width = config->MaxWidth;
    u_int32 height = config->MaxHeigth;
    u_int32 chroma_height = 0;
    u_int32 uVertAlign = 512 - 1;
    u_int32 uAlign = 0x800 - 1;

    /*allign width */
    width = config->Allow10BitFormat ? (width + ((width + 3) >> 2)) : width;
    width = ((width + uVertAlign) & ~uVertAlign);

    /*allign heigth */
    height = ((height + uVertAlign) & ~uVertAlign);

    /* chroma is a half of size */
    chroma_height = height >> 1;

    ctxMem->fb_luma_size = width * height;
    ctxMem->fb_chroma_size = width * chroma_height;
    ctxMem->fb_height = height;
    ctxMem->fb_width = width;

    ctxMem->mbi_size = (ctxMem->fb_luma_size + ctxMem->fb_chroma_size) / 4;
    ctxMem->mbi_size = ((ctxMem->mbi_size + uAlign) & ~uAlign);

    DBG_DEV_PRINT_VERBOSE("Calculated frame buffer size from settings: (%dx%d) Aligned (%dx%d) luma size %d chroma size %d mbi size %d",
                          config->MaxWidth, config->MaxHeigth, width, height,
                          ctxMem->fb_luma_size, ctxMem->fb_chroma_size, ctxMem->mbi_size);

    return;
}

/**
 * fbl_connect_ctx()
 * @brief Due to independend frame_buffer_list implementation,
 *          it is necessary to provide some variables/values from
 *          context. This function provides pointers to the FBL.
 * @param ctx Ptr to context
 * @return NTSTATUS
 */
static NTSTATUS fbl_connect_ctx(vdec_ctx_t *ctx)
{
    if (ctx->CtxMem.fbl) {
        ctx->CtxMem.fbl->pending_vfb_req = &(ctx->pending_vfb_req);
        ctx->CtxMem.fbl->pending_ofb_req = &(ctx->pending_ofb_req);
        ctx->CtxMem.fbl->aborts = (uint32_t *) & (ctx->aborts);
        ctx->CtxMem.fbl->stream_id = &(ctx->stream_id);
        ctx->CtxMem.fbl->file = &(ctx->file);
        ctx->CtxMem.fbl->cond = &(ctx->cond);
        ctx->CtxMem.fbl->fb_luma_size = &(ctx->CtxMem.fb_luma_size);
        ctx->CtxMem.fbl->fb_chroma_size = &(ctx->CtxMem.fb_chroma_size);

        return STATUS_SUCCESS;
    } else {
        return -1;
    }
}

/**
 * fbl_destroy()
 * @brief Destroy context Frame Buffer List
 * @param fbl_t *list context handle
 */
void fbl_destroy(fbl_t *list)
{
    int i = 0;
    if (list) {
        if (list->vfb) {
            for (i = 0; i < list->vfb_capacity; i++) {
                vfb_t *ele = &list->vfb[i];
                VpuMemory *luma = &ele->luma;
                VpuMemory *chroma = &ele->chroma;
                if (luma->virtAddr) {
                    free_vpu_buffer(luma);
                }
                if (chroma->virtAddr) {
                    free_vpu_buffer(chroma);
                }
            }
            ExFreePoolWithTag(list->vfb, MALONE_VPU_POOL_TAG);
            list->vfb = NULL;
        }
        if (list->ofb) {
            for (i = 0; i < list->ofb_capacity; i++) {
                ofb_t *ele = &list->ofb[i];
                VpuMemory *mem = &ele->mem;
                if (mem->virtAddr) {
                    free_vpu_buffer(mem);
                }
            }
            ExFreePoolWithTag(list->ofb, MALONE_VPU_POOL_TAG);
            list->ofb = NULL;
        }
        ExFreePoolWithTag(list, MALONE_VPU_POOL_TAG);
        list = NULL;
    }
}

/**
 * vpu_free_ctx_memory()
 * @brief Releases all memory reserved by context
 * @param dev Pointer do device context
 */
_Use_decl_annotations_
void vpu_free_ctx_memory(MALONE_VPU_DEVICE_CONTEXT *dev)
{
    DBG_DEV_METHOD_BEG();
    ULONG ctx_num = dev->config.MaxContexts;
    ULONG i = 0;
    ULONG j = 0;

    /* release all contexts */
    if (dev->ctx != NULL) {
        for (i = 0; i < ctx_num; i++) {
            vdec_ctx_t *ctx = &dev->ctx[i];
            if (ctx) {
                /* Release stream buffer */
                free_vpu_buffer(&ctx->CtxMem.sb_mem);

                /* Release userdata buffer */
                free_vpu_buffer(&ctx->CtxMem.ud_mem);

                /* Release MBI buffers */
                for (j = 0; j < MAX_MBI_NUM; j++) {
                    free_vpu_buffer(&ctx->CtxMem.mbi_mem[j]);
                }

                /*Release DCP buffers */
                if (dev->config.EnableHEVC) {
                    for (j = 0; j < MAX_DCP_NUM; j++) {
                        free_vpu_buffer(&ctx->CtxMem.dcp_mem[j]);
                    }
                }

                /* Release frame buffers */
                fbl_destroy(ctx->CtxMem.fbl);
            }
        }
        ExFreePoolWithTag(dev->ctx, MALONE_VPU_POOL_TAG);
    }
    DBG_DEV_METHOD_END();
}


/**
 * vpu_alloc_ctx_memory()
 * @brief Reads configuration from a registry
 * @param dev Pointer to deviceContextPtr
 * @return NTSTATUS
 */
_Use_decl_annotations_
NTSTATUS vpu_alloc_ctx_memory(MALONE_VPU_DEVICE_CONTEXT *dev)
{
    NTSTATUS ntstatus = STATUS_SUCCESS;

    ULONG ctx_num = dev->config.MaxContexts;
    ULONG i = 0;
    ULONG j = 0;

    /* S - Allocate all intended contextes CTX */
    dev->ctx = (vdec_ctx_t *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(vdec_ctx_t) * ctx_num, MALONE_VPU_POOL_TAG);
    if (dev->ctx == NULL) {
        ntstatus = STATUS_INSUFFICIENT_RESOURCES;
        DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to allocated ctx");
        DBG_DEV_METHOD_END();
        return ntstatus;
    }
    memset(dev->ctx, 0, sizeof(vdec_ctx_t) * ctx_num); /* clean */

    /* Init all contexts */
    for (i = 0; i < ctx_num; i++) {
        vdec_ctx_t *ctx = &dev->ctx[i];

        /* Reserve all intended memory */
        ctx->stream_id = -1;
        /* Reserve stream buffer */
        ntstatus = alloc_vpu_buffer(&ctx->CtxMem.sb_mem, dev->config.StreamBuffSize + MEM_ALIGN, SB_CACHE_TYPE);
        if (ntstatus != STATUS_SUCCESS) {
            DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve sb memory");
            DBG_DEV_METHOD_END();
            return ntstatus;
        }
        ctx->CtxMem.sb_mem.next = (VpuMemory *)NULL;

        /* Reserve UD buffer */
        ntstatus = alloc_vpu_buffer(&ctx->CtxMem.ud_mem, UDATA_BUFFER_SIZE + MEM_ALIGN, SB_CACHE_TYPE);
        if (ntstatus != STATUS_SUCCESS) {
            DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve ud memory");
            DBG_DEV_METHOD_END();
            return ntstatus;
        }

        ctx->CtxMem.sb_mem.next = &ctx->CtxMem.ud_mem;

        get_frame_size_from_cfg(&ctx->CtxMem, &dev->config);

        /* Reserve mbi buffers */
        for (j = 0; j < MAX_MBI_NUM; j++) {
            ntstatus = alloc_vpu_buffer(&ctx->CtxMem.mbi_mem[j], ctx->CtxMem.mbi_size + MEM_ALIGN, SB_CACHE_TYPE);
            if (ntstatus != STATUS_SUCCESS) {
                DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve mbi memory");
                DBG_DEV_METHOD_END();
                return ntstatus;
            }
            if (j > 0) {
                ctx->CtxMem.mbi_mem[j - 1].next = &ctx->CtxMem.mbi_mem[j];
            } else {
                ctx->CtxMem.ud_mem.next = &ctx->CtxMem.mbi_mem[0];
            }
        }

        /* Reserve VPU and Out Frame Buffers */
        ctx->CtxMem.fbl = (fbl_t *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(fbl_t), MALONE_VPU_POOL_TAG);
        if (ctx->CtxMem.fbl == NULL) {
            ntstatus = STATUS_INSUFFICIENT_RESOURCES;
            DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to allocate ctx");
            DBG_DEV_METHOD_END();
            return ntstatus;
        } else {
            /* List is allocated, now allocate VPU Frame Buffers and Output Frame Buffers*/
            memset(ctx->CtxMem.fbl, 0, sizeof(fbl_t)); /* clean */

            /* Allocate VPU Frame Buffers*/
            ctx->CtxMem.fbl->vfb = (vfb_t *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(vfb_t) * dev->config.FrameBuffers,
                                                                  MALONE_VPU_POOL_TAG);
            if (ctx->CtxMem.fbl->vfb == NULL) {
                ntstatus = STATUS_INSUFFICIENT_RESOURCES;
                DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to allocated ctx");
                DBG_DEV_METHOD_END();
                return ntstatus;
            }
            memset(ctx->CtxMem.fbl->vfb, 0, sizeof(vfb_t) * dev->config.FrameBuffers); /* clean */
            ctx->CtxMem.fbl->vfb_capacity = dev->config.FrameBuffers;
            fbl_connect_ctx(ctx);

            for (j = 0; j < dev->config.FrameBuffers; j++) {
                vfb_t *ele = &ctx->CtxMem.fbl->vfb[j];
                VpuMemory *luma = &ele->luma;
                VpuMemory *chroma = &ele->chroma;

                ntstatus = alloc_vpu_buffer(luma, ctx->CtxMem.fb_luma_size + MEM_ALIGN, SB_CACHE_TYPE);
                if (ntstatus != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve luma memory");
                    DBG_DEV_METHOD_END();
                    return ntstatus;
                }
                if (j > 0) {
                    ctx->CtxMem.fbl->vfb[j - 1].chroma.next = &ctx->CtxMem.fbl->vfb[j].luma;
                } else {
                    ctx->CtxMem.mbi_mem[MAX_MBI_NUM - 1].next = &ctx->CtxMem.fbl->vfb[0].luma;
                }

                ntstatus = alloc_vpu_buffer(chroma, ctx->CtxMem.fb_chroma_size + MEM_ALIGN, SB_CACHE_TYPE);
                if (ntstatus != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve chroma memory");
                    DBG_DEV_METHOD_END();
                    return ntstatus;
                }
                if (j > 0) {
                    ctx->CtxMem.fbl->vfb[j].luma.next = &ctx->CtxMem.fbl->vfb[j].chroma;
                }
            }
            /* Allocate Output Frame Buffers*/
            ctx->CtxMem.fbl->ofb = (ofb_t *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(ofb_t) * dev->config.OutFrameBuffers,
                                                                  MALONE_VPU_POOL_TAG);
            if (ctx->CtxMem.fbl->ofb == NULL) {
                ntstatus = STATUS_INSUFFICIENT_RESOURCES;
                DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to allocated ctx");
                DBG_DEV_METHOD_END();
                return ntstatus;
            }
            memset(ctx->CtxMem.fbl->ofb, 0, sizeof(ofb_t) * dev->config.OutFrameBuffers); /* clean */
            ctx->CtxMem.fbl->ofb_capacity = dev->config.OutFrameBuffers;

            /* fbl_connect_ctx(ctx); */
            for (j = 0; j < dev->config.OutFrameBuffers; j++) {
                ofb_t *ele = &ctx->CtxMem.fbl->ofb[j];
                VpuMemory *mem = &ele->mem;


                ntstatus = alloc_vpu_buffer(mem, (dev->config.MaxWidth * dev->config.MaxHeigth * 4) + MEM_ALIGN, MmCached);
                if (ntstatus != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve output  memory");
                    DBG_DEV_METHOD_END();
                    return ntstatus;
                }

                if (j > 0) {
                    ctx->CtxMem.fbl->ofb[j - 1].mem.next = &ctx->CtxMem.fbl->ofb[j].mem;
                } else {
                    ctx->CtxMem.fbl->vfb[dev->config.FrameBuffers - 1].chroma.next = &ctx->CtxMem.fbl->ofb[0].mem;
                }
            }
        }

        /* Reserve DCP buffers */
        if (dev->config.EnableHEVC) {
            for (j = 0; j < MAX_DCP_NUM; j++) {
                ntstatus = alloc_vpu_buffer(&ctx->CtxMem.dcp_mem[j], DCP_SIZE + MEM_ALIGN, SB_CACHE_TYPE);
                if (ntstatus != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR_WITH_STATUS(ntstatus, "Unable to reserve dcp memory");
                    DBG_DEV_METHOD_END();
                    return ntstatus;
                }
                if (j > 0) {
                    ctx->CtxMem.dcp_mem[j - 1].next = &ctx->CtxMem.dcp_mem[j];
                } else { /* Make link from previous element */
                    ctx->CtxMem.fbl->ofb[dev->config.OutFrameBuffers - 1].mem.next = &ctx->CtxMem.dcp_mem[0];
                }
            }
        }
    }

    return ntstatus;
}


/**
 * vpu_unmap_ctx_memory()
 * @brief Frees all memory reserved by context
 * @param CtxMem Pointer to context memory structure
 */
_Use_decl_annotations_
void vpu_unmap_ctx_memory(CtxMemory *CtxMem)
{
    DBG_DEV_METHOD_BEG();
    ULONG j = 0;

    /* Free stream buffer */
    unmap_vpu_buffer(&CtxMem->sb_mem);

    /* Free userdata buffer */
    unmap_vpu_buffer(&CtxMem->ud_mem);

    /* Free MBI buffers */
    for (j = 0; j < MAX_MBI_NUM; j++) {
        unmap_vpu_buffer(&CtxMem->mbi_mem[j]);
    }


    /* Release frame buffers */
    fbl_t *list = CtxMem->fbl;
    for (j = 0; j < list->vfb_capacity; j++) {
        vfb_t *ele = &list->vfb[j];
        VpuMemory *luma = &ele->luma;
        VpuMemory *chroma = &ele->chroma;
        if (luma->virtAddr) {
            unmap_vpu_buffer(luma);
        }
        if (chroma->virtAddr) {
            unmap_vpu_buffer(chroma);
        }
    }

    for (j = 0; j < list->ofb_capacity; j++) {
        ofb_t *ele = &list->ofb[j];
        VpuMemory *mem = &ele->mem;
        if (mem->virtAddr) {
            unmap_vpu_buffer(mem);
        }
    }

    /*Free DCP buffers */
    for (j = 0; j < MAX_DCP_NUM; j++) {
        unmap_vpu_buffer(&CtxMem->dcp_mem[j]);
    }


    DBG_DEV_METHOD_END();
}
