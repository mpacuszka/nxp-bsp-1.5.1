/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
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

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmd7LAdapter.tmh"

#include "GcKmd7LAdapter.h"
#include "GcKmd7LDevice.h"
#include "GcKmd7LContext.h"
#include "GcKmdGlobal.h"
#include "GcKmdUtil.h"
#include "GcKmd7LUtil.h"

#include "d3d_km_interface.h"

#include "GcKmdUtil.h"
#include "TdrInfo.h"

#include "Gc7LCmdGdiOpData.h"
#include "Gc7LGdiOpParams.h"
#include "Gc7LTexDesc.h"


extern "C" {

#include "gc_hal_kernel_driver.h"

void ProcessResources(PCM_RESOURCE_LIST resources);

irqreturn_t isrRoutine(struct _KINTERRUPT* irq, void* ctxt);

extern PFN_DisplayStartController   g_pfnDisplayStartController;
extern PFN_DisplayStopController    g_pfnDisplayStopController;

VOID GcWddmUnloadDispDll();
}


void* GcKm7LAdapter::operator new(size_t size)
{
    return ExAllocatePoolWithTag(NonPagedPoolNx, size, 'DAL7'); // Tag for 7L Adapter
}

void GcKm7LAdapter::operator delete(void* ptr)
{
    ExFreePool(ptr);
}

NTSTATUS
GcKm7LAdapter::Start(
    IN_PDXGK_START_INFO     DxgkStartInfo,
    IN_PDXGKRNL_INTERFACE   DxgkInterface,
    OUT_PULONG              NumberOfVideoPresentSources,
    OUT_PULONG              NumberOfChildren)
{
    NTSTATUS status;

    do
    {
        status = GcKmAdapter::Start(DxgkStartInfo, DxgkInterface, NumberOfVideoPresentSources, NumberOfChildren);
        if (!NT_SUCCESS(status))
        {
            break;
        }

        // Enable if system supports >= 4GB physical addresses
        if (0 == wcscmp(m_DeviceId, GC_7000_LITE_MQ) &&
            m_DeviceInfo.HighestPhysicalAddress.HighPart != 0)
        {
            // Try to allocate GPU memory from below 4GB
            g_bLimitAllocBelow4gbPa = GcKmdGlobal::s_bLimitAllocBelow4gbPa = true;
        }

        // Leaving this outside of the if-check above to provide flexibility
        // of enabling this feature even if system doesn't have 4GB.
        DWORD FeatureEnabled = 0;
        if (ReadDwordRegistryValue(&m_DeviceInfo.DeviceRegistryPath, L"Enable4GBSupport", &FeatureEnabled))
        {
            g_bLimitAllocBelow4gbPa = GcKmdGlobal::s_bLimitAllocBelow4gbPa = FeatureEnabled == 1;
        }

        // Process resource assigned by PNP
        ProcessResources(m_DeviceInfo.TranslatedResourceList);
        if (!gcPlatform->interrupt.m_set)
        {
            GC_LOG_ERROR("Interrupt resource for GC 7000 Lite is not found.");

            status = STATUS_DEVICE_CONFIGURATION_ERROR;
            break;
        }

        // Start the GC 7000 Lite GPU HW
        gceSTATUS GcStatus = gckDRIVER_DeviceStart();
        if (GcStatus != gcvSTATUS_OK)
        {
            GC_LOG_ERROR("gckDRIVER_DeviceStart(...) failed with (GcStatus=%d).", GcStatus);

            status = STATUS_UNSUCCESSFUL;
            break;
        }

        // Initialize the command processor's fence value to 0
        *(volatile gctUINT64*)(galDevice->kernels[0]->command->fence->logical) = 0L;

        status = m_Mmu.Start();
        if (!NT_SUCCESS(status))
        {
            break;
        }

        status = m_3DNode.Start(DxgkInterface);
        if (!NT_SUCCESS(status))
        {
            break;
        }

        if (GcKmdGlobal::s_bGdiHwAcceleration || UseGdiShaderForPaging())
        {
            gcePOOL     pool = gcvPOOL_DEFAULT;
            gctSIZE_T   allocSize = 0;
            gctUINT32   allocFlag = 0;

            UINT            maxShaderSize = 0;
            GC_7L_GDI_OP*   pGdiOps = GetGdiOps();
            UINT            numGdiOps = GetNumGdiOps();

            //
            // Calculate shader code GPU memory usage
            //

            for (UINT i = 0; i < numGdiOps; i++)
            {
                if (pGdiOps[i].InstSize > maxShaderSize)
                {
                    maxShaderSize = pGdiOps[i].InstSize;
                }
            }

            maxShaderSize = (maxShaderSize + (0x400 - 1)) & (~(0x400 - 1));
            allocSize = maxShaderSize*numGdiOps;

            //
            // Add Gdi command Uniform Buffer heap GPU memory usage
            //

            m_GdiUboHeap.m_Size = GcKm7LContext::s_GdiUboHeapSize;

            allocSize += m_GdiUboHeap.m_Size;

            // Add Gdi command Texure Descriptor heap GPU memory usage

            m_GdiTexDescHeap.m_Size = s_GdiTexDescHeapSize;

            allocSize += m_GdiTexDescHeap.m_Size;

            if (UseGdiShaderForPaging())
            {
                // Add System context Uniform Buffer heap GPU memory usage

                m_SysUboHeap.m_Size = GcKm7LContext::s_GdiUboHeapSize/2;

                allocSize += m_SysUboHeap.m_Size;
            }

            //
            // Allocate reserved GPU memory
            //

            GcStatus = gckKERNEL_AllocateVideoMemory(
                        galDevice->kernels[gcvCORE_MAJOR],
                        1024,
                        gcvVIDMEM_TYPE_COMMAND,
                        0,
                        &allocSize,
                        &pool,
                        &m_GdiVidMem);
            if (GcStatus != gcvSTATUS_OK)
            {
                status = STATUS_UNSUCCESSFUL;
                break;
            }

            // Lock for GPU access
            GcStatus = gckVIDMEM_NODE_Lock(
                        galDevice->kernels[gcvCORE_MAJOR],
                        m_GdiVidMem,
                        &m_GdiVidMemGpuVa);
            if (GcStatus != gcvSTATUS_OK)
            {
                status = STATUS_UNSUCCESSFUL;
                break;
            }

            // Lock for kernel mode CPU access
            GcStatus = gckVIDMEM_NODE_LockCPU(
                        galDevice->kernels[gcvCORE_MAJOR],
                        m_GdiVidMem,
                        gcvFALSE,
                        gcvFALSE,
                        &m_pGdiVidMem);
            if (GcStatus != gcvSTATUS_OK)
            {
                status = STATUS_UNSUCCESSFUL;
                break;
            }

            for (UINT i = 0; i < numGdiOps; i++)
            {
                if (NULL == pGdiOps[i].pInst)
                {
                    continue;
                }

                memcpy(
                    ((BYTE*)m_pGdiVidMem) + maxShaderSize*i,
                    pGdiOps[i].pInst,
                    pGdiOps[i].InstSize);

                pGdiOps[i].InstGpuVa = m_GdiVidMemGpuVa + maxShaderSize*i;
            }

            m_GdiUboHeap.m_GpuVa = m_GdiVidMemGpuVa + maxShaderSize*numGdiOps;
            m_GdiUboHeap.m_pMem = ((BYTE*)m_pGdiVidMem) + maxShaderSize*numGdiOps;

            m_GdiTexDescHeap.m_GpuVa = m_GdiUboHeap.m_GpuVa + m_GdiUboHeap.m_Size;
            m_GdiTexDescHeap.m_pMem = m_GdiUboHeap.m_pMem + m_GdiUboHeap.m_Size;

            if (UseGdiShaderForPaging())
            {
                m_SysUboHeap.m_GpuVa = m_GdiTexDescHeap.m_GpuVa + m_GdiTexDescHeap.m_Size;
                m_SysUboHeap.m_pMem = m_GdiTexDescHeap.m_pMem + m_GdiTexDescHeap.m_Size;
            }

            RtlInitializeBitMap(&m_GdiTexDescBitmap, m_GdiTexDescBitmapBuf, s_GdiTexDescHeapSize/s_TexDescSize);
            RtlClearAllBits(&m_GdiTexDescBitmap);

            KeInitializeMutex(&m_GdiTexDescBitmapMutex, 0);
        }

        if (g_pfnDisplayStartController)
        {
            status = g_pfnDisplayStartController(
                        GcKmdGlobal::s_DriverMode,
                        &m_DxgkInterface,
                        m_Mmu.GetLocalVidMemSegmentBase(),
                        &m_pDisplay,
                        NumberOfVideoPresentSources,
                        NumberOfChildren);
        }
    } while (FALSE);

    if (!NT_SUCCESS(status))
    {
        Stop();

        return status;
    }

    RegisterMmu(&m_Mmu);

    RegisterNode(GC_7L_ENGINE_ORDINAL_3D, &m_3DNode);

    RegisterDisplay(m_pDisplay);

    return status;
}

HANDLE GcKm7LAdapter::GetGdiUniformBufferHeap(
    DXGK_CREATECONTEXTFLAGS ContextFlags,
    HANDLE                  hRtDevice,
    HANDLE                  hRtContext,
    GpuMemoryHeap*          pHeap,
    UINT*                   pMaxPendingDmaBuffers)
{
    DXGKARGCB_CREATECONTEXTALLOCATION   ArgCbCreateCtxAllocation = {};

    if (ContextFlags.GdiContext)
    {
        *pHeap = m_GdiUboHeap;
    }
    else if (ContextFlags.SystemContext)
    {
        *pHeap = m_SysUboHeap;
    }
    else
    {
        DXGK_ALLOCATIONINFO RtAllocInfo = {};
        GcKmGdiAllocation   GdiAllocation = {};

        ArgCbCreateCtxAllocation.hAdapter = m_DxgkInterface.DeviceHandle;
        ArgCbCreateCtxAllocation.hDevice = hRtDevice;
        ArgCbCreateCtxAllocation.hContext = hRtContext;

        ArgCbCreateCtxAllocation.hDriverAllocation = &GdiAllocation;

        ArgCbCreateCtxAllocation.Size = GcKm7LContext::s_GdiUboHeapSize;

        GdiAllocation.m_memorySegmentId = APERTURE_SEGMENT_ID;

        m_pMmu->SetAllocationSegmentInfo(&GdiAllocation, &RtAllocInfo);

        ArgCbCreateCtxAllocation.Alignment = RtAllocInfo.Alignment;
        ArgCbCreateCtxAllocation.SupportedSegmentSet = RtAllocInfo.SupportedWriteSegmentSet;
        ArgCbCreateCtxAllocation.EvictionSegmentSet = RtAllocInfo.EvictionSegmentSet;
        ArgCbCreateCtxAllocation.PreferredSegment = RtAllocInfo.PreferredSegment;

        ArgCbCreateCtxAllocation.Flags.CpuVisible = 1;

        ArgCbCreateCtxAllocation.ContextAllocationFlags.MapGpuVirtualAddress = 1;

        auto Status = m_DxgkInterface.DxgkCbCreateContextAllocation(&ArgCbCreateCtxAllocation);
        if (NT_SUCCESS(Status))
        {
            pHeap->m_Size  = GcKm7LContext::s_GdiUboHeapSize;
            pHeap->m_pMem  = GdiAllocation.m_pCpuAddress;
            pHeap->m_GpuVa = (UINT)GdiAllocation.m_GpuVa;
        }
    }

    *pMaxPendingDmaBuffers = m_DxgkStartInfo.RequiredDmaQueueEntry;

    return ArgCbCreateCtxAllocation.hAllocation;
}

NTSTATUS GcKm7LAdapter::Stop()
{
    if (g_pfnDisplayStopController)
    {
        g_pfnDisplayStopController(m_pDisplay);
        GcWddmUnloadDispDll();
    }
    m_pDisplay = nullptr;

    if (m_pGdiVidMem)
    {
        gckVIDMEM_NODE_UnlockCPU(
            galDevice->kernels[gcvCORE_MAJOR],
            m_GdiVidMem,
            0,
            gcvFALSE,
            gcvFALSE);
    }

    if (m_GdiVidMemGpuVa)
    {
        gckVIDMEM_NODE_Unlock(
            galDevice->kernels[gcvCORE_MAJOR],
            m_GdiVidMem,
            0,
            gcvNULL);
    }

    if (m_GdiVidMem)
    {
        gckVIDMEM_NODE_Dereference(
            galDevice->kernels[gcvCORE_MAJOR],
            m_GdiVidMem);
    }

    m_pGdiVidMem = gcvNULL;
    m_GdiVidMemGpuVa = 0;
    m_GdiVidMem = gcvNULL;

    m_3DNode.Stop();

    m_Mmu.Stop();

    // Stop the GC 7000 Lite GPU HW
    gckDRIVER_DeviceStop();

    return GcKmAdapter::Stop();
}

BOOLEAN GcKm7LAdapter::InterruptRoutine(
    IN_ULONG        MessageNumber)
{
    BOOLEAN bRet = false;

    if (m_pDisplay)
    {
        bRet |= m_pDisplay->InterruptRoutine(MessageNumber);
        if (bRet)
        {
            return bRet;
        }
    }

    //
    // Hard-code gcvCORE_MAJOR, GC 7000 Lite has only 1 engine/core
    //

    bRet |= isrRoutine(NULL, (void*)gcvCORE_MAJOR);

    return bRet;
}

extern BOOLEAN  g_bAllowForcedContextSwitch;

NTSTATUS
GcKm7LAdapter::QueryAdapterInfo(
    IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo)
{
    GC_LOG_TRACE(
        "QueryAdapterInfo was called. (Type=%d)",
        pQueryAdapterInfo->Type);

    switch (pQueryAdapterInfo->Type)
    {
    case DXGKQAITYPE_DRIVERCAPS:
    {
        NTSTATUS    Status;

        Status = GcKmAdapter::QueryAdapterInfo(pQueryAdapterInfo);
        if (STATUS_SUCCESS == Status)
        {
            DXGK_DRIVERCAPS*    pDriverCaps = (DXGK_DRIVERCAPS*)pQueryAdapterInfo->pOutputData;

            //
            // GDI acceleration level of 2 : HW command buffer, 1 : SW redirection surface, 0 : Off
            //

            DWORD   GdiAccLevel = 2;

            ReadDwordRegistryValue(&m_DeviceInfo.DeviceRegistryPath, L"GdiAccLevel", &GdiAccLevel);

            if (GcKmdGlobal::s_bGdiHwAcceleration && (GdiAccLevel > 1))
            {
                pDriverCaps->PresentationCaps.SupportKernelModeCommandBuffer = TRUE;
                pDriverCaps->PresentationCaps.NoCacheCoherentApertureMemory = TRUE;
                pDriverCaps->PresentationCaps.SupportSoftwareDeviceBitmaps = FALSE;

                pDriverCaps->PresentationCaps.NoSameBitmapOverlappedAlphaBlend = TRUE;
                pDriverCaps->PresentationCaps.NoSameBitmapOverlappedStretchBlt = TRUE;

                pDriverCaps->PresentationCaps.NoSameBitmapOverlappedBitBlt = TRUE;
            }

            if (0 == GdiAccLevel)
            {
                pDriverCaps->PresentationCaps.SupportKernelModeCommandBuffer = FALSE;
                pDriverCaps->PresentationCaps.NoCacheCoherentApertureMemory = FALSE;
                pDriverCaps->PresentationCaps.SupportSoftwareDeviceBitmaps = FALSE;

                g_bAllowForcedContextSwitch = TRUE;
            }
        }

        return Status;
    }
    break;

    default:
        return GcKmAdapter::QueryAdapterInfo(pQueryAdapterInfo);
    }
}

NTSTATUS
GcKm7LAdapter::ResetFromTimeout()
{
    for (UINT i = 0; i < m_NumNodes; i++)
    {
        if (nullptr == m_Nodes[i])
        {
            continue;
        }

        m_Nodes[i]->PrepareToReset();
    }

    gckOS_Broadcast(galDevice->os, galDevice->kernels[0]->hardware, gcvBROADCAST_GPU_COMMIT);

    //
    // Set restoreAddress to restart from the last Wait-Link loop
    //
    galDevice->kernels[0]->restoreAddress = galDevice->kernels[0]->command->waitPos.address;
    gckKERNEL_Recovery(galDevice->kernels[0]);

    galDevice->kernels[0]->restoreAddress = 0;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LAdapter::RestartFromTimeout()
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LAdapter::CollectDbgInfo(
    IN_CONST_PDXGKARG_COLLECTDBGINFO    pCollectDbgInfo)
{
    static const Gc7LRegisterRangeDescriptor regIndexes[] = {
        RegisterRangeStartEnd(0x0000, 0x003E),
        /*RegisterRangeStartEnd(0x0F00, 0x0F26),
        RegisterRangeStartEnd(0x0F28, 0x0F34),
        RegisterRangeStartEnd(0x0E00, 0x0E28),
        {0x0E30, 1},
        {0x0180, 1},
        RegisterRangeStartEnd(0x0190, 0x019F),
        {0x01A0, 1},
        {0x01A8, 1},
        {0x01B0, 1},
        {0x01C0, 1},
        {0x01D0, 1},
        {0x01E0, 1},
        RegisterRangeStartEnd(0x01F0, 0x01F4),
        RegisterRangeStartEnd(0x01F6, 0x01FF),
        {0x5180, 16},
        {0x5190, 16},
        {0x51A0, 16},
        {0x51B0, 16},
        {0x5E00, 32},
        {0x5E20, 32},
        {0x5E40, 32},
        {0x5E60, 32},
        {0x5EA0, 32},
        {0x5400, 16},
        {0x5410, 16},
        {0x5430, 16},
        {0x5440, 16},
        {0x5450, 16},
        {0x5460, 16},
        {0x5470, 16},
        {0x5480, 16},
        RegisterRangeStartEnd(0x0100, 0x0128),
        {0x0130, 16},
        {0x0140, 16},
        RegisterRangeStartEnd(0x0150, 0x0167)*/
    };

    gckHARDWARE_SetPowerState(galDevice->kernels[0]->hardware, gcvPOWER_ON_AUTO);

#if 0
    gckHARDWARE_DumpMMUException(galDevice->kernels[0]->hardware);

    gckHARDWARE_DumpGPUState(galDevice->kernels[0]->hardware);
#endif

    const size_t rangesAmount = ARRAYSIZE(regIndexes);

    volatile UINT* pRegBase = (volatile UINT*)galDevice->registerBases[0];

    BYTE* ptr = reinterpret_cast<BYTE*>(pCollectDbgInfo->pBuffer);
    Gc7LRegisterRangeBlob* blob = reinterpret_cast<Gc7LRegisterRangeBlob*>(ptr);

    blob->header.type = RegisterRanges;

    UINT size = sizeof(Gc7LBlobHeader);

    for (int i = 0; i < rangesAmount; ++i)
    {
        UINT numberOfRegisters = regIndexes[i].numberOfRegisters;

        blob->ranges[i].rangeDescriptor = regIndexes[i];

        size += SizeOfRange(numberOfRegisters);

        if (size > pCollectDbgInfo->BufferSize)
        {
            size -= SizeOfRange(numberOfRegisters);
            break;
        }

        UINT* pRegistersValue = blob->ranges[i].registers;

        for (UINT j = 0, index = regIndexes[i].startRegIndex; j < numberOfRegisters; ++j)
        {
            *pRegistersValue = pRegBase[index++];
            ++pRegistersValue;
        }

        ptr = reinterpret_cast<BYTE*>(pRegistersValue);
    }

    blob->header.size = size;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LAdapter::Escape(
    IN_CONST_PDXGKARG_ESCAPE    pEscape)
{
    if ((NULL == pEscape->pPrivateDriverData) ||
        ((FIELD_OFFSET(gcsHAL_INTERFACE, galContext) != pEscape->PrivateDriverDataSize) &&
         (sizeof(D3dKmEscape) != pEscape->PrivateDriverDataSize)))
    {
        return STATUS_INVALID_PARAMETER;
    }

    if (sizeof(D3dKmEscape) == pEscape->PrivateDriverDataSize)
    {
        D3dKmEscape*    pD3dKmEscape = (D3dKmEscape *)pEscape->pPrivateDriverData;

        if (pD3dKmEscape->Size != sizeof(D3dKmEscape))
        {
            return STATUS_INVALID_PARAMETER;
        }

        switch (pD3dKmEscape->Type)
        {
        case D3D_KM_ESCAPE_UPDATE_IMAGE_INFO:
        {
            DXGKARG_RELEASE_HANDLE  ReleaseHandle;
            DXGKARGCB_GETHANDLEDATA GetHandleData = { 0 };
            GcKmAllocation* pAllocation;

            GetHandleData.hObject = pD3dKmEscape->UpdateImageInfo.hAllocation;
            GetHandleData.Type = DXGK_HANDLE_ALLOCATION;

            pAllocation = (GcKmAllocation*)m_DxgkInterface.DxgkCbAcquireHandleData(&GetHandleData, &ReleaseHandle);
            if (pAllocation)
            {
                if (pD3dKmEscape->UpdateImageInfo.UpdateFlags.Pitch)
                {
                    pAllocation->m_hwPitch = pD3dKmEscape->UpdateImageInfo.Pitch;
                }

                if (pD3dKmEscape->UpdateImageInfo.UpdateFlags.TileMode)
                {
                    if (0 == pD3dKmEscape->UpdateImageInfo.TileMode)
                    {
                        pAllocation->m_hwTileMode = Linear;
                    }
                }

                if (pD3dKmEscape->UpdateImageInfo.UpdateFlags.LinearOffset)
                {
                    pAllocation->m_linearOffset = pD3dKmEscape->UpdateImageInfo.LinearOffset;
                }

                DXGKARGCB_RELEASEHANDLEDATA ReleaseHandleData = { 0 };

                ReleaseHandleData.ReleaseHandle = ReleaseHandle;
                ReleaseHandleData.Type = DXGK_HANDLE_ALLOCATION;

                m_DxgkInterface.DxgkCbReleaseHandleData(ReleaseHandleData);
            }
            else
            {
                return STATUS_INVALID_PARAMETER;
            }
        }
        break;

        case D3D_KM_ESCAPE_GET_TILE_MODE_CAP:
        {
            if (0 == wcscmp(m_DeviceId, GC_7000_LITE_MQ) ||
                0 == wcscmp(m_DeviceId, GC_7000_LITE_8X))
            {
                pD3dKmEscape->GetTileModeCap.TileModeCap = 1;
            }
            else
            {
                pD3dKmEscape->GetTileModeCap.TileModeCap = 0;
            }
        }
        break;

        case D3D_KM_ESCAPE_GET_ADAPTER_CAPS:
        {
            if (0 == wcscmp(m_DeviceId, GC_7000_LITE_MP) ||
                0 == wcscmp(m_DeviceId, GC_7000_LITE_MN))
            {
                pD3dKmEscape->GetAdapterCaps.AdapterCaps = 0;
                pD3dKmEscape->GetAdapterCaps.UseNV12AliasCopy = 1;
            }
            else
            {
                pD3dKmEscape->GetAdapterCaps.AdapterCaps = 0;
            }
        }
        break;

        case D3D_KM_ESCAPE_CHECK_DIRECT_FLIP:
        {
            DXGKARG_RELEASE_HANDLE  ReleaseHandle;
            DXGKARGCB_GETHANDLEDATA GetHandleData = { 0 };
            GcKmAllocation* pAllocation;

            pD3dKmEscape->CheckDirectFlip.bSupported = 0;

            GetHandleData.hObject = pD3dKmEscape->CheckDirectFlip.hAllocation;
            GetHandleData.Type = DXGK_HANDLE_ALLOCATION;

            pAllocation = (GcKmAllocation*)m_DxgkInterface.DxgkCbAcquireHandleData(&GetHandleData, &ReleaseHandle);
            if (pAllocation)
            {
                if ((DXGI_FORMAT_B8G8R8A8_UNORM == pAllocation->m_format) ||
                    (DXGI_FORMAT_B8G8R8X8_UNORM == pAllocation->m_format))
                {
                    pD3dKmEscape->CheckDirectFlip.bSupported = 1;
                }

                if (DXGI_FORMAT_R8G8B8A8_UNORM == pAllocation->m_format)
                {
                    if ((0 == wcscmp(m_DeviceId, GC_7000_LITE_MQ)) ||
                        (0 == wcscmp(m_DeviceId, GC_7000_LITE_MP)) ||
                        (0 == wcscmp(m_DeviceId, GC_7000_LITE_8X)))
                    {
                        pD3dKmEscape->CheckDirectFlip.bSupported = 1;
                    }
                }

                DXGKARGCB_RELEASEHANDLEDATA ReleaseHandleData = { 0 };

                ReleaseHandleData.ReleaseHandle = ReleaseHandle;
                ReleaseHandleData.Type = DXGK_HANDLE_ALLOCATION;

                m_DxgkInterface.DxgkCbReleaseHandleData(ReleaseHandleData);
            }
        }
        break;

        default:
            return STATUS_INVALID_PARAMETER;
            break;
        }

        return STATUS_SUCCESS;
    }

    gcsHAL_INTERFACE   *pInterface;

    pInterface = (gcsHAL_INTERFACE *)pEscape->pPrivateDriverData;

#if VK_FLIP
    if (gcvHAL_FLIP == pInterface->command)
    {
#if 1
        D3DGPU_PHYSICAL_ADDRESS RootPageTable;

        if (NULL == pEscape->hContext)
        {
            return STATUS_INVALID_PARAMETER;
        }

        GcKmContext    *pGcKmContext = GcKmContext::Cast(pEscape->hContext);

        pGcKmContext->GetRootPageTable(&RootPageTable);

        GcKm7LPageTableWalker   GpuPageTableWalker(m_Mmu.GetPhysicalAddress(&RootPageTable));
#else
        //
        // Use the global page table
        //

        GcKm7LPageTableWalker   GpuPageTableWalker(galDevice->kernels[0]->mmu->mtlbPhysical);
#endif
        //
        // Flip of 0 indicates flipping back to "boot frame buffer"
        //

        if (pInterface->u.Flip)
        {
            PFN_NUMBER  PageIndex;
            NTSTATUS    Status;

            Status = GpuPageTableWalker.GpuVaRangeToCpuPhysicalPages(
                                            pInterface->u.Flip,
                                            1,
                                            &PageIndex);
            if (!NT_SUCCESS(Status))
            {
                return Status;
            }

            pInterface->u.Flip = ((UINT32)PageIndex) << 12;
        }

#if 1
        //
        // Remove when displayable/primary surface is created physically contiguous
        //
        return STATUS_SUCCESS;
#endif
    }
#endif

    gceSTATUS   GcStatus;

    GcStatus = gckDEVICE_Dispatch(galDevice->device, (gcsHAL_INTERFACE_PTR)pEscape->pPrivateDriverData);

    return (gcvSTATUS_OK == GcStatus) ? STATUS_SUCCESS : STATUS_INVALID_PARAMETER;
}

NTSTATUS GcKm7LAdapter::GetNodeMetaData(
    UINT                            NodeOrdinal,
    OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata)
{
    RtlZeroMemory(pGetNodeMetadata, sizeof(*pGetNodeMetadata));

    if (GC_7L_ENGINE_ORDINAL_3D == NodeOrdinal)
    {
        pGetNodeMetadata->EngineType = DXGK_ENGINE_TYPE_3D;

        RtlStringCbPrintfW(pGetNodeMetadata->FriendlyName,
            sizeof(pGetNodeMetadata->FriendlyName),
            L"3DNode%02X",
            NodeOrdinal);

        pGetNodeMetadata->GpuMmuSupported = 1;
    }
    else
    {
        pGetNodeMetadata->EngineType = DXGK_ENGINE_TYPE_COPY;

        RtlStringCbPrintfW(pGetNodeMetadata->FriendlyName,
            sizeof(pGetNodeMetadata->FriendlyName),
            L"CopyNode%02X",
            NodeOrdinal);

        //
        // This emulated copy engine runs in physical mode and is only used
        // for local video memory segment's paging operation
        //
    }

    return STATUS_SUCCESS;
}

NTSTATUS GcKm7LAdapter::AllocateTextureDescriptor(
    GcKmAllocation* pAllocation)
{
    if (!GcKmdGlobal::s_bGdiHwAcceleration)
    {
        return STATUS_SUCCESS;
    }

    CMutexTracker   MutexTracker(&m_GdiTexDescBitmapMutex);

    ULONG TexDescIndex;

    TexDescIndex = RtlFindClearBitsAndSet(&m_GdiTexDescBitmap, 1, m_GdiLastTexDescIndex);
    if (-1 == TexDescIndex)
    {
        return STATUS_NO_MEMORY;
    }
    m_GdiLastTexDescIndex = TexDescIndex;

    ULONG*  pTexDesc = (ULONG*)(m_GdiTexDescHeap.m_pMem + TexDescIndex*s_TexDescSize);

    pAllocation->m_texDescGpuVa = m_GdiTexDescHeap.m_GpuVa + TexDescIndex*s_TexDescSize;

    UINT    TexW, TexH, AlignedW, AlignedH, Stride, AlignedSize;

    AlignedW = TexW = pAllocation->m_mip0Info.TexelWidth;
    AlignedH = TexH = pAllocation->m_mip0Info.TexelHeight;

    if (pAllocation->m_hwTileMode != Linear)
    {
        AlignedW = (AlignedW + s_TileSize - 1) & (~(s_TileSize - 1));
        AlignedH = (AlignedH + s_TileSize - 1) & (~(s_TileSize - 1));
    }

    Stride = AlignedW*sizeof(UINT);

    AlignedSize = Stride*AlignedH;
    AlignedSize = (AlignedSize + s_TexSizeAlignment - 1) & (~(s_TexSizeAlignment - 1));

    if (Linear == pAllocation->m_hwTileMode)
    {
        memcpy(pTexDesc, GC_7L_TEX_DESC_LINEAR, sizeof(GC_7L_TEX_DESC_LINEAR));
    }
    else
    {
        memcpy(pTexDesc, GC_7L_TEX_DESC_TILED, sizeof(GC_7L_TEX_DESC_TILED));
    }

    pTexDesc[GC_7L_TEX_DESC_TEX_GPUVA] = 0;
    pTexDesc[GC_7L_TEX_DESC_TEX_WIDTH_HEIGHT] = (TexH << 16) | TexW;
    pTexDesc[GC_7L_TEX_DESC_TEX_STRIDE] = Stride;
    pTexDesc[GC_7L_TEX_DESC_TEX_LOG_WIDTH_HEIGHT] = (__vk_UtilLog2inXdot8(TexH) << 16) | __vk_UtilLog2inXdot8(TexW);
    pTexDesc[GC_7L_TEX_DESC_TEX_SIZE] = AlignedSize;

    return STATUS_SUCCESS;
}

void GcKm7LAdapter::ReleaseTextureDescriptor(
    GcKmAllocation* pAllocation)
{
    if (0 == pAllocation->m_texDescGpuVa)
    {
        return;
    }

    CMutexTracker   MutexTracker(&m_GdiTexDescBitmapMutex);

    ULONG TexDescIndex;

    TexDescIndex = ((ULONG)(pAllocation->m_texDescGpuVa - m_GdiTexDescHeap.m_GpuVa))/s_TexDescSize;
    RtlClearBit(&m_GdiTexDescBitmap, TexDescIndex);
}

void GcKm7LAdapter::UpdateTextureDescriptor(
    GcKmAllocation*         pAllocation,
    D3DGPU_VIRTUAL_ADDRESS  TexGpuVa)
{
    ULONG TexDescIndex;

    TexDescIndex = ((ULONG)(pAllocation->m_texDescGpuVa - m_GdiTexDescHeap.m_GpuVa))/s_TexDescSize;

    ULONG* pTexDesc = (ULONG*)(m_GdiTexDescHeap.m_pMem + TexDescIndex*s_TexDescSize);

    pTexDesc[GC_7L_TEX_DESC_TEX_GPUVA] = (UINT)TexGpuVa;
}

NTSTATUS GcKm7LAdapter::IsGdiSurfaceSupported(
    INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData)
{
    switch (pGetStandardAllocationDriverData->StandardAllocationType)
    {
    case D3DKMDT_STANDARDALLOCATION_GDISURFACE:
    {
        D3DKMDT_GDISURFACEDATA* pGdiSurfInfo = pGetStandardAllocationDriverData->pCreateGdiSurfaceData;

        //
        // https://learn.microsoft.com/en-us/windows/win32/direct3d11/overviews-direct3d-11-devices-downlevel-intro
        // D3D11 FL 9.3 driver has Max Texture Dimension of 4096 limit
        //

        if (((pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE_CPUVISIBLE) ||
             (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE)) &&
            ((pGdiSurfInfo->Width > 4096) ||
             (pGdiSurfInfo->Height > 4096)))
        {
            return STATUS_NOT_SUPPORTED;
        }
    }
    break;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LMQAdapter::CreateDevice(
    INOUT_PDXGKARG_CREATEDEVICE pCreateDevice)
{
    GcKmDevice* pDevice;
    NTSTATUS    Status;

    pDevice = new (NonPagedPoolNx, 'VDCG') GcKm7LMQDevice();
    if (!pDevice)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKm7LDevice.");
        return STATUS_NO_MEMORY;
    }

    Status = pDevice->Initialize(this, pCreateDevice);
    if (NT_SUCCESS(Status))
    {
        pCreateDevice->hDevice = pDevice;
    }
    else
    {
        delete pDevice;
    }

    return Status;
}

GC_7L_GDI_OP*
GcKm7LMQAdapter::GetGdiOps()
{
    return MQ::GC_7L_GDI_OP_TABLE;
}

UINT
GcKm7LMQAdapter::GetNumGdiOps()
{
    return sizeof(MQ::GC_7L_GDI_OP_TABLE)/sizeof(MQ::GC_7L_GDI_OP_TABLE[0]);
}


NTSTATUS
GcKm7LMPAdapter::CreateDevice(
    INOUT_PDXGKARG_CREATEDEVICE pCreateDevice)
{
    GcKmDevice* pDevice;
    NTSTATUS    Status;

    pDevice = new (NonPagedPoolNx, 'VDCG') GcKm7LMPDevice();
    if (!pDevice)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKm7LDevice.");
        return STATUS_NO_MEMORY;
    }

    Status = pDevice->Initialize(this, pCreateDevice);
    if (NT_SUCCESS(Status))
    {
        pCreateDevice->hDevice = pDevice;
    }
    else
    {
        delete pDevice;
    }

    return Status;
}

GC_7L_GDI_OP*
GcKm7LMPAdapter::GetGdiOps()
{
    return MP::GC_7L_GDI_OP_TABLE;
}

UINT
GcKm7LMPAdapter::GetNumGdiOps()
{
    return sizeof(MP::GC_7L_GDI_OP_TABLE)/sizeof(MP::GC_7L_GDI_OP_TABLE[0]);
}


