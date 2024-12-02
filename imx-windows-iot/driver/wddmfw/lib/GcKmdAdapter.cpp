/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdAdapter.tmh"

#include "GcKmdAdapter.h"
#include "GcKmdDevice.h"
#include "GcKmdProcess.h"
#include "GcKmdAllocation.h"
#include "GcKmdResource.h"

#include "GcKmdGlobal.h"

#include "GcKmdUtil.h"

void* GcKmAdapter::operator new(size_t size)
{
    return ExAllocatePoolWithTag(NonPagedPoolNx, size, 'MKCG'); // Tag for GCKM
}

void GcKmAdapter::operator delete(void* ptr)
{
    ExFreePool(ptr);
}

GcKmAdapter::GcKmAdapter(
    IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
    WCHAR  *pwszDeviceId,
    BOOLEAN IsSoftwareDevice)
{
    m_pPhysicalDevice = PhysicalDeviceObject;

    wcsncpy(m_DeviceId, pwszDeviceId, MAX_DEVICE_ID_LENGTH - 1);
    m_IsSoftwareDevice = IsSoftwareDevice;

    m_NumNodes = 0;
    RtlZeroMemory(&m_Nodes, sizeof(m_Nodes));

    m_pMmu = NULL;
    m_pDisplay = NULL;
}

GcKmAdapter::~GcKmAdapter()
{
    // do nothing
}

NTSTATUS
GcKmAdapter::Start(
    IN_PDXGK_START_INFO     DxgkStartInfo,
    IN_PDXGKRNL_INTERFACE   DxgkInterface,
    OUT_PULONG              NumberOfVideoPresentSources,
    OUT_PULONG              NumberOfChildren)
{
    m_DxgkStartInfo = *DxgkStartInfo;
    m_DxgkInterface = *DxgkInterface;

    //
    // Render only device has no VidPn source and target
    // Subclass should overwrite these values if it is not render-only.
    //
    *NumberOfVideoPresentSources = 0;
    *NumberOfChildren = 0;

    if (GcKmdGlobal::s_DriverMode != DisplayOnly)
    {
        //
        // On WDDM 2.3 driver, app can use fullscreen exclusive mode
        // For newer driver, MaximizedWindowedFullscreen mode is default
        //

#ifdef DXGKDDI_INTERFACE_VERSION_WDDM2_6
        m_WddmVersion = DXGKDDI_WDDMv2_6;
#elif defined(DXGKDDI_INTERFACE_VERSION_WDDM2_5)
        m_WddmVersion = DXGKDDI_WDDMv2_5;
#else
        m_WddmVersion = DXGKDDI_WDDMv2_3;
#endif
    }
    else
    {
        m_WddmVersion = DXGKDDI_WDDMv2;
    }

    m_NumNodes = GetNumNodes();

    NTSTATUS status;

    status = m_DxgkInterface.DxgkCbGetDeviceInformation(
                                m_DxgkInterface.DeviceHandle,
                                &m_DeviceInfo);
    if (!NT_SUCCESS(status))
    {
        GC_LOG_ERROR(
            "DxgkCbGetDeviceInformation(...) failed. (status=%!STATUS!, m_DxgkInterface.DeviceHandle=0x%p)",
            status,
            m_DxgkInterface.DeviceHandle);

        return status;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmAdapter::Stop()
{
    if (m_hGdiRegKey)
    {
        ZwClose(m_hGdiRegKey);
        m_hGdiRegKey = NULL;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmAdapter::DispatchIoRequest(
    IN_ULONG                    VidPnSourceId,
    IN_PVIDEO_REQUEST_PACKET    VideoRequestPacket)
{
    return STATUS_NOT_SUPPORTED;
}

BOOLEAN GcKmAdapter::InterruptRoutine(
    IN_ULONG        MessageNumber)
{
    return false;
}

NTSTATUS
GcKmAdapter::CreateAllocation(
    INOUT_PDXGKARG_CREATEALLOCATION pCreateAllocation)
{
    //
    // Check input parameter from UMD
    //
    if ((pCreateAllocation->NumAllocations != 1) ||
        (pCreateAllocation->pAllocationInfo->PrivateDriverDataSize != sizeof(AllocationExchangeData)))
    {
        GC_LOG_ERROR( "Invalid parameter for CreateAllocation() from UMD");

        return STATUS_INVALID_PARAMETER;
    }

    NTSTATUS                Status = STATUS_SUCCESS;
    GcKmResource           *pGcKmResource = NULL;
    DXGK_ALLOCATIONINFO    *pRtAllocInfo = pCreateAllocation->pAllocationInfo;
    GcKmAllocation         *pGcKmAllocation = NULL;
    D3DGPU_VIRTUAL_ADDRESS  TexDescGpuVa = 0L;

    do
    {
        if (pCreateAllocation->Flags.Resource)
        {
            if (pCreateAllocation->hResource == NULL)
            {
                pGcKmResource = (GcKmResource *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(GcKmResource), GC_POOL_TAG_RESOURCE);
                if (!pGcKmResource)
                {
                    GC_LOG_LOW_MEMORY(
                        "Failed to allocate nonpaged pool for sizeof(GcKmResource) structure. (sizeof(GcKmResource)=%d)",
                        sizeof(GcKmResource));

                    Status = STATUS_NO_MEMORY;
                    break;
                }

                pGcKmResource->m_Dummy = 0;
            }
            else
            {
                pGcKmResource = (GcKmResource*)pCreateAllocation->hResource;
            }
        }

        AllocationExchangeData *pUmdAllocInfo = (AllocationExchangeData*)pRtAllocInfo->pPrivateDriverData;

        pGcKmAllocation = (GcKmAllocation *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(GcKmAllocation), GC_POOL_TAG_ALLOCATION);
        if (!pGcKmAllocation)
        {
            GC_LOG_ERROR(
                "Failed to allocated nonpaged pool for GcKmAllocation. (sizeof(GcKmAllocation)=%d)",
                sizeof(GcKmAllocation));

            Status = STATUS_NO_MEMORY;
            break;
        }

        *(AllocationExchangeData *)pGcKmAllocation = *pUmdAllocInfo;

        memset(&pGcKmAllocation->m_gpuPhysicalAddress, 0, sizeof(pGcKmAllocation->m_gpuPhysicalAddress));
        pGcKmAllocation->m_texDescGpuVa = 0;

        if (pGcKmAllocation->m_isShared || pGcKmAllocation->m_isGdiSurface)
        {
            Status = AllocateTextureDescriptor(pGcKmAllocation);
            if (!NT_SUCCESS(Status))
            {
                break;
            }
        }

        pRtAllocInfo->Alignment = 64;
        pRtAllocInfo->AllocationPriority = D3DDDI_ALLOCATIONPRIORITY_NORMAL;
        pRtAllocInfo->EvictionSegmentSet = 0; // don't use apperture for eviction

        pRtAllocInfo->FlagsWddm2.Value = 0;

        //
        // Set CpuVisible per UMD request
        //
        pRtAllocInfo->FlagsWddm2.CpuVisible = pGcKmAllocation->m_isCpuVisible;

        //
        // Use Cached for API read-only allocation 
        //
#if CACHE_COHERENT_PLATFORM
        if ((pGcKmAllocation->m_resourceDimension != 0) &&
            (pGcKmAllocation->m_mapFlags == D3D10_DDI_CPU_ACCESS_READ))
        {
            pRtAllocInfo->FlagsWddm2.Cached = true;
        }
#endif

        //
        // Primary's physical address is needed for setting display mode
        //
        if (pGcKmAllocation->m_isPrimary)
        {
            pRtAllocInfo->FlagsWddm2.AccessedPhysically = 1;
            pRtAllocInfo->FlagsWddm2.ExplicitResidencyNotification = 1;
        }

        pRtAllocInfo->HintedBank.Value = 0;
        pRtAllocInfo->MaximumRenamingListLength = 0;
        pRtAllocInfo->pAllocationUsageHint = NULL;
        pRtAllocInfo->PhysicalAdapterIndex = 0;
        pRtAllocInfo->PitchAlignedSize = 0;

        // zero-size allocations are not allowed
        NT_ASSERT(pGcKmAllocation->m_hwSizeBytes != 0);
        pRtAllocInfo->Size = pGcKmAllocation->m_hwSizeBytes;

        m_pMmu->SetAllocationSegmentInfo(pGcKmAllocation, pRtAllocInfo);

        //
        // Return driver allocation, resource objects to runtime
        //
        pRtAllocInfo->hAllocation = pGcKmAllocation;

        if (pCreateAllocation->Flags.Resource && pCreateAllocation->hResource == NULL)
        {
            pCreateAllocation->hResource = pGcKmResource;
        }
    } while (FALSE);

    if (Status == STATUS_SUCCESS)
    {
        GC_LOG_TRACE(
            "Created allocation. (Flags.CpuVisible=%d, Flags.Cacheable=%d, Size=%Id)",
            pRtAllocInfo->Flags.CpuVisible,
            pRtAllocInfo->Flags.Cached,
            pRtAllocInfo->Size);
    }
    else
    {
        if (pGcKmResource != NULL)
        {
            ExFreePoolWithTag(pGcKmResource, GC_POOL_TAG_RESOURCE);
        }

        if (pGcKmAllocation != NULL)
        {
            ReleaseTextureDescriptor(pGcKmAllocation);
            ExFreePoolWithTag(pGcKmAllocation, GC_POOL_TAG_ALLOCATION);
        }
    }

    return Status;
}

NTSTATUS
GcKmAdapter::DestroyAllocation(
    IN_CONST_PDXGKARG_DESTROYALLOCATION pDestroyAllocation)
{
    for (UINT i = 0; i < pDestroyAllocation->NumAllocations; i++)
    {
        GcKmAllocation *pGcKmAllocation = (GcKmAllocation *)pDestroyAllocation->pAllocationList[i];

        ReleaseTextureDescriptor(pGcKmAllocation);

        ExFreePoolWithTag(pGcKmAllocation, GC_POOL_TAG_ALLOCATION);
    }

    if (pDestroyAllocation->Flags.DestroyResource)
    {
        GcKmResource   *pGcKmResource = (GcKmResource*)pDestroyAllocation->hResource;

        ExFreePoolWithTag(pGcKmResource, GC_POOL_TAG_RESOURCE);
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmAdapter::GetStandardAllocationDriverData(
    INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    if ((NULL == pGetStandardAllocationDriverData->pAllocationPrivateDriverData) &&
        (NULL == pGetStandardAllocationDriverData->pResourcePrivateDriverData))
    {
        //
        // Check per process setting for redirection surface creation
        //

        if ((D3DKMDT_STANDARDALLOCATION_GDISURFACE == pGetStandardAllocationDriverData->StandardAllocationType) &&
            ((D3DKMDT_GDISURFACE_TEXTURE_CPUVISIBLE == pGetStandardAllocationDriverData->pCreateGdiSurfaceData->Type) ||
             (D3DKMDT_GDISURFACE_TEXTURE == pGetStandardAllocationDriverData->pCreateGdiSurfaceData->Type)))
        {
            UNICODE_STRING* pImagePath = nullptr;

            do
            {
                PEPROCESS   pCreatorProcess;
                HANDLE      ProcessId;

                ProcessId = PsGetCurrentProcessId();

                Status = PsLookupProcessByProcessId(ProcessId, &pCreatorProcess);
                if (NULL == pCreatorProcess)
                {
                    break;
                }

                SeLocateProcessImageName(pCreatorProcess, &pImagePath);
                if ((NULL == pImagePath) || (0 == pImagePath->Length))
                {
                    break;
                }

                if (NULL == m_hGdiRegKey)
                {
                    UNICODE_STRING      GdiRegPath;
                    OBJECT_ATTRIBUTES   Attr;

                    RtlInitUnicodeString(&GdiRegPath, L"\\REGISTRY\\MACHINE\\SOFTWARE\\VSI\\GPU\\GdiRedirSurf");

                    InitializeObjectAttributes(
                        &Attr,
                        &GdiRegPath,
                        OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE,
                        NULL,
                        NULL);
                
                    Status = ZwOpenKey(&m_hGdiRegKey, KEY_QUERY_VALUE, &Attr);
                }
                
                if (NULL == m_hGdiRegKey)
                {
                    break;
                }

                DWORD   bAllowRedirSurface = 1;

                ReadDwordRegistryValue2(&m_hGdiRegKey, pImagePath->Buffer, &bAllowRedirSurface);
                if (0 == bAllowRedirSurface)
                {
                    return STATUS_NOT_SUPPORTED;
                }

                WCHAR*  pImageName;

                pImageName = wcsrchr(pImagePath->Buffer, L'\\');
                pImageName++;

                ReadDwordRegistryValue2(&m_hGdiRegKey, pImageName, &bAllowRedirSurface);
                if (0 == bAllowRedirSurface)
                {
                    return STATUS_NOT_SUPPORTED;
                }
            } while (false);

            if (pImagePath)
            {
                ExFreePool(pImagePath);
            }
        }

        pGetStandardAllocationDriverData->AllocationPrivateDriverDataSize = sizeof(AllocationExchangeData);
        pGetStandardAllocationDriverData->ResourcePrivateDriverDataSize = 0;

        return STATUS_SUCCESS;
    }

    Status = IsGdiSurfaceSupported(pGetStandardAllocationDriverData);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    AllocationExchangeData *pAllocationInfo = (AllocationExchangeData *)pGetStandardAllocationDriverData->pAllocationPrivateDriverData;
    GcSurfaceRequirement    SurfaceRequirement;

    GetSurfaceRequirement(&SurfaceRequirement);

    switch (pGetStandardAllocationDriverData->StandardAllocationType)
    {
    case D3DKMDT_STANDARDALLOCATION_SHAREDPRIMARYSURFACE:
    {
        D3DKMDT_SHAREDPRIMARYSURFACEDATA   *pPrimaryInfo = pGetStandardAllocationDriverData->pCreateSharedPrimarySurfaceData;

#ifndef SCAFFOLD_OFF
        pAllocationInfo->m_isGdiPrimary = true;
#endif
        pAllocationInfo->m_isPrimary = true;

        FillAllocationInfo(pAllocationInfo, pPrimaryInfo, &SurfaceRequirement, true, pPrimaryInfo->VidPnSourceId, pPrimaryInfo->RefreshRate);
    }
    break;
    case D3DKMDT_STANDARDALLOCATION_SHADOWSURFACE:
    {
        D3DKMDT_SHADOWSURFACEDATA  *pShadowInfo = pGetStandardAllocationDriverData->pCreateShadowSurfaceData;
        
        pAllocationInfo->m_isPrimary = false;

        FillAllocationInfo(pAllocationInfo, pShadowInfo, &SurfaceRequirement);
        pShadowInfo->Pitch = pAllocationInfo->m_hwPitch;
    }
    break;
    case D3DKMDT_STANDARDALLOCATION_STAGINGSURFACE:
    {
        GC_STAGINGSURFACEDATA GcStagingSurfInfo(pGetStandardAllocationDriverData->pCreateStagingSurfaceData);

        pAllocationInfo->m_isPrimary = false;

        FillAllocationInfo(pAllocationInfo, &GcStagingSurfInfo, &SurfaceRequirement);
        pGetStandardAllocationDriverData->pCreateStagingSurfaceData->Pitch = pAllocationInfo->m_hwPitch;

        pAllocationInfo->m_isGdiSurface = true;
    }
    break;
    case D3DKMDT_STANDARDALLOCATION_GDISURFACE:
    {
        D3DKMDT_GDISURFACEDATA *pGdiSurfInfo = pGetStandardAllocationDriverData->pCreateGdiSurfaceData;

        NT_ASSERT((pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE_CPUVISIBLE) ||
                  (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE) ||
                  (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_STAGING) ||
                  (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_STAGING_CPUVISIBLE) ||
                  (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_LOOKUPTABLE));

#if 0
        if (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE)
        {
            return STATUS_NOT_SUPPORTED;
        }
#endif

        pAllocationInfo->m_isPrimary = false;

        FillAllocationInfo(
            pAllocationInfo,
            pGdiSurfInfo,
            &SurfaceRequirement,
            ((pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE_CPUVISIBLE) ||
             (pGdiSurfInfo->Type == D3DKMDT_GDISURFACE_TEXTURE)));

        pGdiSurfInfo->Pitch = pAllocationInfo->m_hwPitch;

        pAllocationInfo->m_isGdiSurface = true;
    }
    break;
    case D3DKMDT_STANDARDALLOCATION_VGPU:
    default:
        // NT_ASSERT(false);
        Status = STATUS_NOT_SUPPORTED;
        break;
    }

    return Status;
}

NTSTATUS
GcKmAdapter::DescribeAllocation(
    INOUT_PDXGKARG_DESCRIBEALLOCATION   pDescribeAllocation)
{
    GcKmAllocation *pAllocation = (GcKmAllocation *)pDescribeAllocation->hAllocation;

    if (pAllocation->m_isPrimary &&
        ((pAllocation->m_primaryDesc.Flags & DXGI_DDI_PRIMARY_NONPREROTATED) == 0))
    {
        pDescribeAllocation->Width = pAllocation->m_primaryDesc.ModeDesc.Width;
        pDescribeAllocation->Height = pAllocation->m_primaryDesc.ModeDesc.Height;
    }
    else
    {
        pDescribeAllocation->Width = pAllocation->m_mip0Info.TexelWidth;
        pDescribeAllocation->Height = pAllocation->m_mip0Info.TexelHeight;
    }


    pDescribeAllocation->Format = TranslateDxgiFormat(pAllocation->m_format);

    pDescribeAllocation->MultisampleMethod.NumSamples = pAllocation->m_sampleDesc.Count;
    pDescribeAllocation->MultisampleMethod.NumQualityLevels = pAllocation->m_sampleDesc.Quality;

    pDescribeAllocation->RefreshRate.Numerator = pAllocation->m_primaryDesc.ModeDesc.RefreshRate.Numerator;
    pDescribeAllocation->RefreshRate.Denominator = pAllocation->m_primaryDesc.ModeDesc.RefreshRate.Denominator;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmAdapter::QueryAdapterInfo(
    IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo)
{
    switch (pQueryAdapterInfo->Type)
    {
    case DXGKQAITYPE_DRIVERCAPS:
    {
        if (pQueryAdapterInfo->OutputDataSize < sizeof(DXGK_DRIVERCAPS))
        {
            GC_ASSERTION(
                "Output buffer is too small. (pQueryAdapterInfo->OutputDataSize=%d, sizeof(DXGK_DRIVERCAPS)=%d)",
                pQueryAdapterInfo->OutputDataSize,
                sizeof(DXGK_DRIVERCAPS));

            return STATUS_BUFFER_TOO_SMALL;
        }

        DXGK_DRIVERCAPS* pDriverCaps = (DXGK_DRIVERCAPS*)pQueryAdapterInfo->pOutputData;

        //
        // HighestAcceptableAddress
        //
        pDriverCaps->HighestAcceptableAddress.QuadPart = -1;

        //
        // GDI acceleration support
        //
        pDriverCaps->PresentationCaps.SupportKernelModeCommandBuffer = FALSE;
        pDriverCaps->PresentationCaps.SupportSoftwareDeviceBitmaps = TRUE;

        //
        // Cap used for DWM off case, screen to screen blt is slow
        //
        pDriverCaps->PresentationCaps.NoScreenToScreenBlt = TRUE;
        pDriverCaps->PresentationCaps.NoOverlapScreenBlt = TRUE;

        //
        // Allow 16Kx16K (2 << (11 + 3)) texture(redirection device bitmap)
        //
        pDriverCaps->PresentationCaps.MaxTextureWidthShift = 3;
        pDriverCaps->PresentationCaps.MaxTextureHeightShift = 3;

#ifdef DXGKDDI_INTERFACE_VERSION_WDDM2_4
        //
        // Support WDDM 2.4 contextless Present for flip
        //
        pDriverCaps->MiscCaps.SupportContextlessPresent = 1;
#endif

        //
        // Use SW flip queue for flip with interval of 1 or more
        //   - we must NOT generate a DMA buffer in DxgkDdiPresent. That is,
        //     we must set the DXGKARG_PRESENT.pDmaBuffer output parameter
        //     to NULL.
        //   - DxgkDdiSetVidPnSourceAddress will be called at DIRQL
        //
        pDriverCaps->FlipCaps.FlipOnVSyncMmIo = TRUE;

        //
        // Support Independent Flip (upgrade from Direct Flip)
        //

        pDriverCaps->FlipCaps.FlipIndependent = TRUE;

        pDriverCaps->SchedulingCaps.MultiEngineAware = 1;

        //
        // Set scheduling caps to indicate support for cancelling DMA buffer
        //
        pDriverCaps->SchedulingCaps.CancelCommandAware = 1;

        //
        // Set scheduling caps to indicate driver is preemption aware
        //
        pDriverCaps->SchedulingCaps.PreemptionAware = 1;

        //
        // MemoryManagementCaps
        //

        //
        // Declare Cross Adapter Shared Resource in FullDriver mode for WDDM 2.4 requirement
        // In RenderOnly mode (EMU or HW), driver chooses the Present path that avoids CASR
        //
        if (FullDriver == GcKmdGlobal::s_DriverMode)
        {
            pDriverCaps->MemoryManagementCaps.CrossAdapterResource = 1;
        }

#if GC_GPUVA_SUPPORT

        pDriverCaps->MemoryManagementCaps.VirtualAddressingSupported = 1;
        pDriverCaps->MemoryManagementCaps.GpuMmuSupported = 1;

        //
        // Windows Para-Virtualization feature requires GPU VA support
        //

        pDriverCaps->MemoryManagementCaps.ParavirtualizationSupported = 1;

        //
        // IoMmu is used to protected secure virtual machine (used by Windows Sandbox)
        //
        // TODO : Enable later
        //

        // pDriverCaps->MemoryManagementCaps.IoMmuSecureModeSupported = 1;

#endif

        pDriverCaps->MemoryManagementCaps.PagingNode = GetPagingNode();

        //
        // GpuEngineTopology
        //

        pDriverCaps->GpuEngineTopology.NbAsymetricProcessingNodes = m_NumNodes;

        //
        // Version of WDDM DDI supported
        //
        pDriverCaps->WDDMVersion = m_WddmVersion;

        //
        // PreemptionCaps
        //
        pDriverCaps->PreemptionCaps.GraphicsPreemptionGranularity = D3DKMDT_GRAPHICS_PREEMPTION_DMA_BUFFER_BOUNDARY;
        pDriverCaps->PreemptionCaps.ComputePreemptionGranularity = D3DKMDT_COMPUTE_PREEMPTION_DISPATCH_BOUNDARY;

        //
        // Must support DxgkDdiStopDeviceAndReleasePostDisplayOwnership
        //
        pDriverCaps->SupportNonVGA = TRUE;

        //
        // Must support updating path rotation in DxgkDdiUpdateActiveVidPnPresentPath
        //
        pDriverCaps->SupportSmoothRotation = TRUE;

        //
        // Support per engine reset
        //
        // TODO: Enable later
        //

        // pDriverCaps->SupportPerEngineTDR = 1;

        //
        // SupportDirectFlip
        //   - must not allow video memory to be flipped to an incompatible
        //     allocation in DxgkDdiSetVidPnSourceAddress
        //   - the user mode driver must validate Direct Flip resources before
        //     the DWM uses them
        //
        pDriverCaps->SupportDirectFlip = 1;

        //
        // Support SupportRuntimePowerManagement
        //
        pDriverCaps->SupportRuntimePowerManagement = FALSE;
    }
    break;

    case DXGKQAITYPE_PHYSICALADAPTERCAPS:
    {
        DXGK_PHYSICALADAPTERCAPS*   pPhysicalAdapterCaps = (DXGK_PHYSICALADAPTERCAPS*)pQueryAdapterInfo->pOutputData;

        pPhysicalAdapterCaps->NumExecutionNodes = (WORD)m_NumNodes;
        pPhysicalAdapterCaps->PagingNodeIndex = (WORD)GetPagingNode();
        pPhysicalAdapterCaps->DxgkPhysicalAdapterHandle = m_DxgkInterface.DeviceHandle;

#if GC_GPUVA_SUPPORT
        pPhysicalAdapterCaps->Flags.GpuMmuSupported = 1;
#endif

        pPhysicalAdapterCaps->Flags.MovePagingSupported = 1;
    }
    break;

    case DXGKQAITYPE_QUERYSEGMENT4:
    case DXGKQAITYPE_GPUMMUCAPS:
    case DXGKQAITYPE_PAGETABLELEVELDESC:
    {
        return m_pMmu->QueryAdapterInfo(pQueryAdapterInfo);
    }
    break;

#ifdef DXGKDDI_INTERFACE_VERSION_WDDM2_4
    case DXGKQAITYPE_FRAMEBUFFERSAVESIZE:
    {
        // Required for IoMmuSecureModeSupported
    }
    break;

    case DXGKQAITYPE_HARDWARERESERVEDRANGES:
    {
        // Required for IoMmuSecureModeSupported
    }
    break;
#endif

    case DXGKQAITYPE_DISPLAY_DRIVERCAPS_EXTENSION:
    case DXGKQAITYPE_DISPLAYID_DESCRIPTOR:
    {
        if (NULL == m_pDisplay)
        {
            return STATUS_NOT_SUPPORTED;
        }

        return m_pDisplay->QueryAdapterInfo(pQueryAdapterInfo);
    }
    break;

    case DXGKQAITYPE_UMDRIVERPRIVATE:
    {
        if (pQueryAdapterInfo->OutputDataSize < sizeof(AdapterExchangeData))
        {
            GC_LOG_ERROR(
                "Output buffer is too small. (pQueryAdapterInfo->OutputDataSize=%d, sizeof(ROSADAPTERINFO)=%d)",
                pQueryAdapterInfo->OutputDataSize,
                sizeof(AdapterExchangeData));

            return STATUS_BUFFER_TOO_SMALL;
        }
        
        AdapterExchangeData    *pGcAdapterInfo = (AdapterExchangeData *)pQueryAdapterInfo->pOutputData;
        
        pGcAdapterInfo->m_version = GC_DRIVER_VERSION;
        pGcAdapterInfo->m_wddmVersion = m_WddmVersion;

        pGcAdapterInfo->m_isSoftwareDevice = m_IsSoftwareDevice;

        RtlCopyMemory(pGcAdapterInfo->m_deviceId, m_DeviceId, sizeof(m_DeviceId));
    }
    break;
    
    default:
        GC_LOG_WARNING(
            "Unsupported query type. (pQueryAdapterInfo->Type=%d, pQueryAdapterInfo=0x%p)",
            pQueryAdapterInfo->Type,
            pQueryAdapterInfo);

        return STATUS_NOT_SUPPORTED;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmAdapter::CreateDevice(
    INOUT_PDXGKARG_CREATEDEVICE pCreateDevice)
{
    GcKmDevice* pDevice;
    NTSTATUS    Status;

    pDevice = new (NonPagedPoolNx, 'VDCG') GcKmDevice();
    if (!pDevice)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKmDevice.");
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

NTSTATUS
GcKmAdapter::ControlInterrupt(
    DXGK_INTERRUPT_TYPE InterruptType,
    BOOLEAN             EnableInterrupt)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    switch (InterruptType)
    {
    case DXGK_INTERRUPT_CRTC_VSYNC:
    case DXGK_INTERRUPT_DISPLAYONLY_VSYNC:
    case DXGK_INTERRUPT_CRTC_VSYNC_WITH_MULTIPLANE_OVERLAY:
    case DXGK_INTERRUPT_CRTC_VSYNC_WITH_MULTIPLANE_OVERLAY2:
        if (m_pDisplay)
        {
            Status = m_pDisplay->ControlInterrupt(InterruptType, EnableInterrupt);
        }
        break;

    default:
        break;
    }

    return Status;
}

NTSTATUS
GcKmAdapter::SubmitCommandVirtual(
    IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL  pSubmitCommandVirtual)
{
    return m_Nodes[pSubmitCommandVirtual->NodeOrdinal]->SubmitCommandVirtual(pSubmitCommandVirtual);
}

NTSTATUS
GcKmAdapter::PreemptCommand(
    IN_CONST_PDXGKARG_PREEMPTCOMMAND    pPreemptCommand)
{
    return m_Nodes[pPreemptCommand->NodeOrdinal]->Preempt(pPreemptCommand);
}

void GcKmAdapter::DpcRoutine(void)
{
    // dp nothing other than calling back into dxgk

    m_DxgkInterface.DxgkCbNotifyDpc(m_DxgkInterface.DeviceHandle);
}

NTSTATUS
GcKmAdapter::QueryInterface(
    IN_PQUERY_INTERFACE QueryInterface)
{
    if (NULL == m_pDisplay)
    {
        return STATUS_NOT_SUPPORTED;
    }

    return m_pDisplay->QueryInterface(QueryInterface);
}

NTSTATUS
GcKmAdapter::QueryChildRelations(
    INOUT_PDXGK_CHILD_DESCRIPTOR    ChildRelations,
    IN_ULONG                        ChildRelationsSize)
{
    if (NULL == m_pDisplay)
    {
        return STATUS_NOT_SUPPORTED;
    }

    return m_pDisplay->QueryChildRelations(ChildRelations, ChildRelationsSize);
}

void
GcKmAdapter::ResetDevice(void)
{
    // Do nothing
    GC_ASSERTION("Not implemented");
}

NTSTATUS
GcKmAdapter::DestroyProcess(
    IN HANDLE hKmProcess)
{
    GcKmProcess    *pGcKmProcess = static_cast<GcKmProcess*>(hKmProcess);

    delete pGcKmProcess;

    return STATUS_SUCCESS;
}

