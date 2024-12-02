/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "GcKmd.h"
#include "GcKmdAllocation.h"

#include "AdapterExchangeData.h"

#include "gcdispif.h"

#pragma warning(disable:4201)   // nameless struct/union

class GcKmContext;

class GcKmMmu
{
public:

    virtual NTSTATUS
    QueryAdapterInfo(
        CONST DXGKARG_QUERYADAPTERINFO *pQueryAdapterInfo) = NULL;

    virtual NTSTATUS
    CreateProcess(
        DXGKARG_CREATEPROCESS* pCreateProcess) = NULL;

    virtual void
    SetAllocationSegmentInfo(
        GcKmAllocation         *pGcKmAllocation,
        DXGK_ALLOCATIONINFO    *pRtAllocInfo) = NULL;

    virtual SIZE_T
    GetRootPageTableSize(
        INOUT_PDXGKARG_GETROOTPAGETABLESIZE pArgs) = NULL;

    virtual NTSTATUS
    BuildPagingBuffer(
        DXGKARG_BUILDPAGINGBUFFER  *pArgs) = NULL;

    virtual ULONGLONG
    GetPhysicalAddress(
        const D3DGPU_PHYSICAL_ADDRESS  *pGpuPhysicalAddress) = NULL;

    virtual void
    UpdateHwPageTable(
        GcKmContext    *pContext) = NULL;

    virtual NTSTATUS
    SwitchGpuVaSpace(
        const D3DGPU_PHYSICAL_ADDRESS  *pRootPageTable) = NULL;

    virtual PBYTE
    GetLocalVidMemCpuVA(
        ULONGLONG   PageTablePhysicalAddress) = NULL;
};

typedef struct _GcKmNodeErrorCondition
{
    union
    {
        struct
        {
            UINT    m_UnSupportedPagingOp           : 1;
            UINT    m_NotifyDmaBufCompletion        : 1;
            UINT    m_NotifyPreemptionCompletion    : 1;
            UINT    m_NotifyDmaBufFault             : 1;
            UINT    m_PagingFailure                 : 1;
        };

        UINT        m_Value;
    };
} GcKmNodeErrorCondition;

struct GcDmaBufSubmission;
class GcKmContext;

class GcKmEngine
{
public:

    virtual NTSTATUS Start() = NULL;

    virtual void Stop() = NULL;

    virtual NTSTATUS SubmitCommandBuffer(
        D3DGPU_VIRTUAL_ADDRESS  DmaBufGpuVa,
        UINT                    DmaBufSize,
        GcKmContext            *pContext,
        ULONGLONG               RootPhysicalAddress,
        BOOLEAN                 bForceContextSwitch) = NULL;

    virtual NTSTATUS CpuTransfer(
        ULONGLONG               RootPhysicalAddress,
        DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL* pTransferVirtual) = NULL;
};

class GcKmNode
{
public:

    GcKmNode()
    {
        m_ErrorHit.m_Value = 0;
    }

    static GcKmNode* Cast(void *ptr)
    {
        GcKmNode   *pGcKmNode = static_cast<GcKmNode *>(ptr);

        return pGcKmNode;
    }

    //
    // If Start() succeeds, then Stop() must be called to clean up
    //
    // For example, if a subclass first calls Start() and then does 
    // controller-specific stuff that fails, it needs to call Stop()
    // before returning failure.
    //

    virtual NTSTATUS Start(
        IN_PDXGKRNL_INTERFACE   DxgkInterface) = NULL;

    virtual NTSTATUS Stop() = NULL;

    virtual NTSTATUS SubmitCommandVirtual(
        IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL pSubmitCommand) = NULL;

    virtual NTSTATUS Preempt(
        IN_CONST_PDXGKARG_PREEMPTCOMMAND pPreemptCommand) = NULL;

    virtual void PrepareToReset() = NULL;

protected:

    GcKmNodeErrorCondition  m_ErrorHit;
};

class GcKmAdapter
{
public:

    static NTSTATUS AddAdapter(
        IN_CONST_PDEVICE_OBJECT     PhysicalDeviceObject,
        OUT_PPVOID                  MiniportDeviceContext);

    NTSTATUS DispatchIoRequest(
        IN_ULONG                    VidPnSourceId,
        IN_PVIDEO_REQUEST_PACKET    VideoRequestPacket);

    virtual BOOLEAN InterruptRoutine(
        IN_ULONG        MessageNumber);

    NTSTATUS CreateAllocation(
        INOUT_PDXGKARG_CREATEALLOCATION pCreateAllocation);

    NTSTATUS DestroyAllocation(
        IN_CONST_PDXGKARG_DESTROYALLOCATION pDestroyAllocation);

    virtual NTSTATUS QueryAdapterInfo(
        IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo);

    virtual NTSTATUS CreateDevice(
        INOUT_PDXGKARG_CREATEDEVICE pCreateDevice);

    NTSTATUS GetStandardAllocationDriverData(
        INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData);

    NTSTATUS DescribeAllocation(
        INOUT_PDXGKARG_DESCRIBEALLOCATION   pDescribeAllocation);

    virtual VOID GetSurfaceRequirement(
        GcSurfaceRequirement*   pSurfaceRequirement)
    {
        pSurfaceRequirement->AlignmentWidth = 1;
        pSurfaceRequirement->AlignmentHeight = 1;
    }

    virtual NTSTATUS IsGdiSurfaceSupported(
        INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData)
    {
        return STATUS_SUCCESS;
    }

    virtual NTSTATUS GetNodeMetaData(
        UINT                            NodeOrdinal,
        OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata) = NULL;

    NTSTATUS SubmitCommandVirtual(
        IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL  pSubmitCommandVirtual);

    NTSTATUS PreemptCommand(
        IN_CONST_PDXGKARG_PREEMPTCOMMAND    pPreemptCommand);

    virtual NTSTATUS CollectDbgInfo(
        IN_CONST_PDXGKARG_COLLECTDBGINFO    pCollectDbgInfo) = NULL;

    virtual NTSTATUS ResetFromTimeout() = NULL;

    virtual NTSTATUS RestartFromTimeout() = NULL;

    SIZE_T GetRootPageTableSize(
        INOUT_PDXGKARG_GETROOTPAGETABLESIZE pArgs)
    {
        return m_pMmu->GetRootPageTableSize(pArgs);
    }

    NTSTATUS CreateProcess(
        IN DXGKARG_CREATEPROCESS* pArgs)
    {
        return m_pMmu->CreateProcess(pArgs);
    }

    NTSTATUS DestroyProcess(
        IN HANDLE KmdProcessHandle);

    NTSTATUS BuildPagingBuffer(
        IN_PDXGKARG_BUILDPAGINGBUFFER   pArgs)
    {
        return m_pMmu->BuildPagingBuffer(pArgs);
    }

    virtual NTSTATUS Escape(
        IN_CONST_PDXGKARG_ESCAPE    pEscape) = NULL;

    virtual NTSTATUS QueryInterface(
        IN_PQUERY_INTERFACE QueryInterface);

    NTSTATUS QueryChildRelations(
        INOUT_PDXGK_CHILD_DESCRIPTOR    ChildRelations,
        IN_ULONG                        ChildRelationsSize);

    NTSTATUS QueryChildStatus(
        IN_PDXGK_CHILD_STATUS   ChildStatus,
        IN_BOOLEAN              NonDestructiveOnly)
    {
        return m_pDisplay->QueryChildStatus(ChildStatus, NonDestructiveOnly);
    }

    NTSTATUS QueryDeviceDescriptor(
        IN_ULONG                        ChildUid,
        INOUT_PDXGK_DEVICE_DESCRIPTOR   pDeviceDescriptor)
    {
        return m_pDisplay->QueryDeviceDescriptor(ChildUid, pDeviceDescriptor);
    }

    virtual NTSTATUS SetPowerState(
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType) = 0;

    void ResetDevice(void);

    //
    // Display-only DDIs
    //

public: // NONPAGED

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_SETVIDPNSOURCEADDRESS)
    _IRQL_requires_min_(PASSIVE_LEVEL)
    _IRQL_requires_max_(PROFILE_LEVEL - 1)
    NTSTATUS SetVidPnSourceAddress(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress)
    {
        return m_pDisplay->SetVidPnSourceAddress(pSetVidPnSourceAddress);
    }

public: // PAGED

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_SETPALETTE)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS SetPalette (
        IN_CONST_PDXGKARG_SETPALETTE    pSetPalette)
    {
        return STATUS_NOT_IMPLEMENTED;
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_SETPOINTERPOSITION)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS SetPointerPosition(
        IN_CONST_PDXGKARG_SETPOINTERPOSITION    pSetPointerPosition)
    {
        return m_pDisplay->SetPointerPosition(pSetPointerPosition);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_SETPOINTERSHAPE)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS SetPointerShape(
        IN_CONST_PDXGKARG_SETPOINTERSHAPE   pSetPointerShape)
    {
        return m_pDisplay->SetPointerShape(pSetPointerShape);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_ISSUPPORTEDVIDPN)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS IsSupportedVidPn(
        INOUT_PDXGKARG_ISSUPPORTEDVIDPN pIsSupportedVidPn)
    {
        return m_pDisplay->IsSupportedVidPn(pIsSupportedVidPn);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_RECOMMENDFUNCTIONALVIDPN)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS RecommendFunctionalVidPn (
        IN_CONST_PDXGKARG_RECOMMENDFUNCTIONALVIDPN_CONST    pRecommendFunctionalVidPn)
    {
        return m_pDisplay->RecommendFunctionalVidPn(pRecommendFunctionalVidPn);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_ENUMVIDPNCOFUNCMODALITY)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS EnumVidPnCofuncModality (
        IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality)
    {
        return m_pDisplay->EnumVidPnCofuncModality(pEnumCofuncModality);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_SETVIDPNSOURCEVISIBILITY)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS SetVidPnSourceVisibility(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEVISIBILITY  pSetVidPnSourceVisibility)
    {
        return m_pDisplay->SetVidPnSourceVisibility(pSetVidPnSourceVisibility);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_COMMITVIDPN)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS CommitVidPn(
        IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn)
    {
        return m_pDisplay->CommitVidPn(pCommitVidPn);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_UPDATEACTIVEVIDPNPRESENTPATH)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS UpdateActiveVidPnPresentPath(
        IN_CONST_PDXGKARG_UPDATEACTIVEVIDPNPRESENTPATH_CONST    pUpdateActiveVidPnPresentPath)
    {
        return m_pDisplay->UpdateActiveVidPnPresentPath(pUpdateActiveVidPnPresentPath);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_RECOMMENDMONITORMODES)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS RecommendMonitorModes(
        IN_CONST_PDXGKARG_RECOMMENDMONITORMODES_CONST   pRecommendMonitorMode)
    {
        return m_pDisplay->RecommendMonitorModes(pRecommendMonitorMode);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_GETSCANLINE)
    _IRQL_requires_(PASSIVE_LEVEL)
        NTSTATUS GetScanLine(
            INOUT_PDXGKARG_GETSCANLINE  pGetScanLine)
    {
        return m_pDisplay->GetScanLine(pGetScanLine);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_CONTROLINTERRUPT)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS ControlInterrupt(
        IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
        IN_BOOLEAN                      EnableInterrupt);

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_QUERYVIDPNHWCAPABILITY)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS QueryVidPnHWCapability(
        INOUT_PDXGKARG_QUERYVIDPNHWCAPABILITY   pVidPnHWCaps)
    {
        return m_pDisplay->QueryVidPnHWCapability(pVidPnHWCaps);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_PRESENTDISPLAYONLY)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS PresentDisplayOnly(
        IN_CONST_PDXGKARG_PRESENT_DISPLAYONLY  pPresentDisplayOnly)
    {
        return m_pDisplay->PresentDisplayOnly(pPresentDisplayOnly);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_STOPDEVICEANDRELEASEPOSTDISPLAYOWNERSHIP)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS StopDeviceAndReleasePostDisplayOwnership(
        _In_ D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId,
        _Out_ PDXGK_DISPLAY_INFORMATION     pDisplayInfo)
    {
        m_pDisplay->StopDeviceAndReleasePostDisplayOwnership(TargetId, pDisplayInfo);

        return Stop();
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3)
    _IRQL_requires_max_(PROFILE_LEVEL - 1)
    NTSTATUS SetVidPnSourceAddressWithMultiPlaneOverlay3(
        IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetVidPnSourceAddressWithMpo3)
    {
        return m_pDisplay->SetVidPnSourceAddressWithMultiPlaneOverlay3(pSetVidPnSourceAddressWithMpo3);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_UPDATEMONITORLINKINFO)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS UpdateMonitorLinkInfo(
        INOUT_PDXGKARG_UPDATEMONITORLINKINFO    pUpdateMonitorLinkInfoArg)
    {
        return m_pDisplay->UpdateMonitorLinkInfo(pUpdateMonitorLinkInfoArg);
    }

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_GETCHILDCONTAINERID)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS GetChildContainerId(
        _In_ ULONG  ChildUid,
        _Inout_ PDXGK_CHILD_CONTAINER_ID    pChildContainIdArg)
    {
        return m_pDisplay->GetChildContainerId(ChildUid, pChildContainIdArg);
    }

protected:

    GcKmAdapter(
        IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
        WCHAR  *pwszDeviceId,
        BOOLEAN IsSoftwareDevice);
    virtual ~GcKmAdapter();

    void* operator new(size_t  size);
    void operator delete(void* ptr);

public:

    static GcKmAdapter* Cast(void* ptr)
    {
        GcKmAdapter* pGcKmAdapter = static_cast<GcKmAdapter *>(ptr);

        // NT_ASSERT(pGcKmAdapter->m_magic == GcKmAdapter::kMagic);

        return pGcKmAdapter;
    }

    DXGKRNL_INTERFACE* GetDxgkInterface()
    {
        return &m_DxgkInterface;
    }

protected:

    friend class GcKmdDdi;

    //
    // If Start() succeeds, then Stop() must be called to clean up. For example,
    // if a subclass first calls Start() and then does controller-specific
    // stuff that fails, it needs to call Stop() before returning failure
    // to the framework.
    //
    virtual NTSTATUS Start(
        IN_PDXGK_START_INFO     DxgkStartInfo,
        IN_PDXGKRNL_INTERFACE   DxgkInterface,
        OUT_PULONG              NumberOfVideoPresentSources,
        OUT_PULONG              NumberOfChildren);

    virtual NTSTATUS Stop();

    void RegisterMmu(
        GcKmMmu    *pMmu)
    {
        m_pMmu = pMmu;
    }

    virtual UINT GetNumNodes() = 0;

    virtual UINT GetPagingNode() = 0;

    void RegisterNode(
        UINT        Index,
        GcKmNode   *pNode)
    {
        NT_ASSERT(Index < m_NumNodes);

        m_Nodes[Index] = pNode;
    }

    void RegisterDisplay(
        GcKmDisplay    *pDisplay)
    {
        m_pDisplay = pDisplay;
    }

    virtual NTSTATUS AllocateTextureDescriptor(
        GcKmAllocation* pAllocation)
    {
        return STATUS_SUCCESS;
    }

    virtual void ReleaseTextureDescriptor(
        GcKmAllocation* pAllocation)
    {
    }

private:

    void DpcRoutine(void);

protected:

    DXGK_WDDMVERSION            m_WddmVersion;

    DEVICE_OBJECT              *m_pPhysicalDevice;
    DXGKRNL_INTERFACE           m_DxgkInterface;
    DXGK_START_INFO             m_DxgkStartInfo;

    DXGK_DEVICE_INFO            m_DeviceInfo;

    WCHAR                       m_DeviceId[MAX_DEVICE_ID_LENGTH];
    BOOLEAN                     m_IsSoftwareDevice;

    GcKmMmu                    *m_pMmu;

    static const UINT   kMaxNodes = 16;

    UINT                        m_NumNodes;
    GcKmNode                   *m_Nodes[kMaxNodes];

    GcKmDisplay                *m_pDisplay;

    HANDLE                      m_hGdiRegKey = NULL;
};

