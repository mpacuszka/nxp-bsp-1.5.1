/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

class Imx8mDodAdapter : public GcKmAdapter
{
public:

    Imx8mDodAdapter(
        IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
        WCHAR* pwszDeviceId);

    virtual ~Imx8mDodAdapter()
    {
        // do nothing
    }

    void* operator new(size_t  size);
    void operator delete(void* ptr);

protected:

    virtual NTSTATUS Start(
        IN_PDXGK_START_INFO     DxgkStartInfo,
        IN_PDXGKRNL_INTERFACE   DxgkInterface,
        OUT_PULONG              NumberOfVideoPresentSources,
        OUT_PULONG              NumberOfChildren);

    virtual NTSTATUS Stop();

    virtual BOOLEAN InterruptRoutine(
        IN_ULONG        MessageNumber);

    virtual UINT GetNumNodes()
    {
        return 0;
    }

    virtual UINT GetPagingNode()
    {
        return 0;
    }

    virtual NTSTATUS QueryAdapterInfo(
        IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo);

    virtual NTSTATUS GetNodeMetaData(
        UINT                            NodeOrdinal,
        OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata)
    {
        return STATUS_NOT_SUPPORTED;
    }

    virtual NTSTATUS CollectDbgInfo(
        IN_CONST_PDXGKARG_COLLECTDBGINFO    pCollectDbgInfo)
    {
        return STATUS_SUCCESS;
    }

    virtual NTSTATUS ResetFromTimeout()
    {
        return STATUS_SUCCESS;
    }

    virtual NTSTATUS RestartFromTimeout()
    {
        return STATUS_SUCCESS;
    }

    virtual NTSTATUS Escape(
        IN_CONST_PDXGKARG_ESCAPE    pEscape)
    {
        return STATUS_INVALID_PARAMETER;
    }

    virtual NTSTATUS SetPowerState(
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType);
};
