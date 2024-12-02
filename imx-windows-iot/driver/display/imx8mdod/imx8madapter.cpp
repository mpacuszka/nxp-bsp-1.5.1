/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"
#include "imx8madapter.tmh"


void* Imx8mDodAdapter::operator new(size_t size)
{
    return ExAllocatePoolWithTag(NonPagedPoolNx, size, 'DAMI'); // Tag for Imx8m Dod Adapter
}

void Imx8mDodAdapter::operator delete(void* ptr)
{
    ExFreePool(ptr);
}

Imx8mDodAdapter::Imx8mDodAdapter(
    IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
    WCHAR* pwszDeviceId) :
    GcKmAdapter(PhysicalDeviceObject, pwszDeviceId, false)
{
    // do nothing
}

NTSTATUS
Imx8mDodAdapter::Start(
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

        status = DisplayStartController(
                    GcKmdGlobal::s_DriverMode,
                    &m_DxgkInterface,
                    0,
                    &m_pDisplay,
                    NumberOfVideoPresentSources,
                    NumberOfChildren);
        if (!NT_SUCCESS(status))
        {
            break;
        }
    } while (FALSE);

    if (!NT_SUCCESS(status))
    {
        Stop();

        return status;
    }

    return status;
}

NTSTATUS
Imx8mDodAdapter::Stop()
{
    DisplayStopController(m_pDisplay);

    return GcKmAdapter::Stop();
}

BOOLEAN
Imx8mDodAdapter::InterruptRoutine(
    IN_ULONG        MessageNumber)
{
    BOOLEAN bRet = false;

    if (m_pDisplay)
    {
        bRet |= m_pDisplay->InterruptRoutine(MessageNumber);
    }

    return bRet;
}

NTSTATUS
Imx8mDodAdapter::QueryAdapterInfo(
    IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo)
{
    GC_LOG_TRACE(
        "QueryAdapterInfo was called. (Type=%d)",
        pQueryAdapterInfo->Type);

    switch (pQueryAdapterInfo->Type)
    {
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

    default:
        return GcKmAdapter::QueryAdapterInfo(pQueryAdapterInfo);
    }
}

NTSTATUS 
Imx8mDodAdapter::SetPowerState(
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    return m_pDisplay->SetPowerState(DeviceUid, DevicePowerState, ActionType);
}

