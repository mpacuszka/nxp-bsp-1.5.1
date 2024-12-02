/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "GcKmd.h"

class GcKmAdapter;
class GcKmProcess;

class GcKmDevice
{
public: // NONPAGED

    __forceinline static GcKmDevice* Cast (HANDLE hDevice)
    {
        return static_cast<GcKmDevice*>(hDevice);
    }
    
    GcKmProcess*
    GetProcess()
    {
        return m_pGcKmProcess;
    }

    static NTSTATUS __stdcall
    DdiCloseAllocation(
        IN_CONST_HANDLE                     hDevice,
        IN_CONST_PDXGKARG_CLOSEALLOCATION   pCloseAllocation);

private:

    DXGK_CREATEDEVICEFLAGS  m_Flags;
    HANDLE                  m_hRTDevice;

    GcKmProcess            *m_pGcKmProcess;

public:

    GcKmAdapter            *m_pGcKmAdapter;

public: // PAGED

    NTSTATUS
    Initialize(
        GcKmAdapter*            pAdapter,
        DXGKARG_CREATEDEVICE*   pCreateDevice);

    virtual NTSTATUS
    CreateContext(
        DXGKARG_CREATECONTEXT*  pCreateContext);

    _Check_return_
    _Function_class_DXGK_(DXGKDDI_OPENALLOCATIONINFO)
    _IRQL_requires_(PASSIVE_LEVEL)
    NTSTATUS OpenAllocation (IN_CONST_PDXGKARG_OPENALLOCATION);

    HANDLE
    GetRtDevice()
    {
        return m_hRTDevice;
    }
};

