/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

class GcKmProcess
{
public:

    GcKmProcess(
        GcKmAdapter            *pGcKmAdapter,
        DXGKARG_CREATEPROCESS  *pCreateProcess)
    {
        m_pGcKmAdapter = pGcKmAdapter;

        m_hDxgkProcess = pCreateProcess->hDxgkProcess;
        m_Flags = pCreateProcess->Flags;
    };

    virtual ~GcKmProcess()
    {
    }

protected:

    GcKmAdapter                *m_pGcKmAdapter;
    HANDLE                      m_hDxgkProcess;
    DXGK_CREATEPROCESSFLAGS     m_Flags;
};

