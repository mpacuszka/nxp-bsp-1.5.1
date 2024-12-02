/* Copyright (c) Microsoft Corporation.
   Modifications Copyright 2023 NXP
   Licensed under the MIT License. */
#include "malonemft.h"
#include "CAutoLock.h"
#include <mfapi.h>
#include <mferror.h>
#include "malonemft_DebugLogger.h"
#include <initguid.h>

HRESULT CMaloneMft::SetDropMode(
    /* [in] */ MF_QUALITY_DROP_MODE eDropMode)
{
    if (this->m_bShutdown) {
        return MF_E_SHUTDOWN;
    }

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): value=%d", __FUNCTION__, (int)eDropMode);

    CAutoLock lock(&m_csLock);

    switch (eDropMode) {
        case MF_DROP_MODE_NONE:
        case MF_DROP_MODE_1:
        case MF_DROP_MODE_2:
            m_dropMode = eDropMode;
            break;

        default:
            return MF_E_NO_MORE_DROP_MODES;
    }

    return S_OK;
}

HRESULT CMaloneMft::SetQualityLevel(
    /* [in] */ MF_QUALITY_LEVEL eQualityLevel)
{
    return MF_E_NO_MORE_QUALITY_LEVELS;
}

HRESULT CMaloneMft::GetDropMode(
    /* [annotation][out] */
    _Out_  MF_QUALITY_DROP_MODE *peDropMode)
{
    if (this->m_bShutdown) {
        return MF_E_SHUTDOWN;
    }

    CAutoLock lock(&m_csLock);

    *peDropMode = m_dropMode;

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): value=%d", __FUNCTION__, (int)m_dropMode);

    return S_OK;
}


HRESULT CMaloneMft::GetQualityLevel(
    /* [annotation][out] */
    _Out_  MF_QUALITY_LEVEL *peQualityLevel)
{
    return E_NOTIMPL;
}


HRESULT CMaloneMft::DropTime(
    /* [in] */ LONGLONG hnsAmountToDrop)
{
    return MF_E_DROPTIME_NOT_SUPPORTED;
}
