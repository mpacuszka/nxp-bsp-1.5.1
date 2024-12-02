/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    CSoc class declaration.

*/
#pragma once

#include "common.h"
#include "minwavertstream.h"

class CSocBase;

class CSocBase
{
public:
    CSocBase()
    {
        m_pRegisters = NULL;
        m_pInterruptObject = NULL;
        m_bIsRenderActive = FALSE;
        m_pPDO = NULL;
        m_RegSize = 0;
    }

    virtual ~CSocBase() {};

public:
    virtual NTSTATUS InitBlock
    (
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR registersDescriptor,
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR interruptDescriptor,
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR txDmaResourcePtr,
        _In_ PDEVICE_OBJECT PDO
    ) = 0;

    virtual NTSTATUS RegisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    ) = 0;

    virtual NTSTATUS UnregisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    ) = 0;

    virtual NTSTATUS StartDma
    (
        _In_        CMiniportWaveRTStream* Stream
    ) = 0;

    virtual NTSTATUS PauseDma
    (
        _In_        CMiniportWaveRTStream* Stream
    ) = 0;

    virtual NTSTATUS StopDma
    (
        _In_        CMiniportWaveRTStream* Stream
    ) = 0;

    virtual NTSTATUS AllocBuffer
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType                        DeviceType,
        _Inout_     PULONG                 Size,
        _Out_       PMDL*                  pMdl,
        _Out_       MEMORY_CACHING_TYPE*   CacheType
    ) = 0;

    virtual NTSTATUS FreeBuffer
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType                        DeviceType,
        _In_        PMDL                   Mdl,
        _In_        ULONG                  Size
    ) = 0;

protected:

    BOOLEAN m_bIsRenderActive;

    volatile UINT8*                  m_pRegisters;
    ULONG                            m_RegSize;
    PDEVICE_OBJECT                   m_pPDO;
    PKINTERRUPT                      m_pInterruptObject;
};

