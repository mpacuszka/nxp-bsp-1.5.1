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
 *
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


#pragma once

#include "minwavertstream.h"
#include "imx_audio.h"

class CDmaBuffer {
public:
    CDmaBuffer()
    {
        m_pDmaAdapter = NULL;
        m_BufVirt = NULL;
        m_pStream = NULL;
        m_isStarted = false;
        m_pPDO = NULL;
        m_MapRegisterBase = NULL;
        m_BufLogical.QuadPart = 0;
        m_BufSize = 0;
        m_BufMdl = NULL;
        m_NumberOfMapRegs = 0;
        m_DmaLength = 0;
        m_SamplesTransferred = 0;
        m_NotificationBytes = 0;
        m_SampleSize = 0;
        m_Watermark = 0;
        m_RequestLine = 0;
        m_DeviceType = eMaxDeviceType;
        m_pWfExt = NULL;
        m_TransferInfo = { 0 };
        RtlZeroMemory(m_DmaTransferContext, sizeof(m_DmaTransferContext));
#if DBG
        cntIsr = 0;
#endif
    };
    ~CDmaBuffer() {
        if (m_pStream) {
            UnregisterStream(m_pStream, m_DeviceType);
        }
        if (m_BufVirt && m_BufSize && m_BufMdl) {
            FreeBuffer(m_DeviceType, m_BufSize, m_BufMdl);
        }
        if (m_pDmaAdapter) {
            m_pDmaAdapter->DmaOperations->FreeAdapterObject(m_pDmaAdapter, DeallocateObject);
            m_pDmaAdapter = NULL;
        }
    };
public:
    NTSTATUS Init
    (
        _In_ PDEVICE_OBJECT     PDO,
        _In_ PHYSICAL_ADDRESS   DataReg,
        _In_ ULONG              DataRegSize,
        _In_ ULONG              RequestLine,
        _In_ ULONG              Channel,
        _In_ ULONG              Watermark
    );

    VOID Notify(DMA_COMPLETION_STATUS Status)
    {
        if (Status == DmaComplete) {
            if (m_pStream) {
#if DBG
                cntIsr++;
#endif
                m_SamplesTransferred += (m_NotificationBytes / m_pWfExt->Format.nBlockAlign);
                m_pStream->UpdateVirtualPositionRegisters(m_SamplesTransferred);
            }
        }
    }

    NTSTATUS RegisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType DeviceType
    );

    NTSTATUS UnregisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType DeviceType
    );

    NTSTATUS AllocBuffer
    (   
        _In_        eDeviceType            DeviceType,
        _In_        ULONG                  Size,
        _Out_       PMDL* pMdl,
        _Out_       MEMORY_CACHING_TYPE* CacheType
    );

    NTSTATUS FreeBuffer
    (
        _In_        eDeviceType            DeviceType,
        _In_        ULONG                  Size,
        _Out_       PMDL                   pMdl
    );

    NTSTATUS Start
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    BOOLEAN IsMyStream(CMiniportWaveRTStream* stream)
    {
        if (stream == m_pStream)
        {
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }

private:
    _DMA_ADAPTER* m_pDmaAdapter;
    PDEVICE_OBJECT       m_pPDO;
    PVOID                m_MapRegisterBase;
    PVOID                m_BufVirt;
    PHYSICAL_ADDRESS     m_BufLogical;
    ULONG                m_BufSize;
    PMDL                 m_BufMdl;
    ULONG                m_NumberOfMapRegs;
    ULONG                m_DmaLength;
    ULONG                m_SamplesTransferred;
    ULONG                m_NotificationBytes;
    DMA_TRANSFER_INFO    m_TransferInfo;
    UINT                 m_DmaTransferContext[DMA_TRANSFER_CONTEXT_SIZE_V1 / sizeof(UINT)];
    CMiniportWaveRTStream* m_pStream;
    ULONG                m_SampleSize;
    ULONG                m_Watermark;
    ULONG                m_RequestLine;
    eDeviceType          m_DeviceType;
    BOOLEAN              m_isStarted;
    PWAVEFORMATEXTENSIBLE  m_pWfExt;
#if DBG
    volatile ULONG       cntIsr;
#endif
};
