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

#include "soc.h"
#include "HalExtiMXDmaCfg.h"

void CompletionRoutine(
    PDMA_ADAPTER DmaAdapter,
    PDEVICE_OBJECT DeviceObject,
    PVOID CompletionContext,
    DMA_COMPLETION_STATUS Status
)
{
    (void)DmaAdapter;
    (void)DeviceObject;
    (void)CompletionContext;

    NT_ASSERT(CompletionContext);
    CDmaBuffer* Dma = (CDmaBuffer*)CompletionContext;
    Dma->Notify(Status);
};

NTSTATUS
CDmaBuffer::Init
(
    _In_ PDEVICE_OBJECT     PDO,
    _In_ PHYSICAL_ADDRESS   DataReg,
    _In_ ULONG              DataRegSize,
    _In_ ULONG              RequestLine,
    _In_ ULONG              Channel,
    _In_ ULONG              Watermark
)
{
    _DEVICE_DESCRIPTION deviceDescript = { 0 };
    DBG_DRV_METHOD_BEG();

    if ((m_pDmaAdapter != NULL) && (DataRegSize == m_SampleSize)) {
        /* Sample size not changed and DMA adapter functional */
        return STATUS_SUCCESS;
    }
    else {
        // This does not happen in the current driver as the SampleSize is fixed
        if (m_pDmaAdapter != NULL) {
            m_pDmaAdapter->DmaOperations->FreeAdapterObject(m_pDmaAdapter, DeallocateObject);
            m_pDmaAdapter = NULL;
        }
    }
    m_SampleSize = DataRegSize;
    deviceDescript.Version = DEVICE_DESCRIPTION_VERSION3;
    deviceDescript.DeviceAddress = DataReg;
    deviceDescript.DmaRequestLine = m_RequestLine = RequestLine;
    deviceDescript.InterfaceType = ACPIBus;
    switch (DataRegSize) {
    default:
        deviceDescript.DmaWidth = Width32Bits;
        break;
    case 16:
        deviceDescript.DmaWidth = Width16Bits;
        break;
    }
    deviceDescript.DmaAddressWidth = 32;
    deviceDescript.DmaChannel = Channel;
    deviceDescript.AutoInitialize = true;
    deviceDescript.ScatterGather = true;

    m_pDmaAdapter = IoGetDmaAdapter(PDO, &deviceDescript, &m_NumberOfMapRegs);
    if (!m_pDmaAdapter) {
        DBG_DRV_PRINT_VERBOSE("IoGetDmaAdapter failed\n");
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    m_pPDO = PDO;
    m_Watermark = Watermark;
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

NTSTATUS
CDmaBuffer::AllocBuffer
(   
    _In_        eDeviceType            DeviceType,
    _In_        ULONG                  Size,
    _Out_       PMDL* pMdl,
    _Out_       MEMORY_CACHING_TYPE* CacheType
)
{
    PHYSICAL_ADDRESS maxAddr;
    const BOOLEAN cacheEn = false;
    DBG_DRV_METHOD_BEG();

    (void)DeviceType;

    ASSERT(m_pDmaAdapter);
    ASSERT(m_BufMdl == NULL);
    ASSERT(m_BufVirt == NULL);

    if (!m_pDmaAdapter || m_BufMdl || m_BufVirt) {
        return STATUS_DRIVER_INTERNAL_ERROR;
    }

    maxAddr.QuadPart = 0xffffffff;
    m_BufVirt = m_pDmaAdapter->DmaOperations->AllocateCommonBufferEx(m_pDmaAdapter, &maxAddr, Size, &m_BufLogical, cacheEn, 0);
    if (!m_BufVirt) {
        DBG_DRV_PRINT_WARNING("AllocateCommonBufferEx failed\n");
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    m_BufMdl = IoAllocateMdl(m_BufVirt, Size, false, false, NULL);
    if (!m_BufMdl) {
        DBG_DRV_PRINT_WARNING("IoAllocateMdl failed\n");
        m_pDmaAdapter->DmaOperations->FreeCommonBuffer(m_pDmaAdapter, Size, m_BufLogical, m_BufVirt, false);
        m_BufVirt = NULL;
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    MmBuildMdlForNonPagedPool(m_BufMdl);
    *pMdl = m_BufMdl;
    if (cacheEn)
        *CacheType = MmCached;
    else 
        *CacheType = MmNonCached;

    m_BufSize = Size;
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

NTSTATUS
CDmaBuffer::FreeBuffer
(
    _In_        eDeviceType            DeviceType,
    _In_        ULONG                  Size,
    _Out_       PMDL                   pMdl
)
{
    const BOOLEAN cacheEn = false;
    (void)DeviceType;

    ASSERT(Size);
    ASSERT(pMdl);
    ASSERT(m_pDmaAdapter);
    ASSERT(m_BufVirt);
    ASSERT(pMdl == m_BufMdl);

    if (!m_pDmaAdapter || !m_BufVirt) {
        return STATUS_DRIVER_INTERNAL_ERROR;
    }

    DBG_DRV_METHOD_BEG();
    m_pDmaAdapter->DmaOperations->FreeCommonBuffer(m_pDmaAdapter, Size, m_BufLogical, m_BufVirt, cacheEn);
    IoFreeMdl(pMdl);

    m_BufVirt = NULL;
    m_BufMdl = NULL;
    m_BufSize = 0;
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
NTSTATUS
CDmaBuffer::RegisterStream
(
    _In_        CMiniportWaveRTStream* Stream,
    eDeviceType DeviceType
)
{
    NTSTATUS status;
    ULONG nMapRegs;
    KIRQL oldIRQL;
    BOOLEAN isWriter = DeviceType == eSpeakerHpDevice ? true : false;
    DBG_DRV_METHOD_BEG();

    ASSERT(m_pStream == NULL);
    ASSERT(Stream);
    ASSERT(m_BufMdl);
    ASSERT(m_BufSize);
    ASSERT(m_pPDO);

    if (!Stream) {
        return STATUS_INVALID_PARAMETER;
    }
    if (m_pStream || !m_BufMdl || !m_BufSize || !m_pPDO) {
        return STATUS_DRIVER_INTERNAL_ERROR;
    }

    m_TransferInfo.Version = 1;
    m_SamplesTransferred = 0;

    status = m_pDmaAdapter->DmaOperations->GetDmaTransferInfo(m_pDmaAdapter, m_BufMdl, 0, m_BufSize, isWriter, &m_TransferInfo);
    if (!NT_SUCCESS(status)) {
        DBG_DRV_PRINT_WARNING("GetDmaTransferInfo failed");
        return status;
    }
    DBG_DRV_PRINT_VERBOSE("MapRegisterCount %u", m_TransferInfo.V1.MapRegisterCount);
    DBG_DRV_PRINT_VERBOSE("ScatterGatherElementCount %u", m_TransferInfo.V1.ScatterGatherElementCount);
    DBG_DRV_PRINT_VERBOSE("ScatterGatherListSize %u", m_TransferInfo.V1.ScatterGatherListSize);

    status = m_pDmaAdapter->DmaOperations->InitializeDmaTransferContext(m_pDmaAdapter, m_DmaTransferContext);
    if (!NT_SUCCESS(status)) {
        DBG_DRV_PRINT_WARNING("InitializeDmaTransferContext failed");
        return status;
    }

    nMapRegs = m_TransferInfo.V1.MapRegisterCount < m_NumberOfMapRegs ? m_TransferInfo.V1.MapRegisterCount : m_NumberOfMapRegs;
    KeRaiseIrql(DISPATCH_LEVEL, &oldIRQL);
    status = m_pDmaAdapter->DmaOperations->AllocateAdapterChannelEx(m_pDmaAdapter, m_pPDO, m_DmaTransferContext, nMapRegs, DMA_SYNCHRONOUS_CALLBACK, NULL, NULL, &m_MapRegisterBase);
    KeLowerIrql(oldIRQL);
    if (!NT_SUCCESS(status)) {
        DBG_DRV_PRINT_WARNING("AllocateAdapterChannelEx failed %d", status);
        return status;
    }

    ULONG notif = Stream->GetNotificationsPerBuffer();
    DBG_DRV_PRINT_VERBOSE("Buffer size %u", m_BufSize);
    DBG_DRV_PRINT_VERBOSE("Notifications %u", notif);

    m_NotificationBytes = m_BufSize / notif;
    status = m_pDmaAdapter->DmaOperations->ConfigureAdapterChannel(
        m_pDmaAdapter,
        SDMA_CFG_FUN_SET_CHANNEL_NOTIFICATION_THRESHOLD,
        &m_NotificationBytes);

    if (!NT_SUCCESS(status)) {
        DBG_DRV_PRINT_WARNING("SDMA_CFG_FUN_SET_CHANNEL_NOTIFICATION_THRESHOLD failed %d", status);
        return status;
    }

    // Calculate watermark in number of bytes transfered - it is DMA Burst size.
    ULONG watermarkLevel = m_Watermark * (m_SampleSize >> 3);

    status = m_pDmaAdapter->DmaOperations->ConfigureAdapterChannel(
        m_pDmaAdapter,
        SDMA_CFG_FUN_SET_CHANNEL_WATERMARK_LEVEL,
        &watermarkLevel);

    if (!NT_SUCCESS(status)) {
        DBG_DRV_PRINT_WARNING("SDMA_CFG_FUN_SET_CHANNEL_WATERMARK_LEVEL failed %d", status);
        return status;
    }

    status = m_pDmaAdapter->DmaOperations->ConfigureAdapterChannel(
        m_pDmaAdapter,
        SDMA_CFG_FUN_ACQUIRE_REQUEST_LINE,
        &m_RequestLine);

    if (!NT_SUCCESS(status)) {
        DBG_DRV_PRINT_WARNING("SDMA_CFG_FUN_ACQUIRE_REQUEST_LINE failed %d", status);
        return status;
    }

    m_pWfExt = Stream->GetDataFormat();
    m_pStream = Stream;
    m_DeviceType = DeviceType;

    DBG_DRV_PRINT_VERBOSE("RegisterStream %s done", isWriter ? "Tx" : "Rx");
    return STATUS_SUCCESS;
}

#pragma code_seg()
NTSTATUS
CDmaBuffer::UnregisterStream
(
    _In_        CMiniportWaveRTStream* Stream,
    eDeviceType DeviceType
)
{
    NTSTATUS status = STATUS_SUCCESS;
    KIRQL oldIRQL;
    BOOLEAN isWriter = DeviceType == eSpeakerHpDevice ? true : false;
    DBG_DRV_METHOD_BEG();

    (void)DeviceType;
    (void)Stream;
    ASSERT(m_pStream == Stream);
    ASSERT(m_pDmaAdapter);
    ASSERT(m_BufMdl);

    if ((m_pStream != Stream) || !m_pDmaAdapter) {
        return STATUS_DRIVER_INTERNAL_ERROR;
    }

    status = m_pDmaAdapter->DmaOperations->CancelMappedTransfer(m_pDmaAdapter, m_DmaTransferContext);
    NT_ASSERT(NT_SUCCESS(status));

    status = m_pDmaAdapter->DmaOperations->FlushAdapterBuffersEx(m_pDmaAdapter, m_BufMdl, m_MapRegisterBase, 0, m_BufSize, isWriter);
    NT_ASSERT(NT_SUCCESS(status));

    DBG_DRV_PRINT_VERBOSE("FreeAdapterChannel");
    KeRaiseIrql(DISPATCH_LEVEL, &oldIRQL);
    m_pDmaAdapter->DmaOperations->FreeAdapterChannel(m_pDmaAdapter);
    KeLowerIrql(oldIRQL);

    DBG_DRV_PRINT_VERBOSE("%s Isr count %u", isWriter ? "playback" : "capture", cntIsr);

    m_isStarted = false;
    m_pStream = NULL;
    m_pWfExt = NULL;
    DBG_DRV_METHOD_END();
    return status;
}

NTSTATUS
CDmaBuffer::Start
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    NTSTATUS status = STATUS_SUCCESS;
    BOOLEAN isWriter = m_DeviceType == eSpeakerHpDevice ? true : false;
    DBG_DRV_METHOD_BEG();
    (void)Stream;
    ASSERT(Stream == m_pStream);

    if ((m_pStream != Stream) || !m_pDmaAdapter) {
        return STATUS_DRIVER_INTERNAL_ERROR;
    }

    if (!m_isStarted) {
#if DBG
        cntIsr = 0;
#endif
        m_DmaLength = m_BufSize;
        status = m_pDmaAdapter->DmaOperations->MapTransferEx(m_pDmaAdapter, m_BufMdl, m_MapRegisterBase, 0, 0, &m_DmaLength, isWriter, NULL, 0, CompletionRoutine, this);
        if (!NT_SUCCESS(status)) {
            DBG_DRV_PRINT_VERBOSE("MapTransferEx failed %d", status);
            return status;
        }
        if (m_DmaLength != m_BufSize) {
            DBG_DRV_PRINT_WARNING("MapTransferEx size is %u", m_DmaLength);
        }
        m_isStarted = true;
    }
    DBG_DRV_METHOD_END();
    return status;
}