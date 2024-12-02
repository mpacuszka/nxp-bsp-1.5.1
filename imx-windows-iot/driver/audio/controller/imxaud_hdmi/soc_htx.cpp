/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    CSoc_htx class implementation.

*/

#include "soc_htx.h"
#include <ImxCpuRev.h>
#include "HalExtiMXDmaCfg.h"

#pragma code_seg()

void CompletionRoutine(
    PDMA_ADAPTER DmaAdapter,
    PDEVICE_OBJECT DeviceObject,
    PVOID CompletionContext,
    DMA_COMPLETION_STATUS Status
);

#pragma code_seg("PAGE")
VOID
CSoc_htx::CleanUp()
{
    PAGED_CODE();

    DBG_DRV_METHOD_BEG();

    if (m_pDmaAdapter) {
        m_pDmaAdapter->DmaOperations->PutDmaAdapter(m_pDmaAdapter);
        m_pDmaAdapter = NULL;
    }
    if (m_pRegisters != NULL)
    {
        MmUnmapIoSpace((PVOID)m_pRegisters, m_RegSize);
        m_pRegisters = NULL;
    }
    DBG_DRV_METHOD_END();
}

#pragma code_seg("PAGE")
NTSTATUS CSoc_htx::InitBlock
(
    _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR registersDescriptor,
    _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR interruptDescriptor,
    _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR txDmaResourcePtr,
    _In_ PDEVICE_OBJECT PDO
)
{
    _DEVICE_DESCRIPTION deviceDescript = { 0 };
    NTSTATUS ntStatus = STATUS_SUCCESS;
    PHYSICAL_ADDRESS writeReg;

    PAGED_CODE();

    UNREFERENCED_PARAMETER(interruptDescriptor);

    DBG_DRV_METHOD_BEG();

    DBG_DRV_PRINT_INFO("Tx dma chnl %u req %u", txDmaResourcePtr->u.DmaV3.Channel, txDmaResourcePtr->u.DmaV3.RequestLine);

    ASSERT(m_pRegisters == NULL);
    if (registersDescriptor->u.Memory.Length >= 1/* adjust to real size e.g. sizeof(SaiRegisters) */)
    {
        m_RegSize = /* adjust to real size e.g. sizeof(SaiRegisters) */ registersDescriptor->u.Memory.Length;
        m_pRegisters = (UINT8*)MmMapIoSpaceEx(m_RegBase = registersDescriptor->u.Memory.Start,
            m_RegSize,
            PAGE_READWRITE | PAGE_NOCACHE);
        ASSERT(m_pRegisters);
        if (m_pRegisters == NULL)
        {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            goto Done;
        }
    }
    else {
        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        goto Done;
    }

    m_pPDO = PDO;
    m_pAud2HtxRegisters = (PAUD2HTX_REGISTERS) m_pRegisters;

    // Channel number occupies lower 7bits. SDMA instance 7th and 8th bit.
    ULONG TxDmaInstance = (txDmaResourcePtr->u.DmaV3.Channel >> 7) & 0x3;
    ULONG TxDmaChannel = txDmaResourcePtr->u.DmaV3.Channel & 0x7F;
    ULONG TxDmaRequestLine = txDmaResourcePtr->u.DmaV3.RequestLine;

    NT_ASSERT(TxDmaInstance <= 2);
    NT_ASSERT(TxDmaRequestLine <= SDMA_REQ_LINE_ID_MASK);

    // Base address of data register 
    writeReg.QuadPart = m_RegBase.QuadPart + AUD2HTX_DATA_REGISTER_OFFSET;
        
    deviceDescript.Version = DEVICE_DESCRIPTION_VERSION3;
    deviceDescript.DeviceAddress = writeReg;
    deviceDescript.DmaRequestLine = m_RequestLine = TxDmaRequestLine | (TxDmaInstance << SDMA_INSTANCE_ID_SHIFT);
    deviceDescript.InterfaceType = ACPIBus;
    deviceDescript.DmaWidth = AUD2HTX_DMA_WIDTH;
    deviceDescript.DmaAddressWidth = 32;
    deviceDescript.DmaChannel = TxDmaChannel;
    deviceDescript.AutoInitialize = true;   
    deviceDescript.ScatterGather = true;

    m_pDmaAdapter = IoGetDmaAdapter(PDO, &deviceDescript, &m_NumberOfMapRegs);
    if (!m_pDmaAdapter) {
        DBG_DRV_PRINT_WARNING("IoGetDmaAdapter failed");
        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        goto Done;
    }

    ntStatus = Setup();

Done:
    // Cleanup
    if (ntStatus != STATUS_SUCCESS)
    {
        CleanUp();
    }
    DBG_DRV_METHOD_END();
    return ntStatus;
}

#pragma code_seg()
NTSTATUS
CSoc_htx::FreeBuffer
(
    _In_        CMiniportWaveRTStream* Stream,
    eDeviceType            DeviceType,
    _In_        PMDL                   Mdl,
    _In_        ULONG                  Size
)
{
    UNREFERENCED_PARAMETER(Stream);
    UNREFERENCED_PARAMETER(DeviceType);
    DBG_DRV_METHOD_BEG();

    NT_ASSERT(Size);
    NT_ASSERT(Mdl);
    NT_ASSERT(m_pDmaAdapter);
    NT_ASSERT(m_BufVirt);
    NT_ASSERT(Mdl == m_BufMdl);

    m_pDmaAdapter->DmaOperations->FreeCommonBuffer(m_pDmaAdapter, Size, m_BufLogical, m_BufVirt, false);
    IoFreeMdl(Mdl);
    m_BufVirt = NULL;
    m_BufMdl = NULL;
    m_BufSize = 0;
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

NTSTATUS
CSoc_htx::AllocBuffer
(
    _In_        CMiniportWaveRTStream* Stream,
    eDeviceType            DeviceType,
    _Inout_     PULONG                 Size,
    _Out_       PMDL* pMdl,
    _Out_       MEMORY_CACHING_TYPE* CacheType
)
{
    PHYSICAL_ADDRESS maxAddr;
    ULONG RequestedSize, Res;

    UNREFERENCED_PARAMETER(Stream);
    UNREFERENCED_PARAMETER(DeviceType);
    DBG_DRV_METHOD_BEG();

    NT_ASSERT(m_pDmaAdapter);
    NT_ASSERT(m_BufVirt == NULL);
    NT_ASSERT(m_BufMdl == NULL);

    RequestedSize = *Size;
    // Round the buffer for DMA to support various sample rates.
    Res = RequestedSize % 64U;
    if (Res != 0) {
        RequestedSize -= Res;
        RequestedSize += 64U;
    }
    maxAddr.QuadPart = 0xffffffff;
    m_BufVirt = m_pDmaAdapter->DmaOperations->AllocateCommonBufferEx(m_pDmaAdapter, &maxAddr, RequestedSize, &m_BufLogical, false, 0);
    if (!m_BufVirt) {
        DBG_DRV_PRINT_WARNING("AllocateCommonBufferEx failed");
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    m_BufMdl = IoAllocateMdl(m_BufVirt, *Size, false, false, NULL);
    if (!m_BufMdl) {
        DBG_DRV_PRINT_WARNING("IoAllocateMdl failed");
        m_pDmaAdapter->DmaOperations->FreeCommonBuffer(m_pDmaAdapter, RequestedSize, m_BufLogical, m_BufVirt, false);
        m_BufVirt = NULL;
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    MmBuildMdlForNonPagedPool(m_BufMdl);
    *pMdl = m_BufMdl;
    *CacheType = MmNonCached;
    m_BufSize = RequestedSize;
    *Size = RequestedSize;
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
NTSTATUS
CSoc_htx::RegisterStream
(
    _In_        CMiniportWaveRTStream* Stream,
                eDeviceType DeviceType
)
{
    NTSTATUS status;
    ULONG nMapRegs;
    KIRQL oldIRQL;

    DBG_DRV_METHOD_BEG();
    UNREFERENCED_PARAMETER(DeviceType);
    
    m_pRtStream = Stream;
    m_ulSamplesTransferred = 0;  
    m_pWfExt = Stream->GetDataFormat();
    
    NT_ASSERT(m_BufMdl);
    NT_ASSERT(m_BufSize);
    NT_ASSERT(Stream->GetDmaBufferSize() == m_BufSize);
    NT_ASSERT(Stream->GetDmaBufferMdl() == m_BufMdl);
    NT_ASSERT(m_pDmaAdapter);

    m_TransferInfo.V1.MapRegisterCount = 0;
    m_TransferInfo.V1.ScatterGatherElementCount = 0;
    m_TransferInfo.V1.ScatterGatherListSize = 0;
    m_TransferInfo.Version = 1;
    
    status = m_pDmaAdapter->DmaOperations->GetDmaTransferInfo(m_pDmaAdapter, m_BufMdl, 0, m_BufSize, true, &m_TransferInfo);
    NT_ASSERT(NT_SUCCESS(status));

    status = m_pDmaAdapter->DmaOperations->InitializeDmaTransferContext(m_pDmaAdapter, m_DmaTransferContext);
    NT_ASSERT(NT_SUCCESS(status));
    
    nMapRegs = m_TransferInfo.V1.MapRegisterCount < m_NumberOfMapRegs ? m_TransferInfo.V1.MapRegisterCount : m_NumberOfMapRegs;
    KeRaiseIrql(DISPATCH_LEVEL, &oldIRQL);
    status = m_pDmaAdapter->DmaOperations->AllocateAdapterChannelEx(m_pDmaAdapter, m_pPDO, m_DmaTransferContext, nMapRegs, DMA_SYNCHRONOUS_CALLBACK, NULL, NULL, &m_MapRegisterBase);
    KeLowerIrql(oldIRQL);
    NT_ASSERT(NT_SUCCESS(status));
    
    ULONG notif = m_pRtStream->GetNotificationsPerBuffer();
    DBG_DRV_PRINT_VERBOSE("Buffer size %u", m_BufSize);
    DBG_DRV_PRINT_VERBOSE("Notifications %u", notif);
    
    m_NotificationBytes = m_BufSize / notif;
    status = m_pDmaAdapter->DmaOperations->ConfigureAdapterChannel(m_pDmaAdapter, SDMA_CFG_FUN_SET_CHANNEL_NOTIFICATION_THRESHOLD,
        &m_NotificationBytes);
    NT_ASSERT(NT_SUCCESS(status));

    ULONG watermarkLevel = AUD2HTX_DMA_BURST_SIZE * AUD2HTX_SAMPLE_WIDTH;
    
    status = m_pDmaAdapter->DmaOperations->ConfigureAdapterChannel(m_pDmaAdapter, SDMA_CFG_FUN_SET_CHANNEL_WATERMARK_LEVEL, &watermarkLevel);
    NT_ASSERT(NT_SUCCESS(status));
    
    status = m_pDmaAdapter->DmaOperations->ConfigureAdapterChannel(m_pDmaAdapter, SDMA_CFG_FUN_ACQUIRE_REQUEST_LINE, &m_RequestLine);
    NT_ASSERT(NT_SUCCESS(status));
    
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
NTSTATUS
CSoc_htx::UnregisterStream
(
    _In_        CMiniportWaveRTStream* Stream,
                eDeviceType DeviceType
)
{
    NTSTATUS status;
    KIRQL oldIRQL;

    UNREFERENCED_PARAMETER(DeviceType);
    UNREFERENCED_PARAMETER(Stream);
    DBG_DRV_METHOD_BEG();
    
    m_pRtStream = NULL;
    m_ulSamplesTransferred = 0;
    m_pWfExt = NULL;
     
    status = m_pDmaAdapter->DmaOperations->CancelMappedTransfer(m_pDmaAdapter, m_DmaTransferContext);
    NT_ASSERT(NT_SUCCESS(status));

    status = m_pDmaAdapter->DmaOperations->FlushAdapterBuffersEx(m_pDmaAdapter, m_BufMdl, m_MapRegisterBase, 0, m_BufSize, true);
    NT_ASSERT(NT_SUCCESS(status));

    KeRaiseIrql(DISPATCH_LEVEL, &oldIRQL);
    m_pDmaAdapter->DmaOperations->FreeAdapterChannel(m_pDmaAdapter);
    KeLowerIrql(oldIRQL);

    m_bIsRenderActive = FALSE;

    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg("PAGE")
NTSTATUS
CSoc_htx::Setup()
{
    PAGED_CODE();
    AUD2HTX_IRQ_MASK mask;
    AUD2HTX_CTRL_EXT ControlExt;
    DBG_DRV_METHOD_BEG();
    TxSoftwareReset();

    // Configure Watermarks for DMA
    ControlExt.AsUlong = 0U;
    ControlExt.WaterLow = AUD2HTX_DMA_WATER_LOW;
    ControlExt.WaterHigh = AUD2HTX_FIFO_SIZE - 1; // Not used
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong, ControlExt.AsUlong);
    // Disable all interrupts
    mask.AsUlong = 0U;
    mask.Overflow = 1U;
    mask.WaterHigh = 1U;
    mask.WaterLow = 1U;
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->IrqMasks.AsUlong, mask.AsUlong);
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
VOID
CSoc_htx::TxSoftwareReset()
{
    /* Disable to do SW reset */
    DBG_DRV_METHOD_BEG();
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong, READ_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong) & (~0x1)); //Disable DMA
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->Control.AsUlong, 0U);   
    DBG_DRV_METHOD_END();
}

#pragma code_seg()
NTSTATUS
CSoc_htx::StartDma
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    NTSTATUS status = STATUS_SUCCESS;
    ULONG DmaLength;

    DBG_DRV_METHOD_BEG();
   
    NT_ASSERT(m_BufMdl);
    NT_ASSERT(m_BufSize);

    UNREFERENCED_PARAMETER(Stream);
    m_IsrCnt = 0;
    
    if (!m_bIsRenderActive) {
        DmaLength = m_BufSize;
        status = m_pDmaAdapter->DmaOperations->MapTransferEx(m_pDmaAdapter, m_BufMdl, m_MapRegisterBase, 0, 0, &DmaLength, true, NULL, 0, CompletionRoutine, this);
        NT_ASSERT(NT_SUCCESS(status));
        NT_ASSERT(DmaLength == m_BufSize);
    }
    m_bIsRenderActive = TRUE;
    m_bIsRenderPaused = FALSE;
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->Control.AsUlong, 1U); // Enable
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong, READ_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong) | 0x1 ); //Enable DMA
#if DBG    
    DumpRegs();
#endif
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
NTSTATUS
CSoc_htx::StopDma
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    DBG_DRV_METHOD_BEG();

    UNREFERENCED_PARAMETER(Stream);

    if (m_bIsRenderActive) {
        NT_ASSERT(m_bIsRenderPaused);
        DBG_DRV_PRINT_VERBOSE(" m_IsrCnt = %u, m_ulSamplesTransferred = %u", m_IsrCnt, m_ulSamplesTransferred);
    }
    
    TxSoftwareReset();
    m_bIsRenderActive = FALSE;
    m_bIsRenderPaused = FALSE;

    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->NonMaskedIntFlags.AsUlong, READ_REGISTER_ULONG(&m_pAud2HtxRegisters->NonMaskedIntFlags.AsUlong));
    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
NTSTATUS
CSoc_htx::PauseDma
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    DBG_DRV_METHOD_BEG();
    UNREFERENCED_PARAMETER(Stream);

    m_bIsRenderPaused = TRUE;
    WRITE_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong, READ_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong) & (~0x1)); //DMA

    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}

#pragma code_seg()
void CompletionRoutine(
    PDMA_ADAPTER DmaAdapter,
    PDEVICE_OBJECT DeviceObject,
    PVOID CompletionContext,
    DMA_COMPLETION_STATUS Status
)
{
    UNREFERENCED_PARAMETER(DmaAdapter);
    UNREFERENCED_PARAMETER(DeviceObject);
    UNREFERENCED_PARAMETER(CompletionContext);

    NT_ASSERT(CompletionContext);

    CSoc_htx* htx = (CSoc_htx*)CompletionContext;
    
    htx->Notify(Status);
};

VOID CSoc_htx::Notify(DMA_COMPLETION_STATUS Status)
{
    if (Status == DmaComplete) {
        m_IsrCnt++;
        if (m_pRtStream) {
            m_ulSamplesTransferred += (m_NotificationBytes / m_pWfExt->Format.nBlockAlign);
            m_pRtStream->UpdateVirtualPositionRegisters(m_ulSamplesTransferred);
        }
    }
}

