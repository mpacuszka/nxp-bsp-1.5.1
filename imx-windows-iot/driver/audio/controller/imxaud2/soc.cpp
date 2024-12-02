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
#include <ImxCpuRev.h>
extern "C" {
#include <acpiioct.h>
#include <initguid.h>
} // extern "C"
#include "acpiutil.hpp"

// Include wave tables as we need to know audio constrains due to many limitations in this Adapter implementation.
#include "micinwavtable.h"
#include "speakerhpwavtable.h"

#if SPEAKERHP_HOST_MIN_SAMPLE_RATE != SPEAKERHP_HOST_MAX_SAMPLE_RATE
    #error "Unsupported Speaker sample rates. Only fixed rate supported!"
#endif
#if MICIN_MIN_SAMPLE_RATE != MICIN_MAX_SAMPLE_RATE
    #error "Unsupported MicIn sample rates. Only fixed rate supported!"
#endif
#if SPEAKERHP_HOST_MIN_SAMPLE_RATE != MICIN_MIN_SAMPLE_RATE
    #error "Unsupported configuration. Speaker and MicIn sample rate must match!"
#endif

#if SPEAKERHP_HOST_MIN_BITS_PER_SAMPLE != SPEAKERHP_HOST_MAX_BITS_PER_SAMPLE
    #error "Unsupported configuration. Max and Min bits per sample must match!"
#endif

#if MICIN_MIN_BITS_PER_SAMPLE != MICIN_MAX_BITS_PER_SAMPLE
    #error "Unsupported configuration. Max and Min bits per sample must match!"
#endif

#define RX_BITS_PER_SAMPLE MICIN_MIN_BITS_PER_SAMPLE
#define TX_BITS_PER_SAMPLE SPEAKERHP_HOST_MIN_BITS_PER_SAMPLE

#if RX_BITS_PER_SAMPLE < 24
    #warning "Unverified sample size. Be careful and align FrameSize."
#endif
#if TX_BITS_PER_SAMPLE < 24
    #warning "Unverified sample size. Be careful and align FrameSize."
#endif

#define RX_SAMPLE_RATE     MICIN_MIN_SAMPLE_RATE
#define TX_SAMPLE_RATE     SPEAKERHP_HOST_MIN_SAMPLE_RATE
#define TX_CHANNELS        SPEAKERHP_DEVICE_MAX_CHANNELS
#define RX_CHANNELS        MICIN_DEVICE_MAX_CHANNELS

#define SAI_CALC_DIV(mclk, sampleRate, slotSize, numSlots) \
        (((mclk) / ((sampleRate) * (slotSize) * (numSlots)) / 2) - 1)

#pragma code_seg()

VOID CSoc::ResetTxFifo()
{
    SAI_TCSR tcsr;
    DBG_DRV_METHOD_BEG();

    tcsr.AsUlong = ReadRegister(SAI_TCSR_OFFSET);
    tcsr.FifoReset = 1;
    WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
    // TODO this is needed for multi-channel support
#if 0
    if (m_Config.Version == mx8m) {
        SAI_TCR3 tcr3;
        tcr3.AsUlong = ReadRegister(SAI_TCR3_OFFSET);
        tcr3.ChannelFifoReset |= 1;
        WriteRegister(SAI_TCR3_OFFSET, tcr3.AsUlong);
    }
#endif
    DBG_DRV_METHOD_END();
}

VOID
CSoc::ResetRxFifo()
{
    SAI_RCSR rcsr;
    DBG_DRV_METHOD_BEG();

    rcsr.AsUlong = ReadRegister(SAI_RCSR_OFFSET);
    rcsr.FifoReset = 1;
    WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
    // TODO this is needed for multi-channel support
#if 0
    if (m_Config.Version == mx8m) {
        SAI_RCR3 rcr3;
        rcr3.AsUlong = ReadRegister(SAI_RCR3_OFFSET);
        rcr3.ChannelFifoReset |= 1;
        WriteRegister(SAI_RCR3_OFFSET, rcr3.AsUlong);
    }
#endif
    DBG_DRV_METHOD_END();
}

CSoc::CSoc()
{
	m_pSaiRegisters = NULL;

    m_bIsRenderActive = false;
    m_bIsCaptureActive = false;
    m_pPDO = NULL;
    m_FifoSize = 0;
    m_DataLine = 0;
    m_FrameSize = 0;
    m_saiOffset = 0;
    m_SaiRx.QuadPart = 0;
    m_SaiTx.QuadPart = 0;
    m_TxWater = 0;
    m_RxWater = 0;
    
    m_TxDmaRequestLine = 0;
    m_TxDmaChannel = 0;
    m_TxDmaInstance = 0;
    
    m_RxDmaRequestLine = 0;
    m_RxDmaChannel = 0;
    m_RxDmaInstance = 0;

    // Make defaults
    m_Config.Mode = master;
    m_Config.TxMsel = 1;
    m_Config.RxMsel = 1;
    m_Config.TxMclk = 11289600;
    m_Config.RxMclk = 11289600;
    m_Config.Synch = async;
    m_Config.TxFrameSize = 32;
    m_Config.RxFrameSize = 32;
    m_Config.Protocol = I2S;
    m_Config.TxWordMask = (UINT32)~3; // unmask 2 channels
    m_Config.RxWordMask = (UINT32)~3;
    //Optional params
    m_Config.OptTxBcklPol = bclkNotSet;
    m_Config.OptRxBcklPol = bclkNotSet;
    m_Config.OptRxEarlyFrameSync = -1;
    m_Config.OptTxEarlyFrameSync = -1;
    m_Config.OptTxFrameSyncPol = fsyncNotSet;
    m_Config.OptRxFrameSyncPol = fsyncNotSet;
    m_Config.OptTxSyncWidth = -1;
    m_Config.OptRxSyncWidth = -1;
    m_Config.OptTxMsbFirst = -1;
    m_Config.OptRxMsbFirst = -1;
    // Note this is not equal to channel count defined in wavtables as it defines physical interface setup
    // There can be many samples in frame and we can mask unwanted.
    m_Config.OptRxSampleCount = 2;
    m_Config.OptTxSampleCount = 2;
}

CSoc::~CSoc()
{
	CleanUp();
}

VOID
CSoc::CleanUp()
{
    DBG_DRV_METHOD_BEG();
	if (m_pSaiRegisters != NULL)
	{

        /* Mask all slots */
        WriteRegister(SAI_TMR_OFFSET, 0xFFFFFFFF);
        WriteRegister(SAI_RMR_OFFSET, 0xFFFFFFFF);

        /* Reset regs */
        WriteRegister(SAI_TCSR_OFFSET, 0);
        WriteRegister(SAI_TCR1_OFFSET, 0);
        WriteRegister(SAI_TCR2_OFFSET, 0);
        WriteRegister(SAI_TCR3_OFFSET, 0);
        WriteRegister(SAI_TCR4_OFFSET, 0);

        WriteRegister(SAI_RCSR_OFFSET, 0);
        WriteRegister(SAI_RCR1_OFFSET, 0);
        WriteRegister(SAI_RCR2_OFFSET, 0);
        WriteRegister(SAI_RCR3_OFFSET, 0);
        WriteRegister(SAI_RCR4_OFFSET, 0);

		MmUnmapIoSpace(m_pSaiRegisters, IMX_SAI_MMAP_SIZE);
		m_pSaiRegisters = NULL;
	}
    DBG_DRV_METHOD_END();
}

NTSTATUS CSoc::InitSaiBlock(PCM_PARTIAL_RESOURCE_DESCRIPTOR registersDescriptor, PCM_PARTIAL_RESOURCE_DESCRIPTOR interruptDescriptor, 
                            PCM_PARTIAL_RESOURCE_DESCRIPTOR rxDmaResourcePtr, PCM_PARTIAL_RESOURCE_DESCRIPTOR txDmaResourcePtr,
                            PDEVICE_OBJECT PDO)
{
    NTSTATUS ntStatus = STATUS_SUCCESS;
    ACPI_EVAL_OUTPUT_BUFFER* DsdBufferPptr;
    ACPI_METHOD_ARGUMENT UNALIGNED const* DevicePropertiesPkgPptr;
    UINT32 i;

    DBG_DRV_METHOD_BEG();
    UNREFERENCED_PARAMETER(interruptDescriptor);
    ASSERT(rxDmaResourcePtr);
    ASSERT(txDmaResourcePtr);

    DBG_DRV_PRINT_INFO("Rx dma chnl %u req %u", rxDmaResourcePtr->u.DmaV3.Channel, rxDmaResourcePtr->u.DmaV3.RequestLine);
    DBG_DRV_PRINT_INFO("Tx dma chnl %u req %u", txDmaResourcePtr->u.DmaV3.Channel, txDmaResourcePtr->u.DmaV3.RequestLine);

    // Restore channel number and SDMA instance from ACPI channel number.
    m_RxDmaChannel = rxDmaResourcePtr->u.DmaV3.Channel & 0x7F;
    m_RxDmaInstance = (rxDmaResourcePtr->u.DmaV3.Channel >> 7) & 0x3;
    m_RxDmaRequestLine = rxDmaResourcePtr->u.DmaV3.RequestLine;
    m_TxDmaChannel = txDmaResourcePtr->u.DmaV3.Channel & 0x7F;
    m_TxDmaInstance = (txDmaResourcePtr->u.DmaV3.Channel >> 7) & 0x3;
    m_TxDmaRequestLine = txDmaResourcePtr->u.DmaV3.RequestLine;
    // Load aditional information from ACPI
    ntStatus = AcpiQueryDsd(PDO, &DsdBufferPptr);
    if (ntStatus != STATUS_SUCCESS) {
        return ntStatus;
    }
    ntStatus = AcpiParseDsdAsDeviceProperties(DsdBufferPptr, &DevicePropertiesPkgPptr);
    if (ntStatus != STATUS_SUCCESS) {
        return ntStatus;
    }
    for (i = 0; i < SAI_DSD_PROPERTY_MAX; i++) {
        ntStatus = AcpiDevicePropertiesQueryIntegerValue(DevicePropertiesPkgPptr, sai_dsd_property[i], &m_Config.dsd[i]);
        if (ntStatus == STATUS_SUCCESS) {
            DBG_DRV_PRINT_INFO("ACPI DSD found %s = %u", sai_dsd_property[i], m_Config.dsd[i]);
        }
    }
    // Deallocate DsdBufferPptr - was allocated by AcpiParseDsdAsDeviceProperties()
    ExFreePoolWithTag(DsdBufferPptr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
    // Continue with SUCCESS as the ACPI can return fail
    ntStatus = STATUS_SUCCESS;

    ASSERT(m_pSaiRegisters == NULL);
    m_pSaiRegisters = (PUINT8)MmMapIoSpaceEx(registersDescriptor->u.Memory.Start, IMX_SAI_MMAP_SIZE,
            PAGE_READWRITE | PAGE_NOCACHE);
    ASSERT(m_pSaiRegisters);
    if (m_pSaiRegisters == NULL) {
        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        goto Done;
    }

    m_pPDO = PDO;

    if (m_Config.Protocol == I2S) {
        // Configure params
        if (m_Config.OptTxBcklPol == bclkNotSet) {
            m_Config.OptTxBcklPol = bclkLow;
        }            
        if (m_Config.OptRxBcklPol == bclkNotSet) {
            m_Config.OptRxBcklPol = bclkLow;
        }            
        if (m_Config.OptRxEarlyFrameSync == -1) {
            m_Config.OptRxEarlyFrameSync = 1;
        }            
        if (m_Config.OptTxEarlyFrameSync == -1) {
            m_Config.OptTxEarlyFrameSync = 1;
        }            
        if (m_Config.OptRxMsbFirst == -1) {
            m_Config.OptRxMsbFirst = 1;
        }            
        if (m_Config.OptTxMsbFirst == -1) {
            m_Config.OptTxMsbFirst = 1;
        }            
        if (m_Config.OptRxSyncWidth == -1) {
            m_Config.OptRxSyncWidth = m_Config.RxFrameSize;
        }            
        if (m_Config.OptTxSyncWidth == -1) {
            m_Config.OptTxSyncWidth = m_Config.TxFrameSize;
}                    
        if (m_Config.OptTxFrameSyncPol == fsyncNotSet) {
            m_Config.OptTxFrameSyncPol = fsyncLow;
        }            
        if (m_Config.OptRxFrameSyncPol == fsyncNotSet) {
            m_Config.OptRxFrameSyncPol = fsyncLow;
        }
    }
    else {
        DBG_DRV_PRINT_WARNING("Unsupported protocol type. Only I2S is supported");
        ntStatus = STATUS_NOT_SUPPORTED;
        goto Done;
    }
    /* Determine params for peripherals where PARAM register is not supported */
    switch (m_Config.Version) {
    case mx7:
        WriteRegister(SAI_TCR1_OFFSET, IMX_SAI_FIFO_SIZE_MAX);
        m_FifoSize = ReadRegister(SAI_TCR1_OFFSET) + 1;
        if (m_FifoSize > IMX_SAI_FIFO_SIZE_MAX) {
            DBG_DRV_PRINT_WARNING("Not possible to determine FIFO size");
        }
        m_DataLine = 1;
        m_FrameSize = 32;
        break;
    case mx8m:
        m_saiOffset = 0x8;
        SAI_PARAM_REGISTER Param;
        Param.AsUlong = ReadRegister(SAI_PARAM_OFFSET);
        m_FifoSize = 1 << Param.FifoSize;
        m_DataLine = Param.DataLines;
        m_FrameSize = 1 << Param.FrameSize;
        break;
    default:
        DBG_DRV_PRINT_WARNING("Wrong ACPI version parameter: %u", m_Config.Version);
        ntStatus = STATUS_NOT_SUPPORTED;
        goto Done;
    }
    DBG_DRV_PRINT_INFO("SAI frameSize = %u", m_FrameSize);
    DBG_DRV_PRINT_INFO("SAI fifoSize = %u", m_FifoSize);
    DBG_DRV_PRINT_INFO("SAI dataLine = %u", m_DataLine);

    /* Do software and FIFO reset */
    SAI_TCSR tcsr, rcsr;
    tcsr.AsUlong = 0;
    tcsr.FifoReset = 1;
    tcsr.SoftwareReset = 1;
    rcsr.AsUlong = tcsr.AsUlong;
    WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
    WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);

    if (m_FifoSize >= 16) {
        // TODO: Better watermark optimization
        m_TxWater = m_FifoSize >> 1;
        m_RxWater = m_FifoSize >> 1;
    }
    else {
        DBG_DRV_PRINT_WARNING("Unexpected FIFO size!");
        ntStatus = STATUS_NOT_SUPPORTED;
        goto Done;
    }

    /* Reset regs */
    WriteRegister(SAI_TCSR_OFFSET, 0);
    WriteRegister(SAI_TCR1_OFFSET, m_TxWater);
    WriteRegister(SAI_TCR2_OFFSET, 0);
    WriteRegister(SAI_TCR3_OFFSET, 0);
    WriteRegister(SAI_TCR4_OFFSET, 0);

    WriteRegister(SAI_RCSR_OFFSET, 0);
    WriteRegister(SAI_RCR1_OFFSET, m_RxWater - 1);
    WriteRegister(SAI_RCR2_OFFSET, 0);
    WriteRegister(SAI_RCR3_OFFSET, 0);
    WriteRegister(SAI_RCR4_OFFSET, 0);

    /* Mask all slots */
    WriteRegister(SAI_TMR_OFFSET, 0xFFFFFFFF);
    WriteRegister(SAI_RMR_OFFSET, 0xFFFFFFFF);

    /* Configure synchronous/asynchronous mode */
    SAI_RCR2 rcr2;
    SAI_TCR2 tcr2;

    rcr2.AsUlong = 0;
    tcr2.AsUlong = 0;

    switch (m_Config.Synch) {
    case async:        
        tcr2.BitClockDivide = SAI_CALC_DIV(m_Config.TxMclk, TX_SAMPLE_RATE, m_Config.TxFrameSize, m_Config.OptTxSampleCount);
        rcr2.BitClockDivide = SAI_CALC_DIV(m_Config.RxMclk, RX_SAMPLE_RATE, m_Config.RxFrameSize, m_Config.OptRxSampleCount);
        break;
    case syncRx2Tx:
        rcr2.BitClockDivide = tcr2.BitClockDivide = SAI_CALC_DIV(m_Config.TxMclk, TX_SAMPLE_RATE, m_Config.TxFrameSize, m_Config.OptTxSampleCount);
        rcr2.SynchronousMode = 1;
        tcr2.SynchronousMode = 0;
        break;
    case syncTx2Rx:
        rcr2.BitClockDivide = tcr2.BitClockDivide = SAI_CALC_DIV(m_Config.RxMclk, RX_SAMPLE_RATE, m_Config.RxFrameSize, m_Config.OptRxSampleCount);
        tcr2.SynchronousMode = 1;
        rcr2.SynchronousMode = 0;
        break;
    default:
        DBG_DRV_PRINT_WARNING("Wrong ACPI SYNCH parameter: %u", m_Config.Synch);
        break;
    }

    /* Configure master clock select */
    if (m_Config.TxMsel < 4) {
        tcr2.MasterClockSelect = m_Config.TxMsel;
    }
    else {
        DBG_DRV_PRINT_WARNING("Wrong ACPI TX_MSEL parameter: %u", m_Config.TxMsel);
    }
    if (m_Config.RxMsel < 4) {
        rcr2.MasterClockSelect = m_Config.RxMsel;
    }
    else {
        DBG_DRV_PRINT_WARNING("Wrong ACPI RX_MSEL parameter: %u", m_Config.RxMsel);
    }
    /* Configure bit clock polarity */
    tcr2.BitClockPolarity = m_Config.OptTxBcklPol;
    rcr2.BitClockPolarity = m_Config.OptRxBcklPol;
    /* Configure bit clock direction */
    if (m_Config.Mode == master) {
        tcr2.BitClockDirection = 1;
        rcr2.BitClockDirection = 1;
    }

    WriteRegister(SAI_TCR2_OFFSET, tcr2.AsUlong);
    WriteRegister(SAI_RCR2_OFFSET, rcr2.AsUlong);

    SAI_TCR3 tcr3;
    SAI_RCR3 rcr3;

    tcr3.AsUlong = ReadRegister(SAI_TCR3_OFFSET);
    tcr3.ChannelEnable = 1;
    WriteRegister(SAI_TCR3_OFFSET, tcr3.AsUlong);

    rcr3.AsUlong = ReadRegister(SAI_RCR3_OFFSET);
    rcr3.ChannelEnable = 1;
    WriteRegister(SAI_RCR3_OFFSET, rcr3.AsUlong);

    SAI_RCR4 rcr4;
    SAI_TCR4 tcr4;

    rcr4.AsUlong = 0;
    tcr4.AsUlong = 0;

    NT_ASSERT(m_Config.OptRxSampleCount >= RX_CHANNELS);
    NT_ASSERT(m_Config.OptTxSampleCount >= TX_CHANNELS);
    // SampleCount refers to physical layer - number of samples in each frame. 
    // It may differ from number of channels processed as we can mask samples in every frame using TMR/RMR regs.
    if ((m_Config.OptRxSampleCount > 0) && (m_Config.OptRxSampleCount <= 32)) {
        rcr4.FrameSize = m_Config.OptRxSampleCount - 1;
    }
    else {
        DBG_DRV_PRINT_WARNING("Wrong ACPI OPT_RXSAMPLE_COUNT parameter: %i", m_Config.OptRxSampleCount);
    }
    if ((m_Config.OptTxSampleCount > 0) && (m_Config.OptTxSampleCount <= 32)) {
        tcr4.FrameSize = m_Config.OptTxSampleCount - 1;
    }
    else {
        DBG_DRV_PRINT_WARNING("Wrong ACPI OPT_TXSAMPLE_COUNT parameter: %i", m_Config.OptTxSampleCount);
    }
    if ((m_Config.OptRxSyncWidth > 0) && (m_Config.OptRxSyncWidth <= (INT32)m_Config.RxFrameSize)) {
        rcr4.SyncWidth = m_Config.OptRxSyncWidth - 1;
    }
    else {
        DBG_DRV_PRINT_WARNING("Wrong ACPI OPT_RXSYNC_WIDTH parameter: %i", m_Config.OptRxSyncWidth);
    }
    if ((m_Config.OptTxSyncWidth > 0) && (m_Config.OptTxSyncWidth <= (INT32)m_Config.TxFrameSize)) {
        tcr4.SyncWidth = m_Config.OptTxSyncWidth - 1;
    }
    else {
        DBG_DRV_PRINT_WARNING("Wrong ACPI OPT_TXSYNC_WIDTH parameter: %i", m_Config.OptTxSyncWidth);
    }

    if (m_Config.OptTxMsbFirst) {
        tcr4.MSBFirst = 1;
    }
    if (m_Config.OptRxMsbFirst) {
        rcr4.MSBFirst = 1;
    }

    if (m_Config.OptRxEarlyFrameSync) {
        rcr4.FrameSyncEarly = 1;
    }
    if (m_Config.OptTxEarlyFrameSync) {
        tcr4.FrameSyncEarly = 1;
    }

    switch (m_Config.OptRxFrameSyncPol) {
    case fsyncHigh:
    case fsyncLow:
        rcr4.FrameSyncPolarity = m_Config.OptRxFrameSyncPol;
        break;
    default:
        DBG_DRV_PRINT_WARNING("Wrong ACPI OPT_RXFSYNC_POL parameter: %i", m_Config.OptRxFrameSyncPol);
        break;
    }
    switch (m_Config.OptTxFrameSyncPol) {
    case fsyncHigh:
    case fsyncLow:
        tcr4.FrameSyncPolarity = m_Config.OptTxFrameSyncPol;
        break;
    default:
        DBG_DRV_PRINT_WARNING("Wrong ACPI OPT_TXFSYNC_POL parameter: %i", m_Config.OptTxFrameSyncPol);
        break;
    }
    /* Generate frame sync internally */
    if (m_Config.Mode == master) {
        tcr4.FrameSyncDirection = 1;
        rcr4.FrameSyncDirection = 1;
    }

    if ((m_Config.Version != mx7) && (m_Config.Protocol == I2S)) {
        /* Transmit data pins are never tristated when slots are masked or channels are disabled */
        tcr4.ChannelMode = 1;
    }
    
    tcr4.FifoContinueOnErr = 1;
    rcr4.FifoContinueOnErr = 1;

    WriteRegister(SAI_TCR4_OFFSET, tcr4.AsUlong);
    WriteRegister(SAI_RCR4_OFFSET, rcr4.AsUlong);

    SAI_RCR5 rcr5;
    SAI_TCR5 tcr5;

    rcr5.AsUlong = 0;
    tcr5.AsUlong = 0;
    // We calculate word width based on FrameSize. Windows 24bit format expects 24th bit on 31th position in frame
    // thus settings is same like for 32bit sample. 
    // In case of 16bit format we expect FrameSize 16bit thus also correct position of 16th bit in 16th slot bit.
    tcr5.Word0Width = tcr5.WordNWidth = m_Config.TxFrameSize - 1;
    if (m_Config.OptTxMsbFirst) {
       tcr5.FirstBitShifted = m_Config.TxFrameSize - 1; 
    }
    else {
        // This is experimental as it may depend on slot size and sample we want to read/write.
        // eg. reading 32bit value like a 16bit sample (lowering DR/SNR).
        tcr5.FirstBitShifted = 0;
    }

    rcr5.Word0Width = rcr5.WordNWidth = m_Config.RxFrameSize - 1;
    if (m_Config.OptRxMsbFirst) {
        rcr5.FirstBitShifted = m_Config.RxFrameSize - 1;            
    }
    else {
        // This is experimental as it may depend on slot size and sample we want to read/write. 
        tcr5.FirstBitShifted = 0;
    }

    WriteRegister(SAI_TCR5_OFFSET, tcr5.AsUlong);
    WriteRegister(SAI_RCR5_OFFSET, rcr5.AsUlong);

    SAI_TMR tmr;
    tmr.AsUlong = 0;
    tmr.WordMask = m_Config.TxWordMask;

    WriteRegister(SAI_TMR_OFFSET, tmr.AsUlong);

    SAI_RMR rmr;
    rmr.AsUlong = 0;
    rmr.WordMask = m_Config.RxWordMask;

    WriteRegister(SAI_RMR_OFFSET, rmr.AsUlong);
    
    // Prepare TX/RX FIFO phy addresses for DMA
    m_SaiRx.QuadPart = registersDescriptor->u.Memory.Start.QuadPart + SAI_RXn_OFFSET(0);
    m_SaiTx.QuadPart = registersDescriptor->u.Memory.Start.QuadPart + SAI_TXn_OFFSET(0);
    
    DBG_DRV_METHOD_END();
Done:
    // Cleanup
    if (ntStatus != STATUS_SUCCESS)
    {
        CleanUp();
    }
    
    return ntStatus;
}

NTSTATUS
CSoc::RegisterStream
(
    _In_        CMiniportWaveRTStream* Stream,
                eDeviceType DeviceType
)
{
    DBG_DRV_METHOD_BEG();
    NT_ASSERT(DeviceType < SOC_MAX_BUFFER_NUMBER);

    switch (DeviceType) {
    case eSpeakerHpDevice:
    case eMicInDevice:
        return m_Dma[DeviceType].RegisterStream(Stream, DeviceType);
    default: 
        return STATUS_NOT_SUPPORTED;
    }
}

NTSTATUS
CSoc::UnregisterStream
(
    _In_        CMiniportWaveRTStream* Stream,
                eDeviceType DeviceType
)
{
    DBG_DRV_METHOD_BEG();
    NT_ASSERT(DeviceType < SOC_MAX_BUFFER_NUMBER);
    switch (DeviceType) {
    case eSpeakerHpDevice:
    case eMicInDevice:
        m_Dma[DeviceType].UnregisterStream(Stream, DeviceType);
    default:
        return STATUS_NOT_SUPPORTED;
    }
}

VOID
CSoc::TxSoftwareReset()
{
    SAI_TCSR tcsr;
    DBG_DRV_METHOD_BEG();
    tcsr.AsUlong = ReadRegister(SAI_TCSR_OFFSET);

    tcsr.SoftwareReset = 1;
    WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);

    tcsr.SoftwareReset = 0;
    WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
    DBG_DRV_METHOD_END();
}

VOID
CSoc::RxSoftwareReset()
{
    SAI_RCSR rcsr;
    DBG_DRV_METHOD_BEG();
    rcsr.AsUlong = ReadRegister(SAI_RCSR_OFFSET);;

    rcsr.SoftwareReset = 1;
    WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);

    rcsr.SoftwareReset = 0;
    WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
    DBG_DRV_METHOD_END();
}

VOID
CSoc::TxStart()
{
    SAI_TCSR tcsr;
    SAI_RCSR rcsr;
    DBG_DRV_METHOD_BEG();

    if (!(m_bIsCaptureActive && (m_Config.Synch == syncRx2Tx)))
    {
        // We can do sw reset in case RX is not running using Tx clock
        TxSoftwareReset();
        ResetTxFifo();
    }

    tcsr.AsUlong = ReadRegister(SAI_TCSR_OFFSET);
    tcsr.Enable = 1;
    tcsr.FifoRequestDMAEnable = 1;

    switch (m_Config.Synch) {
    case async:
    case syncRx2Tx:
        WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        break;
    case syncTx2Rx:
        WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        if (!m_bIsCaptureActive) {
            DBG_DRV_PRINT_INFO("Capture not active, setting RX");
            // Tx is synchronous on Rx, so Rx must be enabled
            ResetRxFifo();
            rcsr.AsUlong = ReadRegister(SAI_RCSR_OFFSET);
            rcsr.Enable = 1;
            WriteRegister(SAI_RCSR_OFFSET, tcsr.AsUlong);
        }
        break;
    }
    DBG_DRV_METHOD_END();
}

VOID
CSoc::RxStart()
{
    SAI_RCSR rcsr;
    SAI_TCSR tcsr;
    DBG_DRV_METHOD_BEG();

    if (!(m_bIsRenderActive && (m_Config.Synch == syncTx2Rx)))
    {
        // We can do sw reset in case TX is not running using Rx clock
        RxSoftwareReset();
    }
    ResetRxFifo();

    rcsr.AsUlong = ReadRegister(SAI_RCSR_OFFSET);
    rcsr.Enable = 1;
    rcsr.FifoRequestDMAEnable = 1;

    switch (m_Config.Synch) {
    case async:
    case syncTx2Rx:
        WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
        break;
    case syncRx2Tx:
        WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
        if (!m_bIsRenderActive) {
            DBG_DRV_PRINT_INFO("Render not active, setting TX");
            ResetTxFifo();
            // Rx is synchronous on Tx, so Tx must be enabled
            tcsr.AsUlong = ReadRegister(SAI_TCSR_OFFSET);
            tcsr.Enable = 1;
            WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        }
        break;
    }
    DBG_DRV_METHOD_END();
}

VOID
CSoc::TxStop()
{
    SAI_RCSR rcsr;
    SAI_TCSR tcsr;
    DBG_DRV_METHOD_BEG();

    tcsr.AsUlong = ReadRegister(SAI_TCSR_OFFSET);
    tcsr.FifoRequestDMAEnable = 0;
    WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);    
    tcsr.Enable = 0;
    switch (m_Config.Synch) {
    case async:
        WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        break;
    case syncTx2Rx:
        WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        if (!m_bIsCaptureActive) {
            rcsr.AsUlong = ReadRegister(SAI_RCSR_OFFSET);
            rcsr.Enable = 0;
            WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
        }
        break;
    case syncRx2Tx:
        if (!m_bIsCaptureActive) {
            WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        }
        break;
    }
    DBG_DRV_METHOD_END();
}

VOID
CSoc::RxStop()
{
    SAI_RCSR rcsr;
    SAI_TCSR tcsr;
    DBG_DRV_METHOD_BEG();

    rcsr.AsUlong = ReadRegister(SAI_RCSR_OFFSET);
    rcsr.FifoRequestDMAEnable = 0;
    WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
    rcsr.Enable = 0;
    switch (m_Config.Synch) {
    case async:
        WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
        break;
    case syncRx2Tx:
        WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
        if (!m_bIsRenderActive) {
            tcsr.AsUlong = ReadRegister(SAI_TCSR_OFFSET);
            tcsr.Enable = 0;
            WriteRegister(SAI_TCSR_OFFSET, tcsr.AsUlong);
        }
        break;
    case syncTx2Rx:
        if (!m_bIsRenderActive) {
            WriteRegister(SAI_RCSR_OFFSET, rcsr.AsUlong);
        }
        break;
    }
    DBG_DRV_METHOD_END();
}

NTSTATUS
CSoc::StartDma
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    NTSTATUS Status = STATUS_NOT_SUPPORTED;
    DBG_DRV_METHOD_BEG();

    if (m_Dma[eMicInDevice].IsMyStream(Stream))
    {
        Status = m_Dma[eMicInDevice].Start(Stream);
        if (NT_SUCCESS(Status)) {
            RxStart();
            m_bIsCaptureActive = TRUE;
        }
    }
    else if (m_Dma[eSpeakerHpDevice].IsMyStream(Stream))
    {
        Status = m_Dma[eSpeakerHpDevice].Start(Stream);
        if (NT_SUCCESS(Status)) {
            TxStart();
            m_bIsRenderActive = TRUE;
        }
    }
    DBG_DRV_METHOD_END();
    return Status;
}

NTSTATUS
CSoc::StopDma
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    NTSTATUS Status = STATUS_SUCCESS;
    DBG_DRV_METHOD_BEG();

    if (m_Dma[eMicInDevice].IsMyStream(Stream) && m_bIsCaptureActive)
    {
        RxStop();
        m_bIsCaptureActive = FALSE;
    }
    else if (m_Dma[eSpeakerHpDevice].IsMyStream(Stream) && m_bIsRenderActive)
    {
        TxStop();
        m_bIsRenderActive = FALSE;
    }
    DBG_DRV_METHOD_END();
    return Status;
}

NTSTATUS 
CSoc::AllocBuffer
(
    _In_        CMiniportWaveRTStream* Stream,
    eDeviceType                        DeviceType,
    _Inout_     PULONG                 Size,
    _Out_       PMDL*                  pMdl,
    _Out_       MEMORY_CACHING_TYPE* CacheType
) {
    ULONG Res;
    ULONG RequestedSize;
    NTSTATUS ntStatus = STATUS_NOT_SUPPORTED;
    PWAVEFORMATEXTENSIBLE pWaveFormat;

    NT_ASSERT(DeviceType < SOC_MAX_BUFFER_NUMBER);

    pWaveFormat = Stream->GetDataFormat();

    if (DeviceType == eSpeakerHpDevice) {
        ntStatus = m_Dma[eSpeakerHpDevice].Init(m_pPDO, m_SaiTx, pWaveFormat->Format.wBitsPerSample, m_TxDmaRequestLine | (m_TxDmaInstance << 10), m_TxDmaChannel, m_TxWater);
    }
    else if (DeviceType == eMicInDevice) {
        ntStatus = m_Dma[eMicInDevice].Init(m_pPDO, m_SaiRx, pWaveFormat->Format.wBitsPerSample, m_RxDmaRequestLine | (m_RxDmaInstance << 10), m_RxDmaChannel, m_RxWater);
    }
    NT_ASSERT(ntStatus == STATUS_SUCCESS);
    
    if (!NT_SUCCESS(ntStatus)) {
        /* Failed to get and init DMA object */
        return ntStatus;
    }

    RequestedSize = *Size;
    // Round the buffer for DMA
    Res = RequestedSize % 64U;
    if (Res != 0) {
        RequestedSize -= Res;
        RequestedSize += 64U;
    }
    switch (DeviceType) {
    case eSpeakerHpDevice:
    case eMicInDevice:
        *Size = RequestedSize;
        return m_Dma[DeviceType].AllocBuffer(DeviceType, RequestedSize, pMdl, CacheType);
    default:
        return STATUS_NOT_SUPPORTED;
    }
}

NTSTATUS 
CSoc::FreeBuffer
(
    _In_        CMiniportWaveRTStream* Stream,
    eDeviceType                        DeviceType,
    _In_        PMDL                   Mdl,
    _In_        ULONG                  Size
) {
    (void)Stream;
    NT_ASSERT(DeviceType < SOC_MAX_BUFFER_NUMBER);

    switch (DeviceType) {
    case eSpeakerHpDevice:
    case eMicInDevice:
        return m_Dma[DeviceType].FreeBuffer(DeviceType, Size, Mdl);
    default:
        return STATUS_NOT_SUPPORTED;
    }
}


#pragma code_seg()
NTSTATUS
CSoc::PauseDma
(
    _In_        CMiniportWaveRTStream* Stream
)
{
    NTSTATUS Status = STATUS_NOT_SUPPORTED;
    DBG_DRV_METHOD_BEG();

    if (m_Dma[eMicInDevice].IsMyStream(Stream))
    {
        RxStop();
        m_bIsCaptureActive = FALSE;
        Status = STATUS_SUCCESS;
    }
    else if (m_Dma[eSpeakerHpDevice].IsMyStream(Stream))
    {
        TxStop();
        m_bIsRenderActive = FALSE;
        Status = STATUS_SUCCESS;
    }
    DBG_DRV_METHOD_END();
    return Status;
}
