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

#include "imx_audio.h"
#include "common.h"
#include "imx_sairegs.h"
#include "minwavertstream.h"
#include "sai_acpi_dsd.h"
#include "dma.h"

#define SOC_MAX_BUFFER_NUMBER 2

#if DBG
#define DBG_VERBOSITY 1
#define DBG_MSG_DRV_PREFIX "IMXAUD2"
#if DBG_VERBOSITY > 1
    // Be careful with this verbosity settings. It impacts driver performance.
    #define DBG_DRV_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:+++%s()\n"                    ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__)
    #define DBG_DRV_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:+++%s("_format_str_")\n"      ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:---%s()\n"                    ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__)
    #define DBG_DRV_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:---%s("_format_str_")\n"      ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:---%s() [0x%.8X]\n"           ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,_status_)
    #define DBG_DRV_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:   %s "_format_str_"\n"       ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#else
    #define DBG_DRV_METHOD_BEG()                                 
    #define DBG_DRV_METHOD_BEG_WITH_PARAMS(_format_str_,...)     
    #define DBG_DRV_METHOD_END()                                 
    #define DBG_DRV_METHOD_END_WITH_PARAMS(_format_str_,...)     
    #define DBG_DRV_METHOD_END_WITH_STATUS(_status_)      
    #define DBG_DRV_PRINT_VERBOSE(_format_str_,...)              
#endif
#define DBG_DRV_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:   %s "_format_str_"\n"       ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_DRV_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"D%d %s:   %s "_format_str_"\n"       ,KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#else
#define DBG_DRV_METHOD_BEG()                                 
#define DBG_DRV_METHOD_BEG_WITH_PARAMS(_format_str_,...)     
#define DBG_DRV_METHOD_END()                                 
#define DBG_DRV_METHOD_END_WITH_PARAMS(_format_str_,...)     
#define DBG_DRV_METHOD_END_WITH_STATUS(_status_)             
#define DBG_DRV_PRINT_WARNING(_format_str_,...)              
#define DBG_DRV_PRINT_VERBOSE(_format_str_,...)              
#define DBG_DRV_PRINT_INFO(_format_str_,...)                 
#endif

class CSoc
{
public:
    CSoc();
    ~CSoc();

public:
    NTSTATUS InitSaiBlock
    (
            PCM_PARTIAL_RESOURCE_DESCRIPTOR registersDescriptor, PCM_PARTIAL_RESOURCE_DESCRIPTOR interruptDescriptor,
            PCM_PARTIAL_RESOURCE_DESCRIPTOR rxDmaResourcePtr, PCM_PARTIAL_RESOURCE_DESCRIPTOR txDmaResourcePtr,
            PDEVICE_OBJECT PDO
    );

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

    NTSTATUS StartDma
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    NTSTATUS PauseDma
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    NTSTATUS StopDma
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    NTSTATUS AllocBuffer
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType                        DeviceType,
        _Inout_     PULONG                 Size,
        _Out_       PMDL* pMdl,
        _Out_       MEMORY_CACHING_TYPE* CacheType
    );

    NTSTATUS FreeBuffer
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType                        DeviceType,
        _In_        PMDL                   Mdl,
        _In_        ULONG                  Size
    );

    BOOL isRenderSupported(VOID)
    {
        return m_Config.TxWordMask != 0xFFFFFFFF ? TRUE : FALSE;
    };
    BOOL isCaptureSupported(VOID)
    {
        return m_Config.RxWordMask != 0xFFFFFFFF ? TRUE : FALSE;
    };

private:

    CDmaBuffer             m_Dma[SOC_MAX_BUFFER_NUMBER];

    volatile BOOLEAN                 m_bIsRenderActive;
    volatile BOOLEAN                 m_bIsCaptureActive;
     
    volatile PUINT8                  m_pSaiRegisters;
    PDEVICE_OBJECT                   m_pPDO;
    SAI_DSD_CONFIG                   m_Config;
    UINT32                           m_FifoSize;
    UINT32                           m_DataLine;
    UINT32                           m_FrameSize;
    UINT32                           m_saiOffset;
    PHYSICAL_ADDRESS                 m_SaiRx;
    PHYSICAL_ADDRESS                 m_SaiTx;
    UINT32                           m_TxWater;
    UINT32                           m_RxWater;

    ULONG                            m_TxDmaRequestLine;
    ULONG                            m_TxDmaChannel;
    ULONG                            m_TxDmaInstance;

    ULONG                            m_RxDmaRequestLine;
    ULONG                            m_RxDmaChannel;
    ULONG                            m_RxDmaInstance;

	VOID CleanUp();
    VOID ResetTxFifo();
    VOID ResetRxFifo();
    VOID TxStart();
    VOID TxStop();
    VOID RxStart();
    VOID RxStop();
    VOID TxSoftwareReset();
    VOID RxSoftwareReset();

    VOID WriteRegister(ULONG32 reg_offset, ULONG32 value)
    {
        WRITE_REGISTER_NOFENCE_ULONG((volatile ULONG*)(m_pSaiRegisters + reg_offset), value);
    }

    ULONG ReadRegister(ULONG32 reg_offset)
    {
        return READ_REGISTER_NOFENCE_ULONG((volatile ULONG*)(m_pSaiRegisters + reg_offset));
    }
};

