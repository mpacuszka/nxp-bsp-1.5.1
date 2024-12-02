/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    CSoc class declaration.

*/
#pragma once

#include "imx_audio.h"
#include "socbase.h"
#include "imx_aud2htx.h"

#include <ntddk.h>


#if DBG
#define DBG_MSG_DRV_PREFIX "HDMIAUD"
#define DBG_VERBOSITY 1
#if DBG_VERBOSITY > 1
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

class CSoc_htx;

class CSoc_htx : public CSocBase
{
public:
    CSoc_htx() {
        m_pAud2HtxRegisters = NULL;
        m_RegBase.QuadPart = 0;
        m_pWfExt = NULL;
        m_pRtStream = NULL;
        m_bIsRenderPaused = FALSE;
        m_ulSamplesTransferred = 0;
        m_pDmaAdapter = NULL;
        m_TransferInfo.V1.MapRegisterCount = 0;
        m_TransferInfo.V1.ScatterGatherElementCount = 0;
        m_TransferInfo.V1.ScatterGatherListSize = 0;
        m_BufVirt = NULL;
        m_MapRegisterBase = NULL;
        m_BufMdl = NULL;
        m_BufLogical.QuadPart = 0;
        m_BufSize = 0;
        m_NumberOfMapRegs = 0;
        m_RequestLine = 0;
        m_NotificationBytes = 0;
        m_IsrCnt = 0;
        memset(m_DmaTransferContext, 0, DMA_TRANSFER_CONTEXT_SIZE_V1 / sizeof(UINT));
    };
    ~CSoc_htx() 
    {
        CleanUp();
    }

public:
    virtual NTSTATUS InitBlock
    (
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR registersDescriptor,
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR interruptDescriptor,
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR txDmaResourcePtr,
        _In_ PDEVICE_OBJECT PDO
    ) override;

    virtual NTSTATUS RegisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    ) override;

    virtual NTSTATUS UnregisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    ) override;

    virtual NTSTATUS StartDma
    (
        _In_        CMiniportWaveRTStream* Stream
    ) override;

    virtual NTSTATUS PauseDma
    (
        _In_        CMiniportWaveRTStream* Stream
    ) override;

    virtual NTSTATUS StopDma
    (
        _In_        CMiniportWaveRTStream* Stream
    ) override;

    NTSTATUS AllocBuffer
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType                        DeviceType,
        _Inout_     PULONG                 Size,
        _Out_       PMDL*                  pMdl,
        _Out_       MEMORY_CACHING_TYPE*   CacheType
    ) override;

    NTSTATUS FreeBuffer
    (
        _In_        CMiniportWaveRTStream* Stream,
        eDeviceType                        DeviceType,
        _In_        PMDL                   Mdl,
        _In_        ULONG                  Size
    ) override;

    VOID Notify(DMA_COMPLETION_STATUS Status);

private:
    volatile PAUD2HTX_REGISTERS m_pAud2HtxRegisters;
    PHYSICAL_ADDRESS            m_RegBase;
    PWAVEFORMATEXTENSIBLE  m_pWfExt;
    CMiniportWaveRTStream* m_pRtStream;
    BOOLEAN                m_bIsRenderPaused;
    ULONG                  m_ulSamplesTransferred;
    /* DMA context */
    _DMA_ADAPTER          *m_pDmaAdapter;
    DMA_TRANSFER_INFO      m_TransferInfo;
    PVOID                  m_BufVirt;
    PVOID                  m_MapRegisterBase;
    PMDL                   m_BufMdl;
    PHYSICAL_ADDRESS       m_BufLogical;
    ULONG                  m_BufSize;    
    ULONG                  m_NumberOfMapRegs;
    ULONG                  m_RequestLine;  
    ULONG                  m_NotificationBytes;
    UINT                   m_DmaTransferContext[DMA_TRANSFER_CONTEXT_SIZE_V1 / sizeof(UINT)];
    
    volatile ULONG         m_IsrCnt;

    VOID CleanUp();
    NTSTATUS Setup();

    VOID TxSoftwareReset();

    VOID DumpRegs(void)
    {
        DBG_DRV_PRINT_VERBOSE("Control:0x%x", READ_REGISTER_ULONG(&m_pAud2HtxRegisters->Control.AsUlong));
        DBG_DRV_PRINT_VERBOSE("ControlExt:0x%x", READ_REGISTER_ULONG(&m_pAud2HtxRegisters->ControlExt.AsUlong));
        DBG_DRV_PRINT_VERBOSE("Status:0x%x", READ_REGISTER_ULONG(&m_pAud2HtxRegisters->Status.AsUlong));
        DBG_DRV_PRINT_VERBOSE("NonMaskedIntFlags:0x%x", READ_REGISTER_ULONG(&m_pAud2HtxRegisters->NonMaskedIntFlags.AsUlong));
        DBG_DRV_PRINT_VERBOSE("MaskedIntFlags:0x%x", READ_REGISTER_ULONG(&m_pAud2HtxRegisters->MaskedIntFlags.AsUlong));
        DBG_DRV_PRINT_VERBOSE("IrqMasks:0x%x", READ_REGISTER_ULONG(&m_pAud2HtxRegisters->IrqMasks.AsUlong));
    }
};
