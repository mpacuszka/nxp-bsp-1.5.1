/*
 * Copyright 2022 NXP
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

extern "C" {
#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
}
#include "dsdtutil.hpp"
#include "trace.h"
#include "public.h"
#include "ImxCpuRev.h"

#define DISPATCH_CODE() PAGED_ASSERT(KeGetCurrentIrql() == DISPATCH_LEVEL);
#if (DBG)
#define _DbgKdPrint
// KdPrint
// Uncomment _DbgFrameKdPrint to enable ISR debug messages.
#define _DbgFrameKdPrint(...)
// KdPrint(__VA_ARGS__)
#else // !DBG
#define _DbgKdPrint(...)
#define _DbgFrameKdPrint(...)
#endif // !DBG


/* Two frame buffer loaded to ISI register. Must be 3 or higher. */
#define COMMON_FRAME_BUFFER_NUM 3U

#if COMMON_FRAME_BUFFER_NUM < 3
#error COMMON_FRAME_BUFFER_NUM must be 3 or higher.
#endif

EXTERN_C_START

//
// WDFDRIVER Events
//

DRIVER_INITIALIZE DriverEntry;

EXTERN_C_END

struct WdfIsi_ctx;
typedef WdfIsi_ctx DEVICE_CONTEXT, *PDEVICE_CONTEXT;
struct WdfIsiDeviceChannel_ctx;
typedef WdfIsiDeviceChannel_ctx SUBDEVICE_QUEUE_CONTEXT, *PSUBDEVICE_QUEUE_CONTEXT;
struct WdfIsiRequest_ctx;
typedef WdfIsiRequest_ctx  REQUEST_CONTEXT, *PREQUEST_CONTEXT;
struct IsiIsrCtx_t;
typedef IsiIsrCtx_t DEVICE_INTERRUPT_CONTEXT, *PDEVICE_INTERRUPT_CONTEXT;
struct WdfIsiFile_ctx;
typedef WdfIsiFile_ctx DEVICE_FILE_CONTEXT, *PDEVICE_FILE_CONTEXT;


#define MAX_CHAN 2

#include "WdfIoTargets.hpp"

#include "isi_iomap.h"

using namespace ISI_REG_BITFIELDS;
using namespace ISI_REG_BITFIELDS_V3;

struct WdfIsi_ctx: io::ctx_acpi_csr_stub
{
    enum : ULONG { IMX_CSI_POOL_TAG = 'ICXM' };
    struct DiscardBuffInfo_t {
        PHYSICAL_ADDRESS physY; // Base address
        PHYSICAL_ADDRESS physU; // Optional, calculated from physY.
        PHYSICAL_ADDRESS physV; // Optional, calculated from physY.
        PVOID virt;

        PMDL mdlPtr;
        enum {
            FREE,
            WORKING,
            DONE
        } state;
    };

    const WDFDEVICE m_WdfDevice;
    WDFQUEUE m_Queue;

    WDFWAITLOCK m_CreateDevLock;

    /* PNP static callbacks */
    static EVT_WDF_DEVICE_D0_ENTRY EvtD0Entry;
    static EVT_WDF_DEVICE_D0_EXIT EvtD0Exit;
    static EVT_WDF_DEVICE_RELEASE_HARDWARE EvtReleaseHw; // Translates call to nonstatic member.
    static EVT_WDF_DEVICE_PREPARE_HARDWARE EvtPrepareHw;

    NTSTATUS PrepareHw(_In_ WDFCMRESLIST ResourcesRaw, _In_ WDFCMRESLIST ResourcesTranslated);
    NTSTATUS Get_DsdAcpiResources();
    AcpiDsdRes_t m_DsdRes;

    reg<ISI_REGS> m_Isi1Reg; // TODO consider moving to DSD or resource class
    UINT32 m_Isi1RegResId;
    UINT32 m_MipiCsiSrc;
    UINT32 m_MaxMipiCsiSrc;
    UINT32 m_IsiFbMemReserveResId;
    bool m_IsiHasFbMemReserveRes;
    reg<UINT32*> m_IsiFbMemReserve;
    volatile ISI_REGS *m_IsiRegistersPtr;
    Resources_t m_IsiRes;

    CHAR m_DeviceEndpoint[DEVICE_ENDPOINT_NAME_MAX_LEN];
    WCHAR m_DeviceEndpointUnicodeNameBuff[DEVICE_ENDPOINT_NAME_MAX_LEN];
    UNICODE_STRING m_DeviceEndpointUnicodeName;

    // interrupt servicing

    PDEVICE_INTERRUPT_CONTEXT m_IsrCtx;

    /* IRP */
    static EVT_WDF_DEVICE_FILE_CREATE EvtDeviceFileCreate;
    static EVT_WDF_FILE_CLOSE EvtDeviceFileClose;
    static EVT_WDF_FILE_CLEANUP EvtDeviceFileCleanup;

    static EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL EvtDeviceControl;
    static EVT_WDF_IO_QUEUE_IO_STOP EvtIoStop;
    static EVT_WDF_REQUEST_CANCEL EvtWdfRequestCancel;
    /* IRP MJ */
    NTSTATUS Close();
    void EvtFrameRequest(PREQUEST_CONTEXT RequestCtxPtr);
    void TerminateIo();
    void ReinitializeRequest(PREQUEST_CONTEXT RequestCtxPtr);
    void EvtInputFormatRequest(PREQUEST_CONTEXT RequestCtxPtr);

    enum {
        S_FRAME_REQUESTED,
        S_STOP_REQUESTED,
        S_DISCARDING,
        S_STOPPED,
    } m_State;

    UINT32 m_CurrSrErrorMask;
    void DisableAllInterruptsNofence();
    void EnableErrorInterruptsNofence(bool enable, bool clear = false);
    void EnableFrameInterruptsNofence(bool enable, bool clear = false);
    void IsiStart(bool Enable);
    void IsiResetAndStop();
    void WdfIsi_ctx::setFrameBuffer(UINT8 FrameBudId, DiscardBuffInfo_t &discardBuff);
    NTSTATUS IsiInit(const camera_config_t &Config);
    void FinishGetFrameRequest(WdfIsi_ctx::DiscardBuffInfo_t*, PREQUEST_CONTEXT RequestCtxPtr);

    PREQUEST_CONTEXT m_ActiveRequestCtxPtr;
    volatile DiscardBuffInfo_t *m_FinishedBuffPtr;
    DiscardBuffInfo_t m_DiscardBuff[COMMON_FRAME_BUFFER_NUM];
    NTSTATUS AllocFb(DiscardBuffInfo_t &BuffInfo);
    NTSTATUS MapFb(DiscardBuffInfo_t &BuffInfo);
    NTSTATUS UnmapFb(DiscardBuffInfo_t &BuffInfo);
    void FreeFb(DiscardBuffInfo_t &BuffInfo);
    NTSTATUS ReSplitFrameBuffers(const camera_config_t &Config);
    NTSTATUS AcquireBuffers();
    NTSTATUS ReleaseBuffers();
    static constexpr SIZE_T m_BufferRequiredSize = 1366 * 768 * 4;
    SIZE_T m_FrameLenBytes = 0;
    UINT8 m_NumPlanes;
    UINT8 m_PlaneBytesPerPixel;
    UINT8 m_CpuId;
    UINT8 m_QuirkInvertFrameId;

    // Counters
    UINT32 m_DiscardedFrameCnt;
    UINT32 m_ErrorHrespCnt;
    UINT32 m_ErrorAddrChangeCnt;
    UINT32 m_ErrorOverflowCnt;
    UINT32 m_CompleteFrameCnt;
public:
    WdfIsi_ctx(WDFDEVICE &Device);
    NTSTATUS RegisterQueue();
    static EVT_WDF_OBJECT_CONTEXT_CLEANUP EvtDriverContextCleanup;
    static EVT_WDF_DEVICE_CONTEXT_CLEANUP EvtDeviceObjCtxCleanup;
    static EVT_WDF_DRIVER_DEVICE_ADD EvtDeviceAdd;
};
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, DeviceGetContext);

struct SubdevCtxPtrListItem : list_item_t
{
    WdfIsiDeviceChannel_ctx *ptr;

    SubdevCtxPtrListItem(WdfIsiDeviceChannel_ctx *Ptr) : ptr(Ptr) { ; }
};

struct IsiIsrCtx_t
{
    PDEVICE_CONTEXT m_CtxPtr;
    WDFINTERRUPT m_WdfInterrupt;
    WdfIsiRequest_ctx *m_ReqCtxPtr;

    list_t<SubdevCtxPtrListItem> m_ActiveChannels;

    IsiIsrCtx_t(PDEVICE_CONTEXT ctxPtr, WDFINTERRUPT WdfInterrupt);
    static NTSTATUS registerInterruptHandler(PDEVICE_CONTEXT ctxPtr);
    static EVT_WDF_INTERRUPT_ENABLE EvtWdfInterruptEnable;
    static EVT_WDF_INTERRUPT_DISABLE EvtWdfInterruptDisable;
    static EVT_WDF_INTERRUPT_ISR EvtInterruptIsr;
    static EVT_WDF_INTERRUPT_DPC EvtInterruptDpc;

};
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_INTERRUPT_CONTEXT, DeviceGetInterruptContext);

struct WdfIsiRequest_ctx
{
    NTSTATUS m_status;
    ULONG m_CtlCode;
    WDFREQUEST m_WdfRequest;
    size_t m_InLen;
    FrameInfo_t m_FrameInfo;
};
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(REQUEST_CONTEXT, GetRequestContext);

struct WdfIsiFile_ctx
{
    WDFQUEUE m_WdfQueue;
    PDEVICE_CONTEXT    m_CtxPtr;
    PDEVICE_INTERRUPT_CONTEXT m_IsrCtxPtr;
    UCHAR m_ChanId;
};
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_FILE_CONTEXT, DeviceGetFileContext);