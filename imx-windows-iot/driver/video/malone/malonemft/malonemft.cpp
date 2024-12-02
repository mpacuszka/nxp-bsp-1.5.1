/* Modifications Copyright 2023 NXP */
#include "malonemft.h"
#include "malonemft_DecodeTask.h"
#include "CAutoLock.h"
#include <mfapi.h>
#include <mferror.h>
#include "malonemft_DebugLogger.h"
#include <initguid.h>
#include "dwl.h"
#include <minwindef.h>
#include <assert.h>
#include "CVpuMediaBuffer.h"

// {C6FAA015-44C0-42ea-813F-CE02159E4D7C}
DEFINE_GUID(MaloneMft_MFSampleExtension_Marker,
            0xc6faa015, 0x44c0, 0x42ea, 0x81, 0x3f, 0xce, 0x2, 0x15, 0x9e, 0x4d, 0x7c);

#define DUMP_DATA 0

#define MFT_NUM_DEFAULT_ATTRIBUTES  4
#define MALONE_HW_URL      L"i.MX8QM/QXP Malone VPU MFT hardware accelerator"

#if 0
    #define _MFT_METHOD_BEG
    #define _MFT_METHOD_END
    #define _MFT_METHOD_INFO
    #define _MFT_METHOD_ERROR
#endif
static char mftmsgbuf[256];
static int len = 0;
static _timespec64 mytime = { 0 };

#ifdef _MFT_METHOD_BEG
#define MFT_METHOD_BEG() \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms MaloneMFT:+++%s()\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_METHOD_BEG(...)
#endif

#ifdef _MFT_METHOD_END
#define MFT_METHOD_END(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms MaloneMFT:---%s() " _format_str_ "\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_METHOD_END(...)
#endif

#ifdef _MFT_METHOD_ERROR
#define MFT_METHOD_ERROR(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms MaloneMFT:   %s() ERROR: " _format_str_ "\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_METHOD_ERROR(...)
#endif

#ifdef _MFT_METHOD_INFO
#define MFT_METHOD_INFO(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms MaloneMFT:   %s() INFO: " _format_str_ "\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_METHOD_INFO(...)
#endif


// Global Variables
const struct _codecMap g_ppguidInputTypes[] = {
    { &MFVideoFormat_H264, "H264"},
    { &MFVideoFormat_H265, "H265"},
    { &MFVideoFormat_HEVC, "HEVC"},
    { &MFVideoFormat_VP80, "VP80"},
    { &MFVideoFormat_MPEG2, "MP2V"},
    { &MFVideoFormat_MP4S, "MP4V"},
    { &MFVideoFormat_M4S2, "MP4V"},
    { &MFVideoFormat_MP4V, "MP4V"},
};

const DWORD     g_dwNumInputTypes   = sizeof(g_ppguidInputTypes) / sizeof(g_ppguidInputTypes[0]);

const GUID     *g_ppguidOutputTypes[] = {
    &MFVideoFormat_ARGB32,
    //&MFVideoFormat_NV12,
};
const DWORD     g_dwNumOutputTypes   = sizeof(g_ppguidOutputTypes) / sizeof(g_ppguidOutputTypes[0]);

struct _timespec64 CMaloneMft::m_clockStart = { 0, 0 };

// Initializer
HRESULT CMaloneMft::CreateInstance(IMFTransform **ppHWMFT)
{
    HRESULT hr          = S_OK;
    CMaloneMft *pMyHWMFT    = NULL;
    MFT_METHOD_BEG();
    do {
        if (ppHWMFT == NULL) {
            hr = E_POINTER;
            break;
        }

        pMyHWMFT = new CMaloneMft();
        if (FAILED(hr)) {
            break;
        }

        hr = pMyHWMFT->InitializeTransform();
        if (FAILED(hr)) {
            break;
        }

        hr = pMyHWMFT->QueryInterface(IID_IMFTransform, (void **)ppHWMFT);
        if (FAILED(hr)) {
            break;
        }
    } while (false);

    SAFERELEASE(pMyHWMFT);
    MFT_METHOD_END("");
    return hr;
}

/****************************
********** ***********
****************************/

CMaloneMft::CMaloneMft(void)
{
    /****************************************************
    ** Todo: Initialize All Member variables used by your
    ** MFT
    ****************************************************/
    MFT_METHOD_BEG();
    // Do no insert anything before this call, this is the DLLs object count
    InterlockedIncrement(&m_ulNumObjects);

    if (InterlockedCompareExchange(&m_ulNumObjects, 1, 1) == 1) {
        // we're the first object, turn on tracing
        TraceInitialize();
    }

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    m_ulRef                 = 1;
    m_ulInputCounter        = 0;
    m_ulOutputCounter       = 0;
    m_ulInputSamplesAdded   = 0;
    m_ulInputSamplesDequeued = 0;
    m_ulKeyFrames           = 0;
    m_ulDroppedFrames       = 0;
    m_llNextSampleTime      = 0;

    m_Sub5000               = 0;
    m_Above5000             = 0;
    m_AboveFPS              = 0;

    m_dropMode = MF_DROP_MODE_NONE;

    m_streamStart.tv_nsec   = 0;
    m_streamStart.tv_sec    = 0;
    m_pInputMT              = NULL;
    m_pOutputMT             = NULL;
    m_pAttributes           = NULL;
    memset(&m_fps, 0, sizeof(m_fps));
    m_inputHeight           = 0;
    m_inputWidth            = 0;
    m_outputHeight          = 0;
    m_outputWidth           = 0;
    m_uiInterlaceMode       = MFVideoInterlace_Unknown;
    m_bInterlaced           = FALSE;
    m_pEventQueue           = NULL;
    m_dwStatus              = 0;
    m_dwNeedInputCount      = 0;
    m_dwHaveOutputCount     = 0;
    m_dwPendingFrameDecodeCount = 0;
    m_dwDecodeWorkQueueID   = 0;
    m_bShutdown             = FALSE;
    m_bFirstSample          = TRUE;
    m_bDXVA                 = FALSE;
    m_pInputSampleQueue     = NULL;
    m_pWaitingSampleQueue   = NULL;
    m_pOutputSampleQueue    = NULL;
    m_bufferCollection      = NULL;

    m_vdec_init = { 0 };
    m_vdec_status = { 0 };
    m_vdec_decode = { 0 };

    m_frame_out_cnt = 0;

    m_vpuHandle             = NULL;
    m_vpuDecInit            = false;
    m_totalInputDataLen     = 0;

    InitializeCriticalSection(&m_csLock);

    if (m_clockStart.tv_sec == 0 && m_clockStart.tv_nsec == 0) {
        _timespec64_get(&m_clockStart, TIME_UTC);
    }

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit", __FUNCTION__);
    MFT_METHOD_END("");
}

CMaloneMft::~CMaloneMft(void)
{
    /****************************************************
    ** Todo: Release All Member variables used by your
    ** MFT
    ****************************************************/
    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    SAFERELEASE(m_pInputMT);
    SAFERELEASE(m_pOutputMT);
    SAFERELEASE(m_pAttributes);
    if (m_pEventQueue != NULL) {
        m_pEventQueue->Shutdown();
        SAFERELEASE(m_pEventQueue);
    }
    SAFERELEASE(m_pInputSampleQueue);
    SAFERELEASE(m_pWaitingSampleQueue);
    SAFERELEASE(m_pOutputSampleQueue);
    SAFERELEASE(m_bufferCollection);

    if (m_vpuDecInit != 0) {
        decoder_deinit(m_vpuHandle);
        m_vpuDecInit = 0;
    }
    if (m_vpuHandle != 0) {
        decoder_close(m_vpuHandle);
        m_vpuHandle = 0;
    }
    DeleteCriticalSection(&m_csLock);

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit", __FUNCTION__);

    if (InterlockedCompareExchange(&m_ulNumObjects, 1, 1) == 1) {
        // We're the last instance, turn off tracing
        TraceUninitialize();
    }

    InterlockedDecrement(&m_ulNumObjects);
    // Do no insert anything after this call, this is the DLLs object count
    MFT_METHOD_END("");
}

HRESULT CMaloneMft::InitializeTransform(void)
{
    /*************************************
    ** Todo: Use this function to setup
    ** anything that can't be setup in
    ** the constructor
    *************************************/

    HRESULT hr = S_OK;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        hr = MFCreateAttributes(&m_pAttributes, MFT_NUM_DEFAULT_ATTRIBUTES);
        if (FAILED(hr)) {
            break;
        }

        /*********************************
        ** Certain Attributes are required
        ** for HW MFTs
        ** See http://msdn.microsoft.com/en-us/library/dd940330(VS.85).aspx#attributes
        *********************************/
        hr = m_pAttributes->SetUINT32(MF_TRANSFORM_ASYNC, TRUE);
        if (FAILED(hr)) {
            break;
        }

        /****************************************
        ** !!MSFT_TODO: Report as HW MFT
        ****************************************
        hr = m_pAttributes->SetString(MFT_ENUM_HARDWARE_URL_Attribute, MFT_HW_URL);
        if(FAILED(hr))
        {
            break;
        }

        hr = m_pAttributes->SetString(MFT_ENUM_HARDWARE_VENDOR_ID_Attribut, MFT_HW_VENDOR_ID);
        if(FAILED(hr))
        {
            break;
        }
        */

        hr = m_pAttributes->SetUINT32(MFT_SUPPORT_DYNAMIC_FORMAT_CHANGE, TRUE);
        if (FAILED(hr)) {
            break;
        }

        /**********************************
        ** Since this is an Async MFT, an
        ** event queue is required
        ** MF Provides a standard implementation
        **********************************/
        hr = MFCreateEventQueue(&m_pEventQueue);
        if (FAILED(hr)) {
            MFT_METHOD_ERROR("Failed to create MF Event Queue (hr=0x%x)", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to create MF Event Queue (hr=0x%x)", __FUNCTION__, hr);
            break;
        }

        hr = CSampleQueue::Create(&m_pInputSampleQueue);
        if (FAILED(hr)) {
            MFT_METHOD_ERROR("Failed to create Input Sample Queue (hr=0x%x)", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to create Input Sample Queue (hr=0x%x)", __FUNCTION__, hr);
            break;
        }

        hr = CSampleQueue::Create(&m_pWaitingSampleQueue);
        if (FAILED(hr)) {
            MFT_METHOD_ERROR("Failed to create Input Sample Queue (hr=0x%x)", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to create Input Sample Queue (hr=0x%x)", __FUNCTION__, hr);
            break;
        }

        hr = CSampleQueue::Create(&m_pOutputSampleQueue);
        if (FAILED(hr)) {
            MFT_METHOD_ERROR("Failed to create Output Sample Queue (hr=0x%x)", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to create Output Sample Queue (hr=0x%x)", __FUNCTION__, hr);
            break;
        }

        /**********************************
        ** Since this is an Async MFT, all
        ** work will be done using standard
        ** MF Work Queues
        **********************************/
        hr = MFAllocateWorkQueueEx(MF_STANDARD_WORKQUEUE, &m_dwDecodeWorkQueueID);
        if (FAILED(hr)) {
            break;
        }
    } while (false);

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit (hr=0x%x)", __FUNCTION__, hr);
    MFT_METHOD_END("(hr = 0x%x)", hr);
    return hr;
}

HRESULT CMaloneMft::CheckInputType(
    IMFMediaType   *pMT,
    UINT32         *fourcc)
{
    HRESULT hr      = S_OK;
    GUID    guid    = GUID_NULL;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        /******************************
        ** No need to verify pMT != NULL
        ** as this is a private function
        ******************************/

        hr = pMT->GetGUID(MF_MT_MAJOR_TYPE, &guid);
        if (FAILED(hr)) {
            break;
        }

        if (guid != MFMediaType_Video) {
            hr = MF_E_INVALIDMEDIATYPE;
            break;
        }

        hr = pMT->GetGUID(MF_MT_SUBTYPE, &guid);
        if (FAILED(hr)) {
            break;
        }

        hr = MF_E_INVALIDMEDIATYPE; // Gets set to S_OK if MT is acceptable

        for (DWORD i = 0; i < g_dwNumInputTypes; i++) {
            if (guid == *(g_ppguidInputTypes[i].guid)) {
                *fourcc = AOIFOURCC(g_ppguidInputTypes[i].fourcc);
                hr = S_OK;
                break;
            }
        }

        if (FAILED(hr)) {
            break;
        }
        // The Mediatype is acceptable
    } while (false);

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);
    MFT_METHOD_END("(hr = 0x%x)", hr);

    return hr;
}

HRESULT CMaloneMft::CheckOutputType(
    IMFMediaType   *pMT)
{
    /*************************************
    ** Todo: Your MFT should verify the
    ** Output media type is acceptable
    ** Modify this function as necessary
    *************************************/

    HRESULT hr              = S_OK;
    GUID    guid            = GUID_NULL;
    UINT32  un32HighWord    = 0;
    UINT32  un32LowWord     = 0;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        /******************************
        ** No need to verify pMT != NULL
        ** as this is a private function
        ******************************/

        hr = pMT->GetGUID(MF_MT_MAJOR_TYPE, &guid);
        if (FAILED(hr)) {
            break;
        }

        if (guid != MFMediaType_Video) {
            hr = MF_E_INVALIDMEDIATYPE;
            break;
        }

        hr = pMT->GetGUID(MF_MT_SUBTYPE, &guid);
        if (FAILED(hr)) {
            break;
        }

        hr = MF_E_INVALIDMEDIATYPE; // Gets set to S_OK if MT is acceptable

        for (DWORD i = 0; i < g_dwNumOutputTypes; i++) {
            if (guid == *(g_ppguidOutputTypes[i])) {
                hr = S_OK;
                break;
            }
        }

        if (FAILED(hr)) {
            break;
        }

        hr = MFGetAttributeSize(pMT, MF_MT_FRAME_SIZE, &un32HighWord, &un32LowWord);
        if (FAILED(hr)) {
            break;
        }

        if ((un32HighWord != m_outputWidth) || (un32LowWord != m_outputHeight)) {
            hr = MF_E_INVALIDMEDIATYPE;
            break;
        }

        hr = MFGetAttributeRatio(pMT, MF_MT_FRAME_RATE, &un32HighWord, &un32LowWord);
        if (FAILED(hr)) {
            break;
        }

        if ((un32HighWord != m_fps.Numerator) || (un32LowWord != m_fps.Denominator)) {
            hr = MF_E_INVALIDMEDIATYPE;
            break;
        }

        // The Mediatype is acceptable
    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::ShutdownEventQueue(void)
{
    HRESULT hr = S_OK;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        /***************************************
        ** Since this in an internal function
        ** we know m_pEventQueue can never be
        ** NULL due to InitializeTransform()
        ***************************************/

        hr = m_pEventQueue->Shutdown();
        if (FAILED(hr)) {
            break;
        }
    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::OnStartOfStream(void)
{
    HRESULT hr = S_OK;
    int counter = 0;
    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        {
            CAutoLock lock(&m_csLock);
            m_vdec_decode.trick_mode = 0;
            m_vdec_decode.eos = 0;

            m_dwStatus |= MYMFT_STATUS_STREAM_STARTED;
        }

        _timespec64_get(&m_streamStart, TIME_UTC);

        hr = RequestSample(0);
        if (FAILED(hr)) {
            break;
        }
    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::OnEndOfStream(void)
{
    HRESULT hr = S_OK;
    bool dummyNeedInput = 0;
    int counter = 0;
    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        CAutoLock lock(&m_csLock);
        decoder_status(m_vpuHandle, &m_vdec_status);
        AddEos(&m_vdec_decode, &m_vdec_init, &m_vdec_status, m_sb_mem.virtAddress);
        m_vdec_decode.eos = 1;
        hr = decoder_decode(m_vpuHandle, &m_vdec_decode);

        if (m_vdec_decode.oindex != -1) {
            hr = decoder_clear_output(m_vpuHandle, &m_vdec_decode.oindex);
        }
        m_dwStatus &= (~MYMFT_STATUS_STREAM_STARTED);

        /*****************************************
        ** See http://msdn.microsoft.com/en-us/library/dd317909(VS.85).aspx#processinput
        ** Upon receiving EOS, the outstanding process
        ** input request should be reset to 0
        *****************************************/
        m_dwNeedInputCount = 0;
    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::OnDrain(
    const UINT32 un32StreamID)
{
    HRESULT hr = S_OK;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {

        CAutoLock lock(&m_csLock);
        m_dwStatus |= (MYMFT_STATUS_DRAINING);
        if (m_dwHaveOutputCount == 0 && m_dwPendingFrameDecodeCount == 0) {
            hr = SendDrainCompleteEvent();
        }

    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::OnFlush(void)
{
    HRESULT hr = S_OK;
    bool dummyNeedInput = 0;
    int counter = 0;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        CAutoLock lock(&m_csLock);

        m_dwStatus &= (~MYMFT_STATUS_STREAM_STARTED);

        hr = FlushSamples();
        if (FAILED(hr)) {
            break;
        }

        m_llNextSampleTime   = 0;    // Reset our sample time to 0 on a flush

        Flush(m_vpuHandle, &m_vdec_decode, &m_vdec_init, &m_vdec_status, (char *)m_sb_mem.virtAddress);

    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::OnMarker(
    const ULONG_PTR pulID)
{
    HRESULT hr  = S_OK;
    bool dummyNeedInput = 0;
    int counter = 0;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);

    do {
        // No need to lock, our sample queue is thread safe

        /***************************************
        ** Since this in an internal function
        ** we know m_pInputSampleQueue can never be
        ** NULL due to InitializeTransform()
        ***************************************/

        hr = m_pInputSampleQueue->MarkerNextSample(pulID);
        if (FAILED(hr)) {
            break;
        }
    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::RequestSample(
    const UINT32 un32StreamID)
{
    HRESULT         hr      = S_OK;
    IMFMediaEvent  *pEvent  = NULL;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        {
            CAutoLock lock(&m_csLock);

            if ((m_dwStatus & MYMFT_STATUS_STREAM_STARTED) == 0) {
                // Stream hasn't started
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Stream Hasn't Started",  __FUNCTION__);
                hr = MF_E_NOTACCEPTING;
                break;
            }
        }

        hr = MFCreateMediaEvent(METransformNeedInput, GUID_NULL, S_OK, NULL, &pEvent);
        if (FAILED(hr)) {
            break;
        }

        hr = pEvent->SetUINT32(MF_EVENT_MFT_INPUT_STREAM_ID, un32StreamID);
        if (FAILED(hr)) {
            break;
        }

        // Increment the input needed counter before enqueueing the input needed event.
        {
            CAutoLock lock(&m_csLock);

            m_dwNeedInputCount++;

            MFT_METHOD_INFO("NeedInputCount: %u", m_dwNeedInputCount);
            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): NeedInputCount: %u",  __FUNCTION__, m_dwNeedInputCount);
        }

        hr = m_pEventQueue->QueueEvent(pEvent);
        if (FAILED(hr)) {
            break;
        }

    } while (false);

    SAFERELEASE(pEvent);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}


HRESULT CMaloneMft::FlushSamples(void)
{
    HRESULT hr = S_OK;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        CAutoLock lock(&m_csLock);

        m_ulInputSamplesAdded = 0;
        if (FAILED(hr)) {           // a new stream is started
            break;
        }

        m_dwHaveOutputCount = 0;    // Don't Output samples until new input samples are given

        hr = m_pInputSampleQueue->RemoveAllSamples();
        if (FAILED(hr)) {
            break;
        }

        hr = m_pWaitingSampleQueue->RemoveAllSamples();
        if (FAILED(hr)) {
            break;
        }

        hr = m_pOutputSampleQueue->RemoveAllSamples();
        if (FAILED(hr)) {
            break;
        }

        m_bFirstSample = TRUE; // Be sure to reset our first sample so we know to set discontinuity
    } while (false);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::ScheduleFrameDecode(void)
{
    HRESULT             hr              = S_OK;
    IMFSample          *pInputSample    = NULL;
    CDecodeTask        *pDecodeTask     = NULL;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        // No need to lock, sample queues are thread safe

        /***************************************
        ** Since this in an internal function
        ** we know m_pInputSampleQueue can never be
        ** NULL due to InitializeTransform()
        ***************************************/
        hr = m_pInputSampleQueue->GetNextSample(&pInputSample);
        if (FAILED(hr)) {
            break;
        }

        if (hr == S_FALSE) {
            // empty sample queue
            hr = S_OK;
            break;
        }

        {
            CAutoLock lock(&m_csLock);
            InterlockedIncrement(&m_ulInputSamplesDequeued);
            m_dwPendingFrameDecodeCount++;
        }

        hr = CDecodeTask::Create(
                 m_dwDecodeWorkQueueID,
                 pInputSample,
                 (IMFAsyncCallback **)&pDecodeTask);
        if (FAILED(hr)) {
            CAutoLock lock(&m_csLock);
            m_dwPendingFrameDecodeCount--;
            break;
        }

        hr = pDecodeTask->Begin(this);
        if (FAILED(hr)) {
            CAutoLock lock(&m_csLock);
            m_dwPendingFrameDecodeCount--;
            break;
        }
    } while (false);

    SAFERELEASE(pInputSample);
    SAFERELEASE(pDecodeTask);

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

/**
 * Append stream data to VPU streambuffer.
 *
 * @param malonefd VPU Handler.
 * @param dec Pointer to exchange structure for driver.
 * @param status Pointer to status structure.
 * @param stream_buffer_virt Stream buffer virtual address.
 * @param buffer Input buffer.
 * @param size Size of stream buffer.
 *
 * @return 0 if success; -1 if an error
 */
HRESULT CMaloneMft::AppendBuffer(vdec_decode_t *dec, vdec_status_t *status, char *stream_buffer_virt, uint8_t *buffer,
                                 int size)
{
    int     delta, space;
    unsigned    offset;
    int    head_length;

    MFT_METHOD_BEG();

    /* Because we cannot update status->wptr from client (due to race condition),
     * we have to use temporary wptr variable.
     */
    uint32_t    temp_wptr = status->wptr;

    if (temp_wptr != dec->wptr) {
        /* Incomplete packet */
        temp_wptr = dec->wptr;
    }


    /* Check available space */
    delta = status->rptr - temp_wptr;
    space = delta > 0 ? delta : m_sb_mem.size + delta;

    if (space < size) {
        MFT_METHOD_ERROR("Stream buffer is FULL!\n");
    }

    offset = temp_wptr - status->start;
    head_length = min(status->end - temp_wptr, (uint32_t)size);

    memcpy(stream_buffer_virt + offset, buffer, head_length);

    if (head_length < size) {
        unsigned tail_length = size - head_length;
        memcpy(stream_buffer_virt, buffer + head_length, tail_length);
    }

    offset = temp_wptr - status->start + size;
    dec->wptr = status->start + offset % m_sb_mem.size;

    MFT_METHOD_END("");
    return S_OK;
}



#define VP8_SEQ_HDR_CODE    0x31
#define VP8_FRM_HDR_CODE    0x32
#define VP8_SLC_HDR_CODE    0x33
#define VP8_CODEC_ID        0x36
#define VP8_CODEC_VERSION   0x1
#define VP8_TIMESTAMP_SIZE  8
#define VP8_MALONE_HDR_SIZE 16
#define VP8_FRAME_SIZE      4
#define VP8_FRAME_HEADER_LEN  (VP8_FRAME_SIZE + VP8_TIMESTAMP_SIZE)

/**
 * Append Add VP8 Seq Hdr to VPU streambuffer.
  *
 * @param malonefd VPU Handler.
 * @param dec Pointer to exchange structure for driver.
 * @param status Pointer to status structure.
 * @param sb_virt_uaddr Stream buffer virtual address.
 * @param size of payload
 * @param timestamp 64bit
 */
void CMaloneMft::AddVP8FrmHdr(vdec_decode_t *dec, vdec_status_t *status, void *sb_virt_uaddr, uint32_t nitems,
                              LONGLONG timestamp)
{
    uint8_t  buffer[VP8_MALONE_HDR_SIZE + VP8_TIMESTAMP_SIZE] = { 0 };
    //
    // MALONE HDR
    //
    *buffer = 0;
    buffer[1] = 0;
    buffer[2] = 1;
    buffer[3] = VP8_FRM_HDR_CODE;
    buffer[4] = (uint8_t)((nitems + VP8_FRAME_HEADER_LEN + VP8_TIMESTAMP_SIZE) >> 16); // Payload length bits[23:16]
    buffer[5] = (uint8_t)((nitems + VP8_FRAME_HEADER_LEN + VP8_TIMESTAMP_SIZE) >> 8);  // Payload length bits[15:8]
    buffer[6] = 0x4e; // const
    buffer[7] = (uint8_t)(nitems + VP8_FRAME_HEADER_LEN + VP8_TIMESTAMP_SIZE); // Payload length bits[7:0]
    buffer[8] = VP8_CODEC_ID;
    buffer[9] = VP8_CODEC_VERSION;
    buffer[10] = (uint8_t)(m_inputWidth >> 8); // bits[15:8]
    buffer[11] = (uint8_t)m_inputWidth;        // bits[7:0]
    buffer[12] = 0x58; // const
    buffer[13] = (uint8_t)(m_outputHeight >> 8); // bits[15:8]
    buffer[14] = (uint8_t)m_outputHeight;        // bits[7:0]
    buffer[15] = 0x50; // const

    uint64_t *buf_ts = (uint64_t *)&buffer[16];
    *buf_ts = timestamp;

    AppendBuffer(dec, status, (char *)sb_virt_uaddr, buffer, VP8_MALONE_HDR_SIZE + VP8_TIMESTAMP_SIZE);
    MFT_METHOD_INFO("VP8 FRM HDR added");

}

/**
 * Append Add VP8 Seq Hdr to VPU streambuffer.
 *
 * @param malonefd VPU Handler.
 * @param dec Pointer to exchange structure for driver.
 * @param init Pointer to init structure.
 * @param status Pointer to status structure.
 * @param sb_virt_uaddr Stream buffer virtual address.
 * @param width input width
 * @param height input height
 * @param m_fps MFRatio fps
 */
void CMaloneMft::AddVP8SeqHdr(vdec_decode_t *dec, vdec_status_t *status, void *sb_virt_uaddr, uint32_t width,
                              int32_t height, MFRatio m_fps)
{
    uint8_t  buffer[3 * VP8_MALONE_HDR_SIZE] = { 0 };

    //
    // MALONE HDR
    //
    *buffer   = 0;
    buffer[1] = 0;
    buffer[2] = 1;
    buffer[3] = VP8_SEQ_HDR_CODE;
    buffer[4] = 0;    // Payload length bits[23:16]
    buffer[5] = 0;    // Payload length bits[15:8]
    buffer[6] = 0x4e; // const
    buffer[7] = 0x2c; // Payload length bits[7:0]
    buffer[8] = VP8_CODEC_ID;
    buffer[9] = VP8_CODEC_VERSION;
    buffer[10] = (uint8_t)(width >> 8); // bits[15:8]
    buffer[11] = (uint8_t)width;        // bits[7:0]
    buffer[12] = 0x58; // const
    buffer[13] = (uint8_t)(height >> 8); // bits[15:8]
    buffer[14] = (uint8_t)height;        // bits[7:0]
    buffer[15] = 0x50; // const

    //
    // IVF HDR
    //
    buffer[16] = 'D';
    buffer[17] = 'K';
    buffer[18] = 'I';
    buffer[19] = 'F';
    buffer[20] = 0;    // version -should be 0
    buffer[21] = 0;    // version -should be 0
    buffer[22] = 32;   // length of header in bytes
    buffer[23] = 0;    // length of header in bytes
    buffer[24] = 'V';  // Fourcc code #1
    buffer[25] = 'P';  // Fourcc code #2
    buffer[26] = '8';  // Fourcc code #3
    buffer[27] = '0';  // Fourcc code #4
    buffer[28] = (uint8_t)(width);
    buffer[29] = (uint8_t)(width >> 8);
    buffer[30] = (uint8_t)(height);
    buffer[31] = (uint8_t)(height >> 8);

    buffer[32] = (uint8_t)(m_fps.Denominator);
    buffer[33] = (uint8_t)(m_fps.Denominator >> 8);
    buffer[34] = (uint8_t)(m_fps.Denominator >> 16);
    buffer[35] = (uint8_t)(m_fps.Denominator >> 24);
    buffer[36] = (uint8_t)(m_fps.Numerator);
    buffer[37] = (uint8_t)(m_fps.Numerator >> 8);
    buffer[38] = (uint8_t)(m_fps.Numerator >> 16);
    buffer[39] = (uint8_t)(m_fps.Numerator >> 24);
    buffer[40] = 0; // Number of frames in file #1
    buffer[41] = 0; // Number of frames in file #2
    buffer[42] = 0; // Number of frames in file #3
    buffer[43] = 0; // Number of frames in file #4
    buffer[44] = 0; // unused
    buffer[45] = 0; // unused
    buffer[46] = 0; // unused
    buffer[47] = 0; // unused

    AppendBuffer(dec, status, (char *)sb_virt_uaddr, buffer, 48);
    MFT_METHOD_INFO("VP8 SEQ HDR added");

}

/**
 * Append EOS to VPU streambuffer.
 *
 * @param malonefd VPU Handler.
 * @param dec Pointer to exchange structure for driver.
 * @param init Pointer to init structure.
 * @param status Pointer to status structure.
 * @param sb_virt_uaddr Stream buffer virtual address.
 *
 * @return 0 if success; -1 if an error
 */
void CMaloneMft::AddEos(vdec_decode_t *dec, vdec_init_t *init, vdec_status_t *status, void *sb_virt_uaddr)
{
    uint8_t  buffer[4096] = { 0 };
    uint32_t *plbuffer = (uint32_t *)buffer;
    int i = 0;

    if (init->fourcc == (int32_t)AOIFOURCC("H264")) {
        plbuffer[0] = 0x0B010000;
        plbuffer[1] = 0x00000000;
    } else if (init->fourcc == (int32_t)AOIFOURCC("H265") || init->fourcc == (int32_t)AOIFOURCC("HEVC")) {
        plbuffer[0] = 0x4A010000;
        plbuffer[1] = 0x00000020;
    } else if (init->fourcc == (int32_t)AOIFOURCC("MP1V") || init->fourcc == (int32_t)AOIFOURCC("MP2V")) {
        plbuffer[0] = 0xCC010000;
        plbuffer[1] = 0x00000000;
    } else if (init->fourcc == (int32_t)AOIFOURCC("WMV3") || init->fourcc == (int32_t)AOIFOURCC("WVC1")) {
        plbuffer[0] = 0x0A010000;
        plbuffer[1] = 0x00000000;
    } else if (init->fourcc == (int32_t)AOIFOURCC("H263") || init->fourcc == (int32_t)AOIFOURCC("S263")
               || init->fourcc == (int32_t)AOIFOURCC("MP4V")) {
        plbuffer[0] = 0xB1010000;
        plbuffer[1] = 0x00000000;
    } else if (init->fourcc == (int32_t)AOIFOURCC("FLV1") || init->fourcc == (int32_t)AOIFOURCC("VP80")
               || init->fourcc == (int32_t)AOIFOURCC("VP60")) {
        plbuffer[0] = 0x34010000;
        plbuffer[1] = 0x00000000;
    } else if (init->fourcc == (int32_t)AOIFOURCC("MJPG")) {
        plbuffer[0] = 0xEFFF0000;
        plbuffer[1] = 0x00000000;
    }

    for (i = 2; i < 4096 >> 2; i++) {
        plbuffer[i] = 0;
    }

    AppendBuffer(dec, status, (char *)sb_virt_uaddr, buffer, 4096);
    MFT_METHOD_INFO("eos added");

}

#define MIN_SPACE 4096
void CMaloneMft::Flush(IDeviceIoControl *malonefd, vdec_decode_t *dec, vdec_init_t *init, vdec_status_t *status,
                       char *sb_virt_uaddr)
{
    int32_t flush_marker = 0x0;
    uint8_t buffer[MIN_SPACE + 40] = { 0 };
    uint32_t align = 3;
    int32_t front;
    int32_t cwptr = status->wptr;
    int32_t awptr = ((cwptr + align) & ~align);
    front = awptr - cwptr;
    vdec_flush_t flush = { 0 };

    if (init->fourcc == (int32_t)AOIFOURCC("H264")) {
        flush_marker = ABORT_MARKER_AVC;
    } else if (init->fourcc == (int32_t)AOIFOURCC("H265") || init->fourcc == (int32_t)AOIFOURCC("HEVC")) {
        flush_marker = ABORT_MARKER_HEVC;
    } else if (init->fourcc == (int32_t)AOIFOURCC("MP1V") || init->fourcc == (int32_t)AOIFOURCC("MP2V")) {
        flush_marker = ABORT_MARKER_MPEG2;
    } else if (init->fourcc == (int32_t)AOIFOURCC("WMV3") || init->fourcc == (int32_t)AOIFOURCC("WVC1")) {
        flush_marker = ABORT_MARKER_VC1;
    } else if (init->fourcc == (int32_t)AOIFOURCC("H263") || init->fourcc == (int32_t)AOIFOURCC("S263")) {
        flush_marker = EOS_MARKER_MPEG4;
    } else if (init->fourcc == (int32_t)AOIFOURCC("FLV1")) {
        flush_marker = ABORT_MARKER_SPARK;
    } else if (init->fourcc == (int32_t)AOIFOURCC("MJPG")) {
        flush_marker = 0x0;
    } else if (init->fourcc == (int32_t)AOIFOURCC("VP60")) {
        flush_marker = ABORT_MARKER_VP6;
    } else if (init->fourcc == (int32_t)AOIFOURCC("VP80")) {
        flush_marker = ABORT_MARKER_VP8;
    } else if (init->fourcc == (int32_t)AOIFOURCC("MP4V")) {
        flush_marker = ABORT_MARKER_MPEG4;
    }


    //TODO REVIEW
    memcpy(buffer + front, &flush_marker, 4);
    AppendBuffer(dec, status, (char *)sb_virt_uaddr, (uint8_t *)buffer, front + MIN_SPACE);

    flush.wptr = cwptr;
    flush.padding_size = front + MIN_SPACE;
    if ((decoder_flush(malonefd, &flush)) != 0) {
        MFT_METHOD_INFO("decoder_flush failed");
    }
    if ((decoder_status(malonefd, &m_vdec_status)) != 0) {
        MFT_METHOD_INFO("decoder_status failed");
    }
}

HRESULT CMaloneMft::ReadYUVFrame_FSL_8b(
    UINT32 nPicWidth,
    UINT32 nPicHeight,
    UINT32 uVOffsetLuma,
    UINT32 uVOffsetChroma,
    UINT32 nFsWidth,
    UINT8 **nBaseAddr,
    UINT8 *pDstBuffer,
    BOOL   bInterlaced
)
{
    UINT32 i;
    UINT32 h_tiles, v_tiles, v_offset, nLines, vtile, htile;
    UINT32 nLinesLuma = nPicHeight;
    UINT32 nLinesChroma = (nPicHeight >> 1);
    UINT32 nFrameStoreWidth;
    UINT8 *cur_addr;
    UINT32 *pBuffer = (UINT32 *)pDstBuffer;
    UINT32 *pTmpBuffer;

    pTmpBuffer = (UINT32 *)calloc(1, 256 * 4);
    if (!pTmpBuffer) {
        MFT_METHOD_ERROR("Failed to alloc pTmpBuffer\n");
        return -1;
    }

    nFrameStoreWidth = nFsWidth;

    if (bInterlaced) {
        // Per field no effect??
        (nLinesLuma >> 1);
        (nLinesChroma >> 1);
    }

    //---------------------------------------------------------------
    // Read Top Luma
    //---------------------------------------------------------------
    nLines = nLinesLuma;
    v_offset = uVOffsetLuma;
    h_tiles = (nPicWidth + 7) >> 3;
    v_tiles = (nLines + v_offset + 127) >> 7;

    for (vtile = 0; vtile < v_tiles; vtile++) {
        UINT32 v_base_offset = nFsWidth * 128 * vtile;

        for (htile = 0; htile < h_tiles; htile++) {
            cur_addr = (UINT8 *)(nBaseAddr[0] + htile * 1024 + v_base_offset);
            memcpy(pTmpBuffer, cur_addr, 256 * 4);

            for (i = 0; i < 128; i++) {
                INT32 line_num = (i + 128 * vtile) - v_offset;
                UINT32 line_base = (line_num * nPicWidth) >> 2;
                // Skip data that is off the bottom of the pic
                if (line_num == nLines) {
                    break;
                }
                // Skip data that is off the top of the pic
                if (line_num < 0) {
                    continue;
                }
                pBuffer[line_base + (2 * htile) + 0] = pTmpBuffer[2 * i + 0];
                pBuffer[line_base + (2 * htile) + 1] = pTmpBuffer[2 * i + 1];
            }
        }
    }

    if (bInterlaced) {
        pBuffer += (nPicWidth * nPicHeight) >> 3;

        //---------------------------------------------------------------
        // Read Bot Luma
        //---------------------------------------------------------------
        nLines = nLinesLuma;
        v_offset = uVOffsetLuma;
        h_tiles = (nPicWidth + 7) >> 3;
        v_tiles = (nLines + v_offset + 127) >> 7;
        uint8_t *bot_luma = nBaseAddr[0] + nLinesLuma * nPicWidth;
        for (vtile = 0; vtile < v_tiles; vtile++) {
            UINT32 v_base_offset = nFsWidth * 128 * vtile;

            for (htile = 0; htile < h_tiles; htile++) {
                cur_addr = (UINT8 *)(bot_luma + htile * 1024 + v_base_offset);
                memcpy(pTmpBuffer, cur_addr, 256 * 4);

                for (i = 0; i < 128; i++) {
                    INT32 line_num = (i + 128 * vtile) - v_offset;
                    UINT32 line_base = (line_num * nPicWidth) >> 2;
                    // Skip data that is off the bottom of the pic
                    if (line_num == nLines) {
                        break;
                    }
                    // Skip data that is off the top of the pic
                    if (line_num < 0) {
                        continue;
                    }
                    pBuffer[line_base + (2 * htile) + 0] = pTmpBuffer[2 * i + 0];
                    pBuffer[line_base + (2 * htile) + 1] = pTmpBuffer[2 * i + 1];
                }
            }
        }

        pBuffer += (nPicWidth * nPicHeight) >> 3;
    } else {
        pBuffer += (nPicWidth * nPicHeight) >> 2;
    }

    //---------------------------------------------------------------
    // Read Top Chroma
    //---------------------------------------------------------------
    nLines = nLinesChroma;
    v_offset = uVOffsetChroma;
    h_tiles = (nPicWidth + 7) >> 3;
    v_tiles = (nLines + v_offset + 127) >> 7;
    for (vtile = 0; vtile < v_tiles; vtile++) {
        UINT32 v_base_offset = nFsWidth * 128 * vtile;

        for (htile = 0; htile < h_tiles; htile++) {
            cur_addr = (UINT8 *)(nBaseAddr[1] + htile * 1024 + v_base_offset);
            memcpy(pTmpBuffer, cur_addr, 256 * 4);

            for (i = 0; i < 128; i++) {
                INT32 line_num = (i + 128 * vtile) - v_offset;
                UINT32 line_base = (line_num * nPicWidth) >> 2;
                // Skip data that is off the bottom of the pic
                if (line_num == nLines) {
                    break;
                }
                // Skip data that is off the top of the pic
                if (line_num < 0) {
                    continue;
                }
                pBuffer[line_base + (2 * htile) + 0] = pTmpBuffer[2 * i + 0];
                pBuffer[line_base + (2 * htile) + 1] = pTmpBuffer[2 * i + 1];
            }
        }
    }

    if (bInterlaced) {
        pBuffer += (nPicWidth * nPicHeight) >> 4;

        //---------------------------------------------------------------
        // Read Bot Chroma
        //---------------------------------------------------------------
        nLines = nLinesChroma;
        v_offset = uVOffsetChroma;
        h_tiles = (nPicWidth + 7) >> 3;
        v_tiles = (nLines + v_offset + 127) >> 7;
        uint8_t *bot_chroma = nBaseAddr[1] + nLinesChroma * nPicWidth;
        for (vtile = 0; vtile < v_tiles; vtile++) {
            UINT32 v_base_offset = nFsWidth * 128 * vtile;

            for (htile = 0; htile < h_tiles; htile++) {
                cur_addr = (UINT8 *)(bot_chroma + htile * 1024 + v_base_offset);
                memcpy(pTmpBuffer, cur_addr, 256 * 4);

                for (i = 0; i < 128; i++) {
                    INT32 line_num = (i + 128 * vtile) - v_offset;
                    UINT32 line_base = (line_num * nPicWidth) >> 2;
                    // Skip data that is off the bottom of the pic
                    if (line_num == nLines) {
                        break;
                    }
                    // Skip data that is off the top of the pic
                    if (line_num < 0) {
                        continue;
                    }
                    pBuffer[line_base + (2 * htile) + 0] = pTmpBuffer[2 * i + 0];
                    pBuffer[line_base + (2 * htile) + 1] = pTmpBuffer[2 * i + 1];
                }
            }
        }
    }

    free(pTmpBuffer);
    return (0);
}

#if DUMP_DATA
#include <string>
void dump_data(int frame, uint8_t *ipdata, int32_t isize, int plane_type)
{
    HANDLE hFile;
    wchar_t *tmpPath = NULL;
    DWORD tmpPathLen = GetTempPathW(0, tmpPath);
    DWORD dwBytesWritten = 0;
    BOOL bErrorFlag = FALSE;
    std::wstring first(L"frame");
    std::wstring second(L"");
    std::wstring number(std::to_wstring(frame));

    tmpPath = (wchar_t *)malloc(sizeof(wchar_t) * tmpPathLen);
    if (!tmpPath) {
        return;
    }

    tmpPathLen = GetTempPathW(tmpPathLen, tmpPath);
    tmpPath[tmpPathLen] = '\0';


    if (plane_type == 0) {
        second.assign(L"_luma.raw");
    } else if (plane_type == 1) {
        second.assign(L"_chroma.raw");
    } else if (plane_type == 2) {
        second.assign(L"_linear.raw");
    }

    std::wstring concatted_stdstr(tmpPath + first + number + second);
    LPWSTR filename = const_cast<LPWSTR>(concatted_stdstr.c_str());
    free(tmpPath);
    hFile = CreateFile(filename,
                       GENERIC_WRITE,          // open for writing
                       0,                      // do not share
                       NULL,                   // default security
                       CREATE_ALWAYS,             // create new file only
                       FILE_ATTRIBUTE_NORMAL,  // normal file
                       NULL);                  // no attr. template

    if (hFile == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        MFT_METHOD_ERROR("Unable to open file handle, GLE: %d", error);
        return;
    }
    bErrorFlag = WriteFile(
                     hFile,           // open file handle
                     ipdata,          // start of data to write
                     isize,           // number of bytes to write
                     &dwBytesWritten, // number of bytes that were written
                     NULL);            // no ov

    MFT_METHOD_INFO("Dumped frame %d, size %d", frame, plane_type ? isize / 2 : isize);
    CloseHandle(hFile);
}
#endif


HRESULT CMaloneMft::Decode(ULONG ulCurrentSample, bool *needInput, bool *askMeForOutput)
{
    HRESULT hr = S_OK;
    DWORD           dwDataLen = (m_outputWidth * m_outputHeight) * 4; // This is the max needed for the
    // biggest supported output type
    struct          _timespec64 frameStart = { 0 };
    struct          _timespec64 frameEnd = { 0 };

    struct          _timespec64 bufferAcquire = { 0 };
    struct          _timespec64 bufferRelease = { 0 };

    IMFSample *pOutputSample = NULL;
    IMFMediaBuffer *pMediaBuffer = NULL;
    BYTE *pbData = NULL;
    MFT_METHOD_BEG();
    _timespec64_get(&frameStart, TIME_UTC);

    do {
        MFT_METHOD_INFO("Calling decoder_decode, I have added %d input packet, total submitted size: 0x%x!",
                        m_ulInputSamplesAdded, m_totalInputDataLen);

        hr = decoder_decode(m_vpuHandle, &m_vdec_decode);
        if (MyFailed(hr)) {
            MFT_METHOD_ERROR("decoder_decode failed % d", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): decoder_decode failed %d",
                        __FUNCTION__, hr);
            break;
        }

        hr = decoder_status(m_vpuHandle, &m_vdec_status);
        if (MyFailed(hr)) {
            MFT_METHOD_ERROR("decoder_status failed % d", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): decoder_status failed %d",
                        __FUNCTION__, hr);
            break;
        }

        if (m_vdec_decode.oindex == -1) {
            MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
        }
        switch (m_vdec_decode.error) {
            case VDEC_EOK:
                break;
            case VDEC_LOW_INPUT_FRAMES: {
                *needInput = true;
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): VPU_DEC_NO_ENOUGH_INBUF",
                            __FUNCTION__);
                break;
            }
            case VDEC_LOW_OUTPUT_BUFFERS: {
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): VDEC_LOW_OUTPUT_BUFFERS",
                            __FUNCTION__);
                break;
            }
            case VDEC_FATAL_ERROR:
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                assert(0);
                break;
            case VDEC_FATAL_ERROR_UNSUPPORTED_STREAM:
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                assert(0);
                break;
            case VDEC_FATAL_ERROR_NOMEM:
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                assert(0);
                break;
            case VDEC_FATAL_ERROR_NOT_ENOUGH_REGISTERED_OBUFFERS:
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                assert(0);
                break;
            case VDEC_FATAL_ERROR_FIRMWARE_EXCEPTION:
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                assert(0);
                break;
            case VDEC_FATAL_ERROR_DCP_DISABLED_INF:
                MFT_METHOD_ERROR("decoder_decode error: %s! ", vdec_error2str[m_vdec_decode.error]);
                hr = E_OUTOFMEMORY;
                break;
            default:
                break;
        }

        if (MyFailed(hr)) {
            MFT_METHOD_ERROR("VDEC_ERROR reported % d", hr);
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): decoder_decode failed %d",
                        __FUNCTION__, hr);
            break;
        }

        ULONG outputFrameCount = 0;
        LONGLONG decodeTime = 0;
        LONGLONG bufferHoldTime = 0;
        LONGLONG framePeriod = 0;
        if (m_vdec_decode.oindex >= 0) {

            fbo_t fbo = { 0 };
            fbo.index = m_vdec_decode.oindex;
            MFT_METHOD_INFO("VPU Provided an output frame(%d)! ", m_vdec_decode.oindex);
            decoder_get_output(m_vpuHandle, &fbo);
            m_frame_out_cnt++;
            outputFrameCount = InterlockedIncrement(&m_ulOutputCounter);
            // S - TIME

            GetElapsedTime(&frameStart, &frameEnd);

            decodeTime = (LONGLONG)frameEnd.tv_sec * 1000000LL + (LONGLONG)frameEnd.tv_nsec / 1000LL;
            framePeriod = 1000000LL * (LONGLONG)m_fps.Denominator / (LONGLONG)m_fps.Numerator;
            if (decodeTime > framePeriod) {
                MFT_METHOD_INFO("vpu frame decode took longer than frame rate. us = %lld period = %lld", decodeTime, framePeriod);
                TraceString(CHMFTTracing::TRACE_INFORMATION,
                            L"%S(): vpu frame decode took longer than frame rate. us = %ld period = %ld",
                            __FUNCTION__, decodeTime, framePeriod);
                InterlockedIncrement(&m_AboveFPS);
            } else if (decodeTime > 5000) {
                MFT_METHOD_INFO("vpu frame decode took longer than 5ms. us = %lld", decodeTime);
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): vpu frame decode took longer than 5ms. us = %ld",
                            __FUNCTION__, decodeTime);
                InterlockedIncrement(&m_Above5000);
            } else {
                InterlockedIncrement(&m_Sub5000);
            }
            MFT_METHOD_INFO("vpu frame decode time from VPU_DecDecodeBuf to VPU_DecGetOutputFrame took %lld us",
                            decodeTime);
            TraceString(CHMFTTracing::TRACE_INFORMATION,
                        L"%S(): vpu frame decode time from decoder_decode to decoder_get_output took %ld us",
                        __FUNCTION__, decodeTime);

            // reset frameStart timestamp for next decoding.
            _timespec64_get(&frameStart, TIME_UTC);
            // E - TIME

            IMFSample *pInputSample;
            hr = m_pWaitingSampleQueue->GetNextSample(&pInputSample);
            if (FAILED(hr)) {
                MFT_METHOD_ERROR("m_pWaitingSampleQueue->GetNextSample failed % d", hr);
                break;
            }
            hr = CreateOutputSample(ulCurrentSample, pInputSample, &pOutputSample);
            if (FAILED(hr)) {
                MFT_METHOD_ERROR("CreateOutputSample failed % d", hr);
                break;
            }
            SAFERELEASE(pInputSample);
#if defined(VPU_COPY_FRAMEBUFFER)
            {
                hr = MFCreateMemoryBuffer(dwDataLen, &pMediaBuffer);
                if (FAILED(hr)) {
                    break;
                }

                hr = pMediaBuffer->Lock(&pbData, NULL, NULL);
                if (FAILED(hr)) {
                    break;
                }


            }
            UINT32 size = fbo.width * fbo.height * 4;
#if DUMP_DATA1
            dump_data(m_frame_out_cnt, (UINT8 *)fbo.planes[0], size, 0);
            dump_data(m_frame_out_cnt, (UINT8 *)fbo.planes[1], size / 2, 1);
#endif
            //ReadYUVFrame_FSL_8b(fbo.width, fbo.height, 0, 0, fbo.stride, (UINT8**)&fbo.planes, pbData, FALSE);
            memcpy(pbData, (void *)fbo.planes[0], size);     // Fill our buffer with a color correlated to the frame number
#if DUMP_DATA
            dump_data(m_frame_out_cnt, (UINT8 *)pbData, dwDataLen, 2);
#endif
            decoder_clear_output(m_vpuHandle, &m_vdec_decode.oindex);
#else
            MFT_METHOD_INFO("CreateBufferInstance START");
            _timespec64_get(&bufferAcquire, TIME_UTC);
            hr = m_bufferCollection->CreateBufferInstance(m_vpuHandle, dwDataLen, fbo, &pMediaBuffer);
            if (FAILED(hr)) {
                break;
            }
#endif
            hr = pMediaBuffer->SetCurrentLength(dwDataLen);
            if (FAILED(hr)) {
                break;
            }

            hr = pOutputSample->AddBuffer(pMediaBuffer);
            if (FAILED(hr)) {
                break;
            }

            SAFERELEASE(pMediaBuffer);

            hr = UpdateSampleTime(ulCurrentSample, pOutputSample);
            if (FAILED(hr)) {
                break;
            }
            m_vdec_decode.oindex = -2;

            // VPU_DecOutFrameDisplayed is called when pOutputBuffer is released
            MFT_METHOD_INFO("pOutputSample START");
            hr = SendOutputFrameEvent(pOutputSample);
            if (FAILED(hr)) {
                TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to output sample (hr=0x%x)", __FUNCTION__, hr);
                break;
            }
            SAFERELEASE(pOutputSample);
            GetElapsedTime(&bufferAcquire, &bufferRelease);
            bufferHoldTime = (LONGLONG)bufferRelease.tv_sec * 1000000LL + (LONGLONG)bufferRelease.tv_nsec / 1000LL;
            MFT_METHOD_INFO("MFT Since CreateBufferInstance to SendOutputFrameEvent took us = %lld", bufferHoldTime);
        }
#if 0
        else {
            //*needInput = 1;
            MFT_METHOD_ERROR("decoder_decode oindex: %d!", m_vdec_decode.oindex);
        }
#else
        else if (m_vdec_decode.oindex < 0 && m_vdec_decode.error == VDEC_EOK) {
            *askMeForOutput = 1;
        }
#endif
    } while (false);

    if (FAILED(hr)) {
        MFT_METHOD_ERROR("An ERROR!: %d", hr);
    }
#if 1
    if (m_vdec_decode.oindex >= 0) {
        MFT_METHOD_ERROR("Forgot to clear output: %d, fixing", m_vdec_decode.oindex);
        decoder_clear_output(m_vpuHandle, &m_vdec_decode.oindex);
    }
#endif

    MFT_METHOD_END("%d needInput=%d", hr, *needInput);

    return hr;
}

HRESULT CMaloneMft::DecodeInputFrame(
    IMFSample  *pInputSample)
{
    HRESULT         hr = S_OK;
    ULONG           ulCurrentSample = 0;
    IMFMediaBuffer *pInputBuffer = NULL;
    BYTE           *pbInputData = NULL;

    UINT64          pun64MarkerID = 0;
    bool            needInput = false;
    bool            askMeForOutput = false;
    bool            isKeyFrame = false;


    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        /****************************************
        ** Todo, here's where the MFT transforms
        ** the input to output samples. In the
        ** SDK sample, the output is simply a
        ** numbered frame, so no complicated
        ** processing is done here. Your MFT
        ** should reference it's input and output
        ** types to ensure it does the right thing
        ** here and modify this function accordingly
        ** Addionally, your MFT must monitor for
        ** format changes and act accordly
        ** See http://msdn.microsoft.com/en-us/library/ee663587(VS.85).aspx
        ****************************************/
        CAutoLock lock(&m_csLock);

        ulCurrentSample = InterlockedIncrement(&m_ulInputCounter);

        if (m_bDXVA != FALSE) { // Not thread safe!
            /****************************************
            ** !!MSFT_TODO: handle dxva!
            ****************************************/

            //MessageBox(NULL, L"TODO: Make MFT Handle DXVA!", L"ERROR!", MB_OK);
            hr = E_NOTIMPL;
            break;
        } else {

        }

        if (IsKeyFrame(pInputSample)) {
            MFT_METHOD_INFO("Sample %ld is key frame", ulCurrentSample);
            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Sample %ld is key frame",
                        __FUNCTION__, ulCurrentSample);
            InterlockedIncrement(&m_ulKeyFrames);
            isKeyFrame = true;
        }

        // Start frame decoding here
        DWORD maxLen = 0;
        DWORD curLen = 0;

        hr = pInputSample->ConvertToContiguousBuffer(&pInputBuffer);
        if (FAILED(hr)) {
            break;
        }

        hr = pInputBuffer->Lock(&pbInputData, &maxLen, &curLen);
        if (FAILED(hr)) {
            break;
        }

        do {

            if (m_vpuDecInit && (m_dropMode > MF_DROP_MODE_2)) {
                if (isKeyFrame) {
                    // force a discontinuity and resync presentation time stamp
                    m_bFirstSample = true;
                    m_llNextSampleTime = 0;
                } else {
                    MFT_METHOD_INFO(" Dropping P or B frame %ld ", ulCurrentSample);
                    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Dropping P or B frame %ld ",
                                __FUNCTION__, ulCurrentSample);
                    InterlockedIncrement(&m_ulDroppedFrames);
                    needInput = true;
                    break;
                }
            }

            // DECODE
            /*
             * Fill input data
             */
            m_vdec_decode.frame_in_cnt = m_ulInputSamplesAdded;
            m_totalInputDataLen += curLen;
            if (curLen > 0) {
                MFT_METHOD_INFO("Inserting input packet(%d) size: 0x%x!", m_ulInputSamplesAdded, curLen);
                if (m_vdec_init.fourcc == (uint32_t)AOIFOURCC("VP80")) {
                    LONGLONG timestamp;
                    if (SUCCEEDED(pInputSample->GetSampleTime(&timestamp))) {
                        AddVP8FrmHdr(&m_vdec_decode, &m_vdec_status, m_sb_mem.virtAddress, curLen, timestamp);
                    }
                }
                AppendBuffer(&m_vdec_decode, &m_vdec_status, (char *)m_sb_mem.virtAddress, pbInputData, curLen);
            }
            if (pInputBuffer != NULL) {
                if (pbInputData != NULL) {
                    pInputBuffer->Unlock();
                }
                SAFERELEASE(pInputBuffer);
            }


            m_pWaitingSampleQueue->AddSample(pInputSample);

            do {
                askMeForOutput = false;
                hr = Decode(ulCurrentSample, &needInput, &askMeForOutput);
                if (hr != S_OK) {
                    return hr;
                }
                if (needInput) {
                    break;
                }
                if (askMeForOutput) {
                    Sleep(10);
                }
            } while (askMeForOutput);
        } while (0);

        if (pInputSample->GetUINT64(MaloneMft_MFSampleExtension_Marker, &pun64MarkerID) == S_OK) {
            // This input sample flagged a marker
            IMFMediaEvent  *pMarkerEvent    = NULL;

            do {
                hr = MFCreateMediaEvent(METransformMarker, GUID_NULL, S_OK, NULL, &pMarkerEvent);
                if (FAILED(hr)) {
                    break;
                }

                hr = pMarkerEvent->SetUINT64(MF_EVENT_MFT_CONTEXT, pun64MarkerID);
                if (FAILED(hr)) {
                    break;
                }

                /***************************************
                ** Since this in an internal function
                ** we know m_pEventQueue can never be
                ** NULL due to InitializeTransform()
                ***************************************/

                hr = m_pEventQueue->QueueEvent(pMarkerEvent);
                if (FAILED(hr)) {
                    break;
                }
            } while (false);

            SAFERELEASE(pMarkerEvent);

            if (FAILED(hr)) {
                break;
            }
        }
        // Done processing this sample, request another if required
        if (needInput) {
            CAutoLock lock(&m_csLock);

            // CVpuMft::ProcessOutput causes the next sample to be requested. Only if we don't expect it to be called do
            // we request a sample here.
            if (!(m_dwStatus & MYMFT_STATUS_DRAINING) && (m_dwHaveOutputCount == 0) && (m_dwPendingFrameDecodeCount == 1)) {
                hr = RequestSample(0);
                if (FAILED(hr)) {
                    TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): RequestSample failed (hr=0x%x)", __FUNCTION__, hr);
                    break;
                }
                break;
            }
        }
    } while (false);

    {
        CAutoLock lock(&m_csLock);
        m_dwPendingFrameDecodeCount--;

        if ((m_dwStatus & MYMFT_STATUS_DRAINING) && m_dwPendingFrameDecodeCount == 0) {
            hr = SendDrainCompleteEvent();
        }
    }

    MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

HRESULT CMaloneMft::SendOutputFrameEvent(IMFSample *pOutputSample)
{
    HRESULT hr;
    IMFMediaEvent  *pHaveOutputEvent = NULL;
    MFT_METHOD_BEG();
    do {
        if (pOutputSample != NULL) {
            hr = m_pOutputSampleQueue->AddSample(pOutputSample);
            if (FAILED(hr)) {
                TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to add sample to output queue (hr=0x%x)", __FUNCTION__, hr);
                break;
            }
        } else {
            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Sending empty output frame for format change", __FUNCTION__);
        }

        // Now that the sample is in the output queue, send out have output event
        hr = MFCreateMediaEvent(METransformHaveOutput, GUID_NULL, S_OK, NULL, &pHaveOutputEvent);
        if (FAILED(hr)) {
            TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to create METransformHaveOutput event (hr=0x%x)", __FUNCTION__,
                        hr);
            break;
        }

        {
            CAutoLock lock(&m_csLock);

            hr = m_pEventQueue->QueueEvent(pHaveOutputEvent);
            if (FAILED(hr)) {
                // If this fails, consider decrementing m_dwHaveOutputCount
                TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to queue METransformHaveOutput event (hr=0x%x)", __FUNCTION__,
                            hr);
                break;
            }

            m_dwHaveOutputCount++;

            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): HaveOutputCount: %u", __FUNCTION__, m_dwHaveOutputCount);

            m_dwStatus |= MYMFT_STATUS_OUTPUT_SAMPLE_READY;
        }
    } while (FALSE);

    SAFERELEASE(pHaveOutputEvent);
    MFT_METHOD_END("(hr = 0x%x)", hr);
    return hr;
}

BOOL CMaloneMft::IsLocked(void)
{
    /***************************************
    ** Since this in an internal function
    ** we know m_pAttributes can never be
    ** NULL due to InitializeTransform()
    ***************************************/

    //MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    BOOL bUnlocked  = MFGetAttributeUINT32(m_pAttributes,
                                           MF_TRANSFORM_ASYNC_UNLOCK,
                                           FALSE
                                          );

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(%s)", __FUNCTION__,
                (bUnlocked != FALSE) ? L"False" : L"True");
    //MFT_METHOD_END("(%s)", (bUnlocked != FALSE) ? "False" : "True");

    return (bUnlocked != FALSE) ? FALSE : TRUE;
}

BOOL CMaloneMft::IsMFTReady(void)
{
    /*******************************
    ** The purpose of this function
    ** is to ensure that the MFT is
    ** ready for processing
    *******************************/

    BOOL bReady = FALSE;

    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        CAutoLock lock(&m_csLock);

        m_dwStatus &= (~MYMFT_STATUS_INPUT_ACCEPT_DATA);

        if (m_pInputMT == NULL) {
            // The Input type is not set
            break;
        }

        if (m_pOutputMT == NULL) {
            // The output type is not set
            break;
        }

        m_dwStatus |= MYMFT_STATUS_INPUT_ACCEPT_DATA; // The MFT is ready for data

        bReady = TRUE;
    } while (false);

    MFT_METHOD_END("%s", bReady ? "True" : "False");
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(%s)", __FUNCTION__, bReady ? L"True" : L"False");

    return bReady;
}

HRESULT CMaloneMft::UpdateSampleTime(ULONG ulSampleIdx, IMFSample *pSample)
{
    HRESULT hr = S_OK;
    LONGLONG llSampleDuration;

    do {
        CAutoLock lock(&m_csLock);

        if (m_bFirstSample != FALSE) {
            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Sample %u @%p is discontinuity", __FUNCTION__, ulSampleIdx,
                        pSample);

            hr = pSample->SetUINT32(MFSampleExtension_Discontinuity, TRUE);
            if (FAILED(hr)) {
                break;
            }

            m_bFirstSample = FALSE;
        }

        if (FAILED(pSample->GetSampleDuration(&llSampleDuration))) {
            llSampleDuration = MFT_DEFAULT_SAMPLE_DURATION;
        }
// TODO: Can use this to slow down playback.
//        m_llNextSampleTime += llSampleDuration * 4;
        m_llNextSampleTime += llSampleDuration;
    } while (FALSE);

    return hr;
}

HRESULT CMaloneMft::CreateOutputSample(ULONG ulSampleIdx, IMFSample *pInputSample, IMFSample **pOutputSample)
{
    HRESULT hr = S_OK;
    IMFSample *newSample = nullptr;
    LONGLONG llSampleTime;
    LONGLONG llSampleDuration;

    do {
        hr = MFCreateSample(&newSample);
        if (FAILED(hr)) {
            break;
        }

        hr = DuplicateAttributes(newSample, pInputSample);
        if (FAILED(hr)) {
            break;
        }

        if (FAILED(pInputSample->GetSampleDuration(&llSampleDuration))) {
            llSampleDuration = MFT_DEFAULT_SAMPLE_DURATION;
        }

        hr = newSample->SetSampleDuration(llSampleDuration);
        if (FAILED(hr)) {
            break;
        }

        {
            CAutoLock lock(&m_csLock);

            if (m_llNextSampleTime == 0 || IsKeyFrame(pInputSample)) {
                // TODO: this is using the DTS (decode timestamp) value to set the PTS
                // (presentation timestamp value).  They are not necessarily the same.
                if (SUCCEEDED(pInputSample->GetSampleTime(&llSampleTime))) {
                    m_llNextSampleTime = llSampleTime;
                }
            }
            llSampleTime = m_llNextSampleTime;
        }

        hr = newSample->SetSampleTime(llSampleTime);
        if (FAILED(hr)) {
            break;
        }

        *pOutputSample = newSample;
        newSample = nullptr;

        struct _timespec64 clocktime;
        GetElapsedTime(&m_clockStart, &clocktime);
        MFT_METHOD_INFO("Output Sample %u @%p Created: Duration %I64u, Sample Time %I64d.%04I64d, Clock Time %lld.%04d",
                        ulSampleIdx, *pOutputSample, llSampleDuration / MFT_MILLISECOND_TIMEBASE,
                        (llSampleTime / MFT_MILLISECOND_TIMEBASE) / 1000, (llSampleTime / MFT_MILLISECOND_TIMEBASE) % 1000,
                        clocktime.tv_sec, clocktime.tv_nsec / 1000000);
        TraceString(CHMFTTracing::TRACE_INFORMATION,
                    L"%S(): Output Sample %u @%p Created: Duration %I64u, Sample Time %I64d.%04I64d, Clock Time %d.%04d",
                    __FUNCTION__, ulSampleIdx, *pOutputSample, llSampleDuration / MFT_MILLISECOND_TIMEBASE,
                    (llSampleTime / MFT_MILLISECOND_TIMEBASE) / 1000, (llSampleTime / MFT_MILLISECOND_TIMEBASE) % 1000,
                    clocktime.tv_sec, clocktime.tv_nsec / 1000000);


    } while (FALSE);

    SAFERELEASE(newSample);

    return hr;
}

int CMaloneMft::Align(int i, unsigned int align)
{
    return ((i + (align - 1)) & ~(align - 1));
    //return ((i + (align - 1)) / align) * align;
}

void *CMaloneMft::Align(void *ptr, unsigned int align)
{
    return (void *)(((uintptr_t)ptr + ((uintptr_t)align - 1)) & ~((uintptr_t)align - 1));
    //return (void*)((((intptr_t)ptr + (align - 1)) / align) * align);
}

BOOL CMaloneMft::IsKeyFrame(IMFSample *pSample)
{
    UINT32 value = 0;

    if (pSample->GetUINT32(MFSampleExtension_CleanPoint, &value) == S_OK) {
        return !!value;
    }
    return false;
}

BOOL CMaloneMft::IsInterlaced(UINT32 interlaceMode)
{
    switch (interlaceMode) {
        case MFVideoInterlace_FieldInterleavedUpperFirst:
        case MFVideoInterlace_FieldInterleavedLowerFirst:
        case MFVideoInterlace_FieldSingleUpper:
        case MFVideoInterlace_FieldSingleLower:
            return TRUE;

        // video stream defines interlaced/progressive setting
        case MFVideoInterlace_MixedInterlaceOrProgressive:
            return TRUE;

        case MFVideoInterlace_Progressive:
        case MFVideoInterlace_Unknown:
        default:
            return FALSE;
    }
}

HRESULT DuplicateAttributes(
    IMFAttributes  *pDest,
    IMFAttributes  *pSource)
{
    HRESULT     hr      = S_OK;
    GUID        guidKey = GUID_NULL;
    PROPVARIANT pv      = {0};

    //MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        if ((pDest == NULL) || (pSource == NULL)) {
            hr = E_POINTER;
            break;
        }

        PropVariantInit(&pv);

        for (UINT32 un32Index = 0; true; un32Index++) {
            PropVariantClear(&pv);
            PropVariantInit(&pv);

            hr = pSource->GetItemByIndex(un32Index, &guidKey, &pv);
            if (FAILED(hr)) {
                if (hr == E_INVALIDARG) {
                    // all items copied
                    hr = S_OK;
                }

                break;
            }

            hr = pDest->SetItem(guidKey, pv);
            if (FAILED(hr)) {
                break;
            }
        }
    } while (false);

    PropVariantClear(&pv);

    //MFT_METHOD_END("(hr = 0x%x)", hr);
    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit(hr=0x%x)", __FUNCTION__, hr);

    return hr;
}

bool MyFailed(HRESULT hr)
{
    if (hr < 0) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void GetElapsedTime(struct _timespec64 *startTime, struct _timespec64 *elapsedTime)
{
    struct _timespec64 now;
    _timespec64_get(&now, TIME_UTC);

    elapsedTime->tv_nsec = now.tv_nsec - startTime->tv_nsec;
    elapsedTime->tv_sec = now.tv_sec - startTime->tv_sec;

    if (now.tv_nsec - startTime->tv_nsec < 0) {
        elapsedTime->tv_nsec += 1000000000;
        elapsedTime->tv_sec -= 1;
    }
}
