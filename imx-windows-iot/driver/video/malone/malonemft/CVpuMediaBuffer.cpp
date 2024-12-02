/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "CAutoLock.h"
#include "malonemft.h"
//#include <tracing.h>
#include "malonemft_DebugLogger.h"
#include "dwl.h"

//extern bool VpuFailed(VpuDecRetCode result);
#if 0
    #define _MFT_BUFFER_LOG
    #define _MFT_METHOD_BEG
    #define _MFT_METHOD_END
#endif
static char mftmsgbuf[256];
static int len = 0;
static _timespec64 mytime = { 0 };

#ifdef _MFT_BUFFER_LOG
#define MFT_BUFFER_LOG(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CVpuMediaBuffer: %s()" _format_str_ "\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_BUFFER_LOG(...)
#endif

#ifdef _MFT_METHOD_BEG
#define MFT_METHOD_BEG() \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CVpuMediaBuffer:+++%s()\n",  (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_METHOD_BEG(...)
#endif

#ifdef _MFT_METHOD_END
#define MFT_METHOD_END(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CVpuMediaBuffer:---%s() " _format_str_ "\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define MFT_METHOD_END(...)
#endif

static void *Align(void *ptr, unsigned int align)
{
    return (void *)((((intptr_t)ptr + (align - 1)) / align) * align);
}

ULONG CVpuMediaBuffer::AddRef(void)
{
    return InterlockedIncrement(&m_ulRef);
}

HRESULT CVpuMediaBuffer::QueryInterface(
    REFIID riid,
    void **ppvObject)
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();
    do {
        if (ppvObject == NULL) {
            hr = E_POINTER;
            break;
        }

        if (riid == IID_IUnknown) {
            *ppvObject = (IUnknown *)this;
        }
        if (riid == IID_IMFMediaBuffer) {
            *ppvObject = (IMFMediaBuffer *)this;
        } else {
            *ppvObject = NULL;
            hr = E_NOINTERFACE;
            break;
        }

        AddRef();
    } while (false);
    MFT_METHOD_END("");
    return hr;
}

ULONG CVpuMediaBuffer::Release(void)
{
    ULONG ulRef = 0;
    MFT_METHOD_BEG();
    if (m_ulRef > 0) {
        ulRef = InterlockedDecrement(&m_ulRef);
    }

    if (ulRef == 0) {
        m_Collection->MarkBufferFree(this);
        delete this;
    }
    MFT_METHOD_END("");
    return ulRef;
}

HRESULT CVpuMediaBuffer::Lock(
    BYTE **ppbBuffer,  // [OutPtr]
    DWORD *pcbMaxLength,  // [Out optional]
    DWORD *pcbCurrentLength) // [Out optional]

{
    HRESULT hr          = S_OK;
    MFT_METHOD_BEG();
    TraceString(CHMFTTracing::TRACE_MEMORY, L"%S(): Enter",  __FUNCTION__);
    MFT_BUFFER_LOG("Enter");

    do {
        if (ppbBuffer == nullptr) {
            hr = E_POINTER;
            break;
        }

        (*ppbBuffer) = (unsigned char *)m_pVpuFrameBuffer.planes[0];

        if (pcbMaxLength != nullptr) {
            *pcbMaxLength = (DWORD)m_MaxSize;
        }

        if (pcbCurrentLength != nullptr) {
            *pcbCurrentLength = (DWORD)m_Size;
        }
    } while (false);

    TraceString(CHMFTTracing::TRACE_MEMORY, L"%S(): Exit (hr=0x%x)",  __FUNCTION__, hr);
    MFT_BUFFER_LOG("Exit (hr=0x%x)", hr);
    MFT_METHOD_END("");
    return hr;
}


HRESULT CVpuMediaBuffer::Unlock(void)
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();

    MFT_METHOD_END("");
    return hr;
}

HRESULT CVpuMediaBuffer::GetCurrentLength(
    DWORD *pcbCurrentLength) // [out]
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();
    if (pcbCurrentLength == NULL) {
        MFT_METHOD_END("");
        return E_POINTER;
    }

    *pcbCurrentLength = (DWORD)m_Size;
    MFT_METHOD_END("");
    return hr;
}


HRESULT CVpuMediaBuffer::SetCurrentLength(
    DWORD cbCurrentLength)
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();
    if (cbCurrentLength <= m_MaxSize) {
        m_Size = cbCurrentLength;
    } else {
        hr = E_INVALIDARG;
    }
    MFT_METHOD_END("");
    return hr;
}

HRESULT CVpuMediaBuffer::GetMaxLength(
    DWORD *pcbMaxLength) // [out]
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();
    if (pcbMaxLength == NULL) {
        return E_POINTER;
    }

    *pcbMaxLength = (DWORD)m_MaxSize;
    MFT_METHOD_END("");
    return hr;
}


CVpuMediaBuffer::CVpuMediaBuffer(void) :
    m_Collection(nullptr), m_ulRef(1), m_Size(0), m_MaxSize(0), m_vpuHandle(nullptr)
{
    MFT_METHOD_BEG();
    InitializeCriticalSection(&m_csLock);
    MFT_METHOD_END("");
}

CVpuMediaBuffer::CVpuMediaBuffer(
    IVpuBufferCollection *collection,
    IDeviceIoControl  *vpuHandle,
    UINT32 maxSize,
    fbo_t frameBuffer) :
    m_Collection(collection), m_ulRef(1), m_Size(0), m_MaxSize(maxSize), m_vpuHandle(vpuHandle),
    m_pVpuFrameBuffer(frameBuffer), m_BufferAcquire({ 0 }), m_BufferRelease({ 0 })
{
    MFT_METHOD_BEG();
    InitializeCriticalSection(&m_csLock);
    _timespec64_get(&m_BufferAcquire, TIME_UTC);
    m_Collection->AddRef();
    MFT_METHOD_END("");
}

CVpuMediaBuffer::~CVpuMediaBuffer(void)
{
    MFT_METHOD_BEG();
    if (m_vpuHandle != nullptr) {
        GetElapsedTime(&m_BufferAcquire, &m_BufferRelease);
        LONGLONG bufferHoldTime = (LONGLONG)m_BufferRelease.tv_sec * 1000000LL + (LONGLONG)m_BufferRelease.tv_nsec / 1000LL;
        NTSTATUS status = decoder_clear_output(m_vpuHandle, &m_pVpuFrameBuffer.index);
        MFT_BUFFER_LOG("Collection: MFT Hold buffer for  us = %lld", bufferHoldTime);
    }
    m_pVpuFrameBuffer = { 0 };
    SAFERELEASE(m_Collection);
    DeleteCriticalSection(&m_csLock);
    MFT_METHOD_END("");
}

//
// CvpuBufferCollection class implementation
//

CVpuBufferCollection::CVpuBufferCollection(IMaloneMft *mft, ULONG totalBuffers, ULONG minBuffers) :
    m_ulRef(0), m_Mft(mft), m_minVpuBuffers(minBuffers), m_totalBuffers(totalBuffers), m_inUse(0), m_maxInUse(0),
    m_deferredFrameDecodes(0), m_freeBuffers(totalBuffers), m_scheduleError(S_OK)
{
    MFT_METHOD_BEG();
    InitializeCriticalSection(&m_csLock);
    MFT_METHOD_END("");
}

CVpuBufferCollection::~CVpuBufferCollection()
{
    DeleteCriticalSection(&m_csLock);
}

HRESULT CVpuBufferCollection::CreateInstance(
    _In_ IMaloneMft *vpuMft,
    _In_ ULONG totalBuffers,
    _In_ ULONG minVpuBuffers,
    _Out_ IVpuBufferCollection **collection)
{
    MFT_METHOD_BEG();
    *collection = new CVpuBufferCollection(vpuMft, totalBuffers, minVpuBuffers);

    (*collection)->AddRef();
    MFT_METHOD_END("");
    return S_OK;
}

HRESULT CVpuBufferCollection::QueryInterface(
    REFIID riid,
    void **ppvObject)
{
    MFT_METHOD_BEG();
    HRESULT hr = S_OK;

    do {
        if (ppvObject == NULL) {
            hr = E_POINTER;
            break;
        }

        if (riid == IID_IUnknown) {
            *ppvObject = (IUnknown *)this;
        }
        if (riid == IID_IVpuBufferCollection) {
            *ppvObject = (IVpuBufferCollection *)this;
        } else {
            *ppvObject = NULL;
            hr = E_NOINTERFACE;
            break;
        }

        AddRef();
    } while (false);
    MFT_METHOD_END("");
    return hr;
}

ULONG CVpuBufferCollection::AddRef(void)
{
    return InterlockedIncrement(&m_ulRef);
}

ULONG CVpuBufferCollection::Release(void)
{
    MFT_METHOD_BEG();
    ULONG ulRef = 0;

    if (m_ulRef > 0) {
        ulRef = InterlockedDecrement(&m_ulRef);
    }

    if (ulRef == 0) {
        delete this;
    }
    MFT_METHOD_END("");
    return ulRef;
}

HRESULT CVpuBufferCollection::CreateBufferInstance(
    IDeviceIoControl *vpuHandle, // [in]
    UINT32 maxLength,       // [in]
    fbo_t frameBuffer,    // [in]
    IMFMediaBuffer **ppBuffer)  // [out]
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();
    CVpuMediaBuffer *pNewBuffer = NULL;

    do {
        if (ppBuffer == NULL) {
            hr = E_POINTER;
            break;
        }
#if 0 /* No pointer anymore */
        if (frameBuffer == NULL) {
            hr = E_INVALIDARG;
            break;
        }
#endif
        pNewBuffer = new CVpuMediaBuffer(this, vpuHandle, maxLength, frameBuffer);
        if (pNewBuffer == NULL) {
            hr = E_OUTOFMEMORY;
            break;
        }

        (*ppBuffer) = pNewBuffer;
        (*ppBuffer)->AddRef();

        ULONG instances;

        instances = InterlockedIncrement(&m_inUse);
        if (instances > m_maxInUse) {
            InterlockedIncrement(&m_maxInUse);
            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): max buffers in use has grown to %d",
                        __FUNCTION__, instances);
            MFT_BUFFER_LOG("max buffers in use has grown to %d", instances);

        }

        ULONG free;

        free = InterlockedDecrement(&m_freeBuffers);
        if (free < m_minVpuBuffers - 1) {
            // We warn if we dip more than one into the reserved vpu buffer space
            TraceString(CHMFTTracing::TRACE_INFORMATION,
                        L"%S(): buffer collection is dipping into vpu buffer reserved space. inuse = %d, total = %d, reserved = %d",
                        __FUNCTION__, instances, m_totalBuffers, m_minVpuBuffers);
            MFT_BUFFER_LOG("buffer collection is dipping into vpu buffer reserved space. inuse = %d, total = %d, reserved = %d",
                           instances, m_totalBuffers, m_minVpuBuffers);
        }
    } while (false);

    SAFERELEASE(pNewBuffer);
    MFT_METHOD_END("");
    return hr;
}

HRESULT CVpuBufferCollection::MarkBufferFree(
    _In_ IMFMediaBuffer *buffer)
{
    ULONG deferred;
    MFT_METHOD_BEG();
    InterlockedDecrement(&m_inUse);
    InterlockedIncrement(&m_freeBuffers);

    {
        CAutoLock lock(&m_csLock);

        deferred = m_deferredFrameDecodes;
        if (deferred > 0) {
            HRESULT hr;

            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Queuing deferring frame decode. deferred = %d",
                        __FUNCTION__, m_freeBuffers, m_minVpuBuffers, deferred);
            MFT_BUFFER_LOG("Queuing deferring frame decode. m_freeBuffers=%d, m_minVpuBuffers=%d, deferred = %d",  m_freeBuffers,
                           m_minVpuBuffers, deferred);
            hr = m_Mft->ScheduleFrameDecode();
            if (FAILED(hr)) {
                TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): Failed to queue deferred frame decode (hr=0x%x)", __FUNCTION__, hr);
                MFT_BUFFER_LOG("Failed to queue deferred frame decode (hr=0x%x)", hr);

                m_scheduleError = hr;
            } else {
                InterlockedDecrement(&m_deferredFrameDecodes);
            }
        }
    }
    MFT_METHOD_END("");
    return S_OK;
}


HRESULT CVpuBufferCollection::ScheduleFrameDecode(void)
{
    HRESULT hr = S_OK;
    MFT_METHOD_BEG();
    if (m_freeBuffers > m_minVpuBuffers) {
        hr = m_Mft->ScheduleFrameDecode();
    } else {
        ULONG deferred;

        deferred = InterlockedIncrement(&m_deferredFrameDecodes);
        TraceString(CHMFTTracing::TRACE_INFORMATION,
                    L"%S(): Deferring frame decode. Free buffers = %d, min vpu = %d, deferred = %d",
                    __FUNCTION__, m_freeBuffers, m_minVpuBuffers, deferred);
        MFT_BUFFER_LOG("Deferring frame decode. Free buffers = %d, min vpu = %d, deferred = %d",
                       m_freeBuffers, m_minVpuBuffers, deferred);
    }
    MFT_METHOD_END("");
    return hr;
}

HRESULT CVpuBufferCollection::FrameDecodeError(void)
{
    HRESULT hr;
    MFT_METHOD_BEG();
    hr = m_scheduleError;
    m_scheduleError = S_OK;
    MFT_METHOD_END("");
    return hr;
}
