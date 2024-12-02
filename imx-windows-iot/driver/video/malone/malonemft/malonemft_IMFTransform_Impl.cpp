/* Modifications Copyright 2023 NXP */
#include "malonemft.h"
#include "CAutoLock.h"
#include <mferror.h>
#include <mfapi.h>
#include <stdio.h>
#include "malonemft_DebugLogger.h"

#include "dwl.h"

#if 0
    #define _CMFT_METHOD_BEG
    #define _CMFT_METHOD_END
    #define _CMFT_METHOD_INFO
    #define _CMFT_METHOD_ERROR
#endif
static char mftmsgbuf[256];
static int len = 0;
static _timespec64 mytime = { 0 };
#ifdef _CMFT_METHOD_BEG
#define CMFT_METHOD_BEG() \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CMaloneMFT:+++%s()\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define CMFT_METHOD_BEG(...)
#endif

#ifdef _CMFT_METHOD_END
#define CMFT_METHOD_END(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CMaloneMFT:---%s() "_format_str_"\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define CMFT_METHOD_END(...)
#endif

#ifdef _CMFT_METHOD_ERROR
#define CMFT_METHOD_ERROR(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CMaloneMFT:   %s() ERROR: "_format_str_"\n",  (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define CMFT_METHOD_ERROR(...)
#endif


#ifdef _CMFT_METHOD_INFO
#define CMFT_METHOD_INFO(_format_str_,...) \
        _timespec64_get(&mytime, TIME_UTC); \
        sprintf_s(mftmsgbuf,"%llu ms CMaloneMFT:   %s() INFO: "_format_str_"\n", (LONGLONG)mytime.tv_sec * 1000 + (LONGLONG)mytime.tv_nsec / 1000000, __func__, __VA_ARGS__); \
        OutputDebugStringA(mftmsgbuf);
#else
#define CMFT_METHOD_INFO(...)
#endif


/****************************
******* IMFTransform ********
****************************/

HRESULT CMaloneMft::AddInputStreams(
    DWORD   dwStreams,
    DWORD  *pdwStreamIDs)
{
    /*****************************************
    ** Todo: If your MFT does not have a fixed
    ** number of input streams, you must implement
    ** this function, see:
    ** http://msdn.microsoft.com/en-us/library/ms696211(v=VS.85).aspx
    *****************************************/

    return E_NOTIMPL;
}

HRESULT CMaloneMft::DeleteInputStream(
    DWORD   dwStreamID)
{
    /*****************************************
    ** Todo: If your MFT does not have a fixed
    ** number of input streams, you must implement
    ** this function, see:
    ** http://msdn.microsoft.com/en-us/library/ms703159(v=VS.85).aspx
    *****************************************/

    return E_NOTIMPL;
}

HRESULT CMaloneMft::GetAttributes(
    IMFAttributes **ppAttributes)
{
    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (ppAttributes == NULL) {
            hr = E_POINTER;
            break;
        }

        (*ppAttributes) = m_pAttributes;

        if ((*ppAttributes) == NULL) {
            hr = E_UNEXPECTED;
            break;
        }

        (*ppAttributes)->AddRef();
    } while (false);

    CMFT_METHOD_END("");
    return hr;
}

HRESULT CMaloneMft::GetInputAvailableType(
    DWORD           dwInputStreamID,
    DWORD           dwTypeIndex,
    IMFMediaType  **ppType)
{
    /*****************************************
    ** Todo: This function will return a media
    ** type at a given index. The SDK
    ** implementation uses a static array of
    ** media types. Your MFT may want to use
    ** a dynamic array and modify the list
    ** order depending on the MFTs state
    ** See http://msdn.microsoft.com/en-us/library/ms704814(v=VS.85).aspx
    *****************************************/

    HRESULT         hr      = S_OK;
    IMFMediaType   *pMT     = NULL;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (ppType == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        /*****************************************
        ** Todo: Modify the accepted input list
        ** g_ppguidInputTypes or use your own
        ** implementation of this function
        *****************************************/
        if (dwTypeIndex >= g_dwNumInputTypes) {
            hr = MF_E_NO_MORE_TYPES;
            break;
        }

        {
            CAutoLock lock(&m_csLock);

            hr = MFCreateMediaType(&pMT);
            if (FAILED(hr)) {
                break;
            }

            hr = pMT->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
            if (FAILED(hr)) {
                break;
            }

            hr = pMT->SetGUID(MF_MT_SUBTYPE, *(g_ppguidInputTypes[dwTypeIndex].guid));
            if (FAILED(hr)) {
                break;
            }

            (*ppType) = pMT;
            (*ppType)->AddRef();
        }
    } while (false);

    SAFERELEASE(pMT);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetInputCurrentType(
    DWORD           dwInputStreamID,
    IMFMediaType  **ppType)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms705607(v=VS.85).aspx
    *****************************************/

    HRESULT         hr      = S_OK;
    IMFMediaType   *pMT     = NULL;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (ppType == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        {
            CAutoLock lock(&m_csLock);

            if (m_pInputMT == NULL) {
                hr = MF_E_TRANSFORM_TYPE_NOT_SET;
                break;
            }

            /*******************************************
            ** Return a copy of the media type, not the
            ** internal one. Returning the internal one
            ** will allow an external component to modify
            ** the internal media type
            *******************************************/

            hr = MFCreateMediaType(&pMT);
            if (FAILED(hr)) {
                break;
            }

            hr = DuplicateAttributes(pMT, m_pInputMT);
            if (FAILED(hr)) {
                break;
            }
        }

        (*ppType) = pMT;
        (*ppType)->AddRef();
    } while (false);

    SAFERELEASE(pMT);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetInputStatus(
    DWORD   dwInputStreamID,
    DWORD  *pdwFlags)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms697478(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pdwFlags == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        *pdwFlags = 0;

        {
            CAutoLock lock(&m_csLock);

            if ((m_dwStatus & MYMFT_STATUS_INPUT_ACCEPT_DATA) != 0) {
                *pdwFlags = MFT_INPUT_STATUS_ACCEPT_DATA;
            }
        }
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetInputStreamAttributes(
    DWORD           dwInputStreamID,
    IMFAttributes **ppAttributes)
{
    /*****************************************
    ** Todo: Becuase this MFT is acting as a decoder
    ** There will not be an upstream HW MFT.
    ** As such, no input stream attributes
    ** are required.
    ** See http://msdn.microsoft.com/en-us/library/ms695366(v=VS.85).aspx,
    ** http://msdn.microsoft.com/en-us/library/dd940330(VS.85).aspx#handshake
    *****************************************/

    return E_NOTIMPL;
}

HRESULT CMaloneMft::GetInputStreamInfo(
    DWORD                   dwInputStreamID,
    MFT_INPUT_STREAM_INFO  *pStreamInfo)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms703894(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pStreamInfo == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        pStreamInfo->hnsMaxLatency  = 0; // All input is turned directly into output
        pStreamInfo->dwFlags        = MFT_INPUT_STREAM_SINGLE_SAMPLE_PER_BUFFER | MFT_INPUT_STREAM_WHOLE_SAMPLES;
        pStreamInfo->cbSize         = 0; // No minimum size is required
        pStreamInfo->cbMaxLookahead = 0; // No lookahead is performed
        pStreamInfo->cbAlignment    = 0; // No memory allignment is required
    } while (false);

    CMFT_METHOD_END("");
    return hr;
}

HRESULT CMaloneMft::GetOutputAvailableType(
    DWORD           dwOutputStreamID,
    DWORD           dwTypeIndex,
    IMFMediaType  **ppType)
{
    /*****************************************
    ** Todo: This function will return a media
    ** type at a given index. The SDK
    ** implementation uses a static array of
    ** media types. Your MFT may want to use
    ** a dynamic array and modify the list
    ** order depending on the MFTs state
    ** See http://msdn.microsoft.com/en-us/library/ms703812(v=VS.85).aspx
    *****************************************/

    HRESULT         hr      = S_OK;
    IMFMediaType   *pMT     = NULL;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (ppType == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwOutputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        /*****************************************
        ** Todo: Modify the accepted output list
        ** g_ppguidOutputTypes or use your own
        ** implementation of this function
        *****************************************/
        if (dwTypeIndex >= g_dwNumOutputTypes) {
            hr = MF_E_NO_MORE_TYPES;
            break;
        }

        {
            CAutoLock lock(&m_csLock);

            hr = MFCreateMediaType(&pMT);
            if (FAILED(hr)) {
                break;
            }

            hr = pMT->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
            if (FAILED(hr)) {
                break;
            }

            hr = pMT->SetGUID(MF_MT_SUBTYPE, *(g_ppguidOutputTypes[dwTypeIndex]));
            if (FAILED(hr)) {
                break;
            }

            /*****************************************
            ** Todo: The following implementation
            ** forces a standard output resolution
            ** and framerate. Your MFT should set these
            ** values properly and update the Media
            ** Type as necessary after decoding the
            ** stream
            *****************************************/
            if ((m_inputHeight == 0) || (m_inputWidth == 0)) {
                m_outputWidth = MFT_OUTPUT_WIDTH;
                m_outputHeight = MFT_OUTPUT_HEIGHT;
            } else {
                m_outputWidth = Align(m_inputWidth, 16);
                m_outputHeight = Align(m_inputHeight, (m_bInterlaced ? 32 : 16));
            }
            hr = MFSetAttributeSize(pMT, MF_MT_FRAME_SIZE, m_outputWidth, m_outputHeight);
            if (FAILED(hr)) {
                break;
            }

            // Set the display aperture to handle overscan due to output width and height alignment adjustments.
            MFVideoArea displayAperture = { 0 };

            displayAperture.Area.cy = m_inputHeight;
            displayAperture.Area.cx = m_inputWidth;
            hr = pMT->SetBlob(MF_MT_MINIMUM_DISPLAY_APERTURE, (UINT8 *)&displayAperture, sizeof(displayAperture));
            if (FAILED(hr)) {
                break;
            }

            if ((m_fps.Denominator != 0) && (m_fps.Numerator != 0)) {
                hr = MFSetAttributeRatio(pMT, MF_MT_FRAME_RATE, m_fps.Numerator, m_fps.Denominator);
            }
            if (FAILED(hr)) {
                break;
            }

            (*ppType) = pMT;
            (*ppType)->AddRef();
        }
    } while (false);

    SAFERELEASE(pMT);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetOutputCurrentType(
    DWORD           dwOutputStreamID,
    IMFMediaType  **ppType)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms696985(v=VS.85).aspx
    *****************************************/

    HRESULT         hr      = S_OK;
    IMFMediaType   *pMT     = NULL;
    CMFT_METHOD_BEG();

    do {
        /************************************
        ** Since this MFT is a decoder, it
        ** must not allow this function to be
        ** called until it is unlocked. If
        ** your MFT is an encoder, this function
        ** CAN be called before the MFT is
        ** unlocked
        ************************************/
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (ppType == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwOutputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        {
            CAutoLock lock(&m_csLock);

            if (m_pOutputMT == NULL) {
                hr = MF_E_TRANSFORM_TYPE_NOT_SET;
                break;
            }

            /*******************************************
            ** Return a copy of the media type, not the
            ** internal one. Returning the internal one
            ** will allow an external component to modify
            ** the internal media type
            *******************************************/

            hr = MFCreateMediaType(&pMT);
            if (FAILED(hr)) {
                break;
            }

            hr = DuplicateAttributes(pMT, m_pOutputMT);
            if (FAILED(hr)) {
                break;
            }
        }

        (*ppType) = pMT;
        (*ppType)->AddRef();
    } while (false);

    SAFERELEASE(pMT);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetOutputStatus(
    DWORD  *pdwFlags)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms696269(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pdwFlags == NULL) {
            hr = E_POINTER;
            break;
        }

        (*pdwFlags) = 0;

        {
            CAutoLock lock(&m_csLock);

            // TODO: Should reset m_dwStatus when output is retreived?
            if ((m_dwStatus & MYMFT_STATUS_OUTPUT_SAMPLE_READY) != 0) {
                *pdwFlags = MFT_OUTPUT_STATUS_SAMPLE_READY;
            }
        }

        TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Output Status Flags: 0x%x",  __FUNCTION__, (*pdwFlags));
    } while (false);

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit (hr=0x%x)",  __FUNCTION__, hr);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetOutputStreamAttributes(
    DWORD           dwOutputStreamID,
    IMFAttributes **ppAttributes)
{
    /*****************************************
    ** Todo: This MFT does not support a
    ** hardware handshake, so this function
    ** is not implemented
    ** See http://msdn.microsoft.com/en-us/library/ms703886(v=VS.85).aspx,
    ** http://msdn.microsoft.com/en-us/library/dd940330(VS.85).aspx#handshake
    *****************************************/

    return E_NOTIMPL;
}

HRESULT CMaloneMft::GetOutputStreamInfo(
    DWORD                   dwOutputStreamID,
    MFT_OUTPUT_STREAM_INFO *pStreamInfo)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms693880(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pStreamInfo == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwOutputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        pStreamInfo->dwFlags        =   MFT_OUTPUT_STREAM_WHOLE_SAMPLES             |
                                        MFT_OUTPUT_STREAM_SINGLE_SAMPLE_PER_BUFFER  |
                                        MFT_OUTPUT_STREAM_FIXED_SAMPLE_SIZE         |
                                        MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES;
        pStreamInfo->cbSize         = (MFT_OUTPUT_WIDTH * MFT_OUTPUT_HEIGHT) * 4; // Since the MFT can output RGB32,
        // it may need as many as 4 bytes
        // per pixel, so the output buffer
        // size must be set accordinly
        // Todo: Change this value depending
        // On the current output type
        pStreamInfo->cbAlignment    = 0; // No memory allignment is required
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetStreamCount(
    DWORD  *pdwInputStreams,
    DWORD  *pdwOutputStreams)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms697018(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if ((pdwInputStreams == NULL) || (pdwOutputStreams == NULL)) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/

        *pdwInputStreams = MFT_MAX_STREAMS;
        *pdwOutputStreams = MFT_MAX_STREAMS;
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetStreamIDs(
    DWORD   dwInputIDArraySize,
    DWORD  *pdwInputIDs,
    DWORD   dwOutputIDArraySize,
    DWORD  *pdwOutputIDs)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms693988(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if ((pdwInputIDs == NULL) || (pdwOutputIDs == NULL)) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/

        if ((dwInputIDArraySize < MFT_MAX_STREAMS) || (dwOutputIDArraySize < MFT_MAX_STREAMS)) {
            hr = MF_E_BUFFERTOOSMALL;
            break;
        }

        pdwInputIDs[0]  = 0;
        pdwOutputIDs[0] = 0;
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::GetStreamLimits(
    DWORD  *pdwInputMinimum,
    DWORD  *pdwInputMaximum,
    DWORD  *pdwOutputMinimum,
    DWORD  *pdwOutputMaximum)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms697040(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if ((pdwInputMinimum == NULL) || (pdwInputMaximum == NULL) ||
            (pdwOutputMinimum == NULL) || (pdwOutputMaximum == NULL)) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/

        *pdwInputMinimum    = MFT_MAX_STREAMS;
        *pdwInputMaximum    = MFT_MAX_STREAMS;
        *pdwOutputMinimum   = MFT_MAX_STREAMS;
        *pdwOutputMaximum   = MFT_MAX_STREAMS;
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::ProcessEvent(
    DWORD           dwInputStreamID,
    IMFMediaEvent  *pEvent)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms695394(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pEvent == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        /****************************************
        ** Todo: this MFT does not handle any
        ** events. It allows them all to be
        ** propagated downstream. If your MFT
        ** needs to handle events, implement this
        ** function
        ****************************************/
        hr = E_NOTIMPL;
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::ProcessInput(
    DWORD       dwInputStreamID,
    IMFSample  *pSample,
    DWORD       dwFlags)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms703131(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (dwFlags != 0) {
            MyFailed(E_UNEXPECTED);
        }

        {
            CAutoLock lock(&m_csLock);

            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): NeedInputCount: %u",  __FUNCTION__, m_dwNeedInputCount);

            if (m_dwNeedInputCount == 0) {
                // This call does not correspond to a need input call
                hr = MF_E_NOTACCEPTING;
                break;
            } else {
                m_dwNeedInputCount--;
            }
        }

        if (pSample == NULL) {
            hr = E_POINTER;
            break;
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        // First, put sample into the input Queue

        /***************************************
        ** Since this in an internal function
        ** we know m_pInputSampleQueue can never be
        ** NULL due to InitializeTransform()
        ***************************************/
        {
            CAutoLock lock(&m_csLock);
            InterlockedIncrement(&m_ulInputSamplesAdded);
        }

        hr = m_pInputSampleQueue->AddSample(pSample);
        if (FAILED(hr)) {
            break;
        }

        UINT32 inputWidth = 0;
        UINT32 inputHeight = 0;
        UINT32 inputNumerator = 0;
        UINT32 inputDenominator = 0;
        UINT32 interlaceMode = MFVideoInterlace_Unknown;

        if (SUCCEEDED(MFGetAttributeSize(pSample, MF_MT_FRAME_SIZE, &inputWidth, &inputHeight))) {
            CAutoLock lock(&m_csLock);

            if ((m_inputHeight == 0) || (m_inputWidth == 0)) {
                m_inputHeight = inputHeight;
                m_inputWidth = inputWidth;

                m_outputWidth = Align(m_inputWidth, 16);
                m_outputHeight = Align(m_inputHeight, (m_bInterlaced ? 32 : 16));
            }
        }

        if (SUCCEEDED(MFGetAttributeRatio(pSample, MF_MT_FRAME_RATE, &inputNumerator, &inputDenominator))) {
            CAutoLock lock(&m_csLock);

            if ((m_fps.Numerator == 0) || (m_fps.Denominator == 0)) {
                m_fps.Numerator = inputNumerator;
                m_fps.Denominator = inputDenominator;
            }
        }

        if (SUCCEEDED(pSample->GetUINT32(MF_MT_INTERLACE_MODE, &interlaceMode))) {
            CAutoLock lock(&m_csLock);

            if (m_uiInterlaceMode != interlaceMode) {
                m_uiInterlaceMode = interlaceMode;
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): interlaceMode changed to %d",  __FUNCTION__, interlaceMode);
            }
        }

        // Now schedule the work to decode the sample
        if (m_bufferCollection != nullptr) {
            hr = m_bufferCollection->FrameDecodeError();
            if (FAILED(hr)) {
                TraceString(CHMFTTracing::TRACE_ERROR, L"%S(): buffer collection has deferred FrameDecode return error (hr=0x%x)",
                            __FUNCTION__, hr);
                break;
            }
            hr = m_bufferCollection->ScheduleFrameDecode();
        } else {
            hr = ScheduleFrameDecode();
        }

        if (FAILED(hr)) {
            break;
        }
    } while (false);

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit (hr=0x%x)",  __FUNCTION__, hr);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::ProcessMessage(
    MFT_MESSAGE_TYPE eMessage,
    ULONG_PTR ulParam)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms701863(v=VS.85).aspx
    *****************************************/

    HRESULT hr = S_OK;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if ((m_pInputMT == NULL) || (m_pOutputMT == NULL)) {
            // Can't process messages until media types are set
            hr = MF_E_TRANSFORM_TYPE_NOT_SET;
            break;
        }

        switch (eMessage) {
            case MFT_MESSAGE_NOTIFY_START_OF_STREAM: {
                hr = OnStartOfStream();
                if (FAILED(hr)) {
                    break;
                }
            }
            break;
            case MFT_MESSAGE_NOTIFY_END_OF_STREAM: {
                hr = OnEndOfStream();
                if (FAILED(hr)) {
                    break;
                }
            }
            break;
            case MFT_MESSAGE_COMMAND_DRAIN: {
                hr = OnDrain((UINT32)ulParam);
                if (FAILED(hr)) {
                    break;
                }
            }
            break;
            case MFT_MESSAGE_COMMAND_FLUSH: {
                hr = OnFlush();
                if (FAILED(hr)) {
                    break;
                }
            }
            break;
            case MFT_MESSAGE_COMMAND_MARKER: {
                hr = OnMarker(ulParam);
                if (FAILED(hr)) {
                    break;
                }
            }
            break;
            /************************************************
            ** Todo: Add any MFT Messages that are not already
            ** covered
            ************************************************/
            default:
                // Nothing to do, return S_OK
                break;
        };
    } while (false);

    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::ProcessOutput(
    DWORD                   dwFlags,
    DWORD                   dwOutputBufferCount,
    MFT_OUTPUT_DATA_BUFFER *pOutputSamples,
    DWORD                  *pdwStatus)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms704014(v=VS.85).aspx
    *****************************************/

    HRESULT     hr      = S_OK;
    IMFSample  *pSample = NULL;
    CMFT_METHOD_BEG();

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter",  __FUNCTION__);

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (dwFlags != 0) {
            MyFailed(E_UNEXPECTED);
        }

        {
            CAutoLock lock(&m_csLock);

            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): HaveOutputCount: %u",  __FUNCTION__, m_dwHaveOutputCount);

            if (m_dwHaveOutputCount == 0) {
                // This call does not correspond to a have output call
                hr = E_UNEXPECTED;
                break;
            } else {
                m_dwHaveOutputCount--;
            }

            if (m_dwNeedInputCount == 0) {
                if (!(m_dwStatus & MYMFT_STATUS_DRAINING)) {
                    hr = RequestSample(0);
                    if (FAILED(hr)) {
                        break;
                    }
                }
            }
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwOutputBufferCount < MFT_MAX_STREAMS) {
            hr = E_INVALIDARG;
            break;
        }

        if (IsMFTReady() == FALSE) {
            hr = MF_E_TRANSFORM_TYPE_NOT_SET;
            break;
        }

        /***************************************
        ** Since this in an internal function
        ** we know m_pOutputSampleQueue can never be
        ** NULL due to InitializeTransform()
        ***************************************/
        hr = m_pOutputSampleQueue->GetNextSample(&pSample);
        if (FAILED(hr)) {
            break;
        }

        if (pSample == NULL) {
            hr = MF_E_TRANSFORM_NEED_MORE_INPUT;
            break;
        }

        /*******************************
        ** Todo: This MFT only has one
        ** input stream, so the output
        ** samples array and stream ID
        ** will only use the first
        ** member
        *******************************/
        pOutputSamples[0].dwStreamID    = 0;

        if ((pOutputSamples[0].pSample) == NULL) {
            // The MFT is providing it's own samples
            (pOutputSamples[0].pSample)   = pSample;
            (pOutputSamples[0].pSample)->AddRef();
        } else {
            // The pipeline has allocated the samples
            IMFMediaBuffer *pBuffer = NULL;

            do {
                hr = pSample->ConvertToContiguousBuffer(&pBuffer);
                if (FAILED(hr)) {
                    break;
                }

                pOutputSamples[0].pSample->RemoveAllBuffers();

                hr = (pOutputSamples[0].pSample)->AddBuffer(pBuffer);
                if (FAILED(hr)) {
                    break;
                }
            } while (false);

            SAFERELEASE(pBuffer);

            if (FAILED(hr)) {
                break;
            }
        }

        // We're out of samples in the output queue
        if (m_pOutputSampleQueue->IsQueueEmpty() != FALSE) {
            CAutoLock lock(&m_csLock);

            // If we're draining, send the drain complete event
            if ((m_dwStatus & MYMFT_STATUS_DRAINING) != 0 && m_dwPendingFrameDecodeCount == 0) {
                hr = SendDrainCompleteEvent();
                if (FAILED(hr)) {
                    break;
                }
            }
        }
    } while (false);

    SAFERELEASE(pSample);

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit (hr=0x%x)",  __FUNCTION__, hr);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::SendDrainCompleteEvent()
{
    HRESULT hr = S_OK;
    IMFMediaEvent  *pDrainCompleteEvent = NULL;

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Enter", __FUNCTION__);
    CMFT_METHOD_BEG();
    do {
        hr = MFCreateMediaEvent(METransformDrainComplete, GUID_NULL, S_OK, NULL, &pDrainCompleteEvent);
        if (FAILED(hr)) {
            break;
        }

        hr = pDrainCompleteEvent->SetUINT32(MF_EVENT_MFT_INPUT_STREAM_ID, 0);
        if (FAILED(hr)) {
            break;
        }

        hr = m_pEventQueue->QueueEvent(pDrainCompleteEvent);
        if (FAILED(hr)) {
            break;
        }
    } while (false);

    SAFERELEASE(pDrainCompleteEvent);

    if (!FAILED(hr)) {
        m_dwStatus &= (~MYMFT_STATUS_DRAINING);
    }

    TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): Exit (hr=0x%x)", __FUNCTION__, hr);
    CMFT_METHOD_END("");
    return hr;
}

HRESULT CMaloneMft::SetInputType(
    DWORD           dwInputStreamID,
    IMFMediaType   *pType,
    DWORD           dwFlags)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms700113(v=VS.85).aspx
    *****************************************/

    HRESULT         hr      = S_OK;
    IMFMediaType   *pMT     = NULL;
    UINT32          fourcc = 0;
    CMFT_METHOD_BEG();

    do {
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pType == NULL) {
            hr = E_POINTER;
            break;
        }

        if (dwFlags != 0) {
            MyFailed(E_UNEXPECTED);
        }

        if (dwInputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        hr = CheckInputType(pType, &fourcc);
        if (FAILED(hr)) {
            break;
        }

        /*******************************************
        ** Store a copy of the media type, not the
        ** one passed in by the caller. This way the
        ** caller is unable to modify the internal
        ** media type
        *******************************************/

        hr = MFCreateMediaType(&pMT);
        if (FAILED(hr)) {
            break;
        }

        hr = DuplicateAttributes(pMT, pType);
        if (FAILED(hr)) {
            break;
        }

        {
            CAutoLock lock(&m_csLock);
            UINT32 interlaceMode = MFVideoInterlace_Unknown;
            UINT32 inputWidth = 0;
            UINT32 inputHeight = 0;

            /*
             * GATHER STREAM INFO
             */
            hr = MFGetAttributeRatio(pMT, MF_MT_FRAME_RATE, (UINT32 *)&m_fps.Numerator, (UINT32 *)&m_fps.Denominator);
            if (FAILED(hr)) {
                CMFT_METHOD_INFO("Input stream MF_MT_FRAME_RATE unavailable, using defaults (hr=0x%x)", hr);
                TraceString(CHMFTTracing::TRACE_INFORMATION,
                            L"%S(): input stream MF_MT_FRAME_RATE unavailable, using defaults (hr=0x%x)", __FUNCTION__, hr);
                m_fps.Numerator = MFT_FRAMERATE_NUMERATOR;
                m_fps.Denominator = MFT_FRAMERATE_DENOMINATOR;
                hr = S_OK;
            }
            CMFT_METHOD_INFO("Input stream MF_MT_FRAME_RATE numerator=%d, denominator=%d", m_fps.Numerator, m_fps.Denominator);
            TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): input stream MF_MT_FRAME_RATE numerator=%d, denominator=%d",
                        __FUNCTION__, m_fps.Numerator, m_fps.Denominator);

            if (SUCCEEDED(pMT->GetUINT32(MF_MT_INTERLACE_MODE, &interlaceMode))) {
                CMFT_METHOD_INFO("Input stream MF_MT_INTERLACE_MODE mode=%d", interlaceMode);
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): input stream MF_MT_INTERLACE_MODE mode=%d", __FUNCTION__,
                            interlaceMode);
                m_uiInterlaceMode = interlaceMode;
                if (IsInterlaced(interlaceMode) && (interlaceMode != MFVideoInterlace_MixedInterlaceOrProgressive)) {
//                if (IsInterlaced(interlaceMode)) {
                    // input is definitely interlaced
                    m_bInterlaced = TRUE;
                }
            } else {
                CMFT_METHOD_INFO("Input stream MF_MT_INTERLACE_MODE unavailable, using defaults (hr=0x%x)", hr);
                TraceString(CHMFTTracing::TRACE_INFORMATION,
                            L"%S(): input stream MF_MT_INTERLACE_MODE unavailable, using defaults (hr=0x%x)", hr);
            }

            if (SUCCEEDED(MFGetAttributeSize(pMT, MF_MT_FRAME_SIZE, &inputWidth, &inputHeight))) {
                CMFT_METHOD_INFO("Input stream MF_MT_FRAME_SIZE Width=%d, Height=%d", inputWidth, inputHeight);
                TraceString(CHMFTTracing::TRACE_INFORMATION, L"%S(): input stream MF_MT_FRAME_SIZE Width=%d, Height=%d", inputWidth,
                            inputHeight);
                if ((m_inputHeight == 0) || (m_inputWidth == 0)) {
                    m_inputHeight = inputHeight;
                    m_inputWidth = inputWidth;

                    m_outputWidth = Align(m_inputWidth, 16);
                    m_outputHeight = Align(m_inputHeight, (m_bInterlaced ? 32 : 16));

                }
            } else {
                CMFT_METHOD_INFO("Input stream MF_MT_FRAME_SIZE unavailable, using defaults (hr=0x%x)", hr);
                TraceString(CHMFTTracing::TRACE_INFORMATION,
                            L"%S(): input stream MF_MT_FRAME_SIZE unavailable, using defaults (hr=0x%x)", __FUNCTION__, hr);
            }


            /*
             * DECODER INIT START
             */
            m_vdec_init.dis_reorder = 0;
            m_vdec_init.force_fbufs_num = 0;

            m_vdec_init.fourcc = fourcc;
            m_vdec_init.frame_mode = 1;

            if (m_vpuDecInit != 0) {
                decoder_deinit(m_vpuHandle);
                m_vpuDecInit = 0;
            }
            if (m_vpuHandle != 0) {
                decoder_close(m_vpuHandle);
                m_vpuHandle = 0;
            }

            m_vpuHandle = decoder_open();
            if (m_vpuHandle == NULL) {
                CMFT_METHOD_ERROR("decoder_init() ioctl call failed (hr=0x%x)", hr);
                TraceString(CHMFTTracing::TRACE_ERROR, L"decoder_open returned error %d", VDEC_FATAL_ERROR);
                hr = MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ACCELERATOR, VDEC_FATAL_ERROR);
                break;
            }

            hr = decoder_init(m_vpuHandle, &m_vdec_init, &m_sb_mem);
            if (FAILED(hr)) {
                hr = MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ACCELERATOR, VDEC_FATAL_ERROR);
                break;
            } else {
                CMFT_METHOD_INFO("decoder_init() returned stream buffer: stream_buffer_size(%d) stream_buffer_virt(%p) stream_buffer_phy(%p)",
                                 m_sb_mem.size,
                                 m_sb_mem.virtAddress,
                                 (void *)m_sb_mem.physAddress
                                );
                m_vpuDecInit = true;
            }
#if !defined(VPU_COPY_FRAMEBUFFER)
            hr = CVpuBufferCollection::CreateInstance(this, /*bufCount*/10, /*MinFrameBufferCount*/1, &m_bufferCollection);
            if (FAILED(hr)) {
                break;
            }
#endif
            hr = decoder_status(m_vpuHandle, &m_vdec_status);
            if (FAILED(hr)) {
                hr = MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ACCELERATOR, VDEC_FATAL_ERROR);
                CMFT_METHOD_ERROR("decoder_init() ioctl call failed (hr=0x%x)", hr);
                break;
            } else {
                m_vdec_decode.wptr = m_vdec_status.wptr;
            }

            if (m_vdec_init.fourcc == (uint32_t)AOIFOURCC("VP80")) {
                AddVP8SeqHdr(&m_vdec_decode, &m_vdec_status, m_sb_mem.virtAddress, m_inputWidth, m_inputHeight, m_fps);
            }
            SAFERELEASE(m_pInputMT);

            m_pInputMT = pMT;
            m_pInputMT->AddRef();
        }

        IsMFTReady();
    } while (false);

    SAFERELEASE(pMT);
    CMFT_METHOD_END("");

    return hr;
}

HRESULT CMaloneMft::SetOutputBounds(
    LONGLONG hnsLowerBound,
    LONGLONG hnsUpperBound)
{
    /*****************************************
    ** Todo: This MFT does not support
    ** sample boundries
    ** See http://msdn.microsoft.com/en-us/library/ms693812(v=VS.85).aspx
    *****************************************/

    return E_NOTIMPL;
}

HRESULT CMaloneMft::SetOutputType(
    DWORD           dwOutputStreamID,
    IMFMediaType   *pType,
    DWORD           dwFlags)
{
    /*****************************************
    ** See http://msdn.microsoft.com/en-us/library/ms702016(v=VS.85).aspx
    *****************************************/

    HRESULT         hr      = S_OK;
    IMFMediaType   *pMT     = NULL;
    CMFT_METHOD_BEG();

    do {
        /************************************
        ** Since this MFT is a decoder, it
        ** must not allow this function to be
        ** called until it is unlocked. If
        ** your MFT is an encoder, this function
        ** CAN be called before the MFT is
        ** unlocked
        ************************************/
        if (IsLocked() != FALSE) {
            hr = MF_E_TRANSFORM_ASYNC_LOCKED;
            break;
        }

        if (pType == NULL) {
            hr = E_POINTER;
            break;
        }

        if (dwFlags != 0) {
            MyFailed(E_UNEXPECTED);
        }

        /*****************************************
        ** Todo: If your MFT supports more than one
        ** stream, make sure you modify
        ** MFT_MAX_STREAMS and adjust this function
        ** accordingly
        *****************************************/
        if (dwOutputStreamID >= MFT_MAX_STREAMS) {
            hr = MF_E_INVALIDSTREAMNUMBER;
            break;
        }

        hr = CheckOutputType(pType);
        if (FAILED(hr)) {
            break;
        }

        /*******************************************
        ** Store a copy of the media type, not the
        ** one passed in by the caller. This way the
        ** caller is unable to modify the internal
        ** media type
        *******************************************/

        hr = MFCreateMediaType(&pMT);
        if (FAILED(hr)) {
            break;
        }

        hr = DuplicateAttributes(pMT, pType);
        if (FAILED(hr)) {
            break;
        }

        pMT->SetUINT32(MF_SA_MINIMUM_OUTPUT_SAMPLE_COUNT, 3);

        {
            CAutoLock lock(&m_csLock);

            SAFERELEASE(m_pOutputMT);

            m_pOutputMT = pMT;
            m_pOutputMT->AddRef();
        }

        IsMFTReady();
    } while (false);

    SAFERELEASE(pMT);
    CMFT_METHOD_END("");

    return hr;
}
