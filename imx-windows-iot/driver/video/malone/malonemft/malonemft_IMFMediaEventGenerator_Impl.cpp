#include "malonemft.h"
#include <Mfapi.h>

HRESULT CMaloneMft::BeginGetEvent(
    IMFAsyncCallback   *pCallback,
    IUnknown           *punkState)
{
    /***************************************
    ** Since this in an internal function
    ** we know m_pEventQueue can never be
    ** NULL due to InitializeTransform()
    ***************************************/

    return m_pEventQueue->BeginGetEvent(pCallback, punkState);
}

HRESULT CMaloneMft::EndGetEvent(
    IMFAsyncResult *pResult,
    IMFMediaEvent **ppEvent)
{
    /***************************************
    ** Since this in an internal function
    ** we know m_pEventQueue can never be
    ** NULL due to InitializeTransform()
    ***************************************/

    return m_pEventQueue->EndGetEvent(pResult, ppEvent);
}

HRESULT CMaloneMft::GetEvent(
    DWORD           dwFlags,
    IMFMediaEvent **ppEvent)
{
    /***************************************
    ** Since this in an internal function
    ** we know m_pEventQueue can never be
    ** NULL due to InitializeTransform()
    ***************************************/

    return m_pEventQueue->GetEvent(dwFlags, ppEvent);
}

HRESULT CMaloneMft::QueueEvent(
    MediaEventType      met,
    REFGUID             guidExtendedType,
    HRESULT             hrStatus,
    const PROPVARIANT  *pvValue)
{
    /***************************************
    ** Since this in an internal function
    ** we know m_pEventQueue can never be
    ** NULL due to InitializeTransform()
    ***************************************/

    return m_pEventQueue->QueueEventParamVar(met, guidExtendedType, hrStatus, pvValue);
}
