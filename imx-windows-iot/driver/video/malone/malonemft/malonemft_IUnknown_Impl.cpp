#include "malonemft.h"
#include "imalonemft.h"

ULONG CMaloneMft::AddRef(void)
{
    return InterlockedIncrement(&m_ulRef);
}

HRESULT CMaloneMft::QueryInterface(
    REFIID riid,
    void **ppvObject)
{
    HRESULT hr = S_OK;

    do {
        if (ppvObject == NULL) {
            hr = E_POINTER;
            break;
        }

        /****************************************************
        ** Todo: add all supported interfaces by your MFT
        ****************************************************/
        if (riid == IID_IMFTransform) {
            *ppvObject = (IMFTransform *)this;
        } else if (riid == IID_IMFAttributes) {
            *ppvObject = (IMFAttributes *)this;
        } else if (riid == IID_IMFShutdown) {
            *ppvObject = (IMFShutdown *)this;
        } else if (riid == IID_IMFMediaEventGenerator) {
            *ppvObject = (IMFMediaEventGenerator *)this;
        } else if (riid == IID_IMFAsyncCallback) {
            *ppvObject = (IMFAsyncCallback *)this;
        } else if (riid == IID_IMALONEMFT) {
            *ppvObject = (IMaloneMft *)this;
        } else if (riid == IID_IUnknown) {
            *ppvObject = this;
        } else {
            *ppvObject = NULL;
            hr = E_NOINTERFACE;
            break;
        }

        AddRef();
    } while (false);

    return hr;
}

ULONG CMaloneMft::Release(void)
{
    ULONG   ulRef = 0;

    if (m_ulRef > 0) {
        ulRef = InterlockedDecrement(&m_ulRef);
    }

    if (ulRef == 0) {
        delete this;
    }

    return ulRef;
}
