#pragma once

#include <Mfidl.h>
#include <deviceaccess.h>
#include <vpu_dec\vdec_cmn.h>

// {BC36CB81-C1FA-4B92-B12D-FCBB8DB3FDF6}
DEFINE_GUID(IID_IMALONEMFT,
            0xbc36cb81, 0xc1fa, 0x4b92, 0xb1, 0x2d, 0xfc, 0xbb, 0x8d, 0xb3, 0xfd, 0xf6);

// The same as hantro!
// {AB1AD049-3E60-4D93-852D-F5CBF6736EAD}
DEFINE_GUID(IID_IVpuBufferCollection,
            0xab1ad049, 0x3e60, 0x4d93, 0x85, 0x2d, 0xf5, 0xcb, 0xf6, 0x73, 0x6e, 0xad);

class IMaloneMft:
    public IUnknown
{
public:
    virtual HRESULT __stdcall   DecodeInputFrame(
        IMFSample  *pInputSample
    )  = 0;

    virtual HRESULT __stdcall ScheduleFrameDecode(void) = 0;
};

class IVpuBufferCollection :
    public IUnknown
{
public:
    virtual HRESULT __stdcall CreateBufferInstance(
        _In_ IDeviceIoControl *m_vpuHandle,
        _In_ UINT32 maxLength,
        _In_ fbo_t frameBuffer,
        _Out_ IMFMediaBuffer **ppBuffer) = 0;

    virtual HRESULT __stdcall MarkBufferFree(
        _In_ IMFMediaBuffer *buffer) = 0;

    virtual HRESULT __stdcall ScheduleFrameDecode(void) = 0;

    virtual HRESULT __stdcall FrameDecodeError(void) = 0;
};
