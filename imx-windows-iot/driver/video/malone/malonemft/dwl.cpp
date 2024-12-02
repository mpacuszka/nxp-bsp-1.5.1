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
#include <windows.h>
#include <stdio.h>
#include <string>
#include <cfgmgr32.h>
#include <stdint.h>
#include <initguid.h>
#include <winioctl.h>
#include <deviceaccess.h>

#include "Public.h"
#include "vpu_dec\vdec_cmn.h"
#include "dwl.h"

#define _DWL_DEBUG

#ifdef WIN32
#ifdef _DWL_DEBUG
char msgbuf[256];
#define DWL_DEBUG(...) \
        sprintf_s(msgbuf, __VA_ARGS__); \
        OutputDebugStringA(msgbuf);
#endif
#else
#ifdef _DWL_DEBUG
#define DWL_DEBUG(fmt, args...) \
  printf(__FILE__ ":%d:%s() " fmt, __LINE__, __func__, ##args)
#else
#define DWL_DEBUG(fmt, args, ...) \
  do {                          \
  } while (0); /* not debugging: nothing */
#endif
#endif

IDeviceIoControl *decoder_open()
{
    ULONG length;
    CONFIGRET cr;
    HRESULT hr = S_OK;
    WCHAR *buf = NULL;
    HANDLE handle = INVALID_HANDLE_VALUE;
    ICreateDeviceAccessAsync *accessAsync = NULL;
    IDeviceIoControl *deviceAccess = NULL;

    do {
        cr = CM_Get_Device_Interface_List_SizeW(
                 &length,
                 (LPGUID)&GUID_DEVINTERFACE_malonekm,
                 NULL,        // pDeviceID
                 CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

        if (cr != CR_SUCCESS || length < 2) {
            break;
        }

        buf = (WCHAR *)malloc(length * sizeof(WCHAR));
        if (buf == NULL) {
            break;
        }

        cr = CM_Get_Device_Interface_ListW(
                 (LPGUID)&GUID_DEVINTERFACE_malonekm,
                 NULL,        // pDeviceID
                 buf,
                 length,
                 CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

        if (cr != CR_SUCCESS) {
            break;
        }

        hr = CreateDeviceAccessInstance(buf, GENERIC_READ | GENERIC_WRITE, &accessAsync);
        if (FAILED(hr)) {
            DWL_DEBUG("CreateDeviceAccessInstance returned 0x%x\n", hr);
            break;
        }

        hr = accessAsync->Wait(INFINITE);
        if (FAILED(hr)) {
            DWL_DEBUG("Wait returned 0x%x\n", hr);
            break;
        }

        hr = accessAsync->GetResult(IID_IDeviceIoControl, (void **)&deviceAccess);
        if (FAILED(hr)) {
            DWL_DEBUG("GetResult returned 0x%x\n", hr);
            break;
        }
    } while (FALSE);

    if (buf != NULL) {
        free(buf);
        buf = NULL;
    }

    if (accessAsync != NULL) {
        accessAsync->Release();
        accessAsync = NULL;
    }

    if (FAILED(hr) || (cr != CR_SUCCESS)) {
        if (deviceAccess != NULL) {
            deviceAccess->Release();
            deviceAccess = NULL;
        }
    } else {
        deviceAccess->AddRef();
    }

    return deviceAccess;
}

static HRESULT ioctl(IDeviceIoControl *handle, DWORD operation, UCHAR *ioctlInBuf, DWORD inSize, UCHAR *ioctlOutBuf,
                     DWORD outSize)
{
    DWORD bytes = 0;
    HRESULT hr = S_OK;

    hr = handle->DeviceIoControlSync(operation, (UCHAR *)ioctlInBuf, inSize, (UCHAR *)ioctlOutBuf, outSize, &bytes);

    if (FAILED(hr)) {
        DWL_DEBUG("ioctl(0x%x) returned 0x%x\n", operation, hr);
        return -1;
    }

    if (bytes != outSize) {
        DWL_DEBUG("ioctl(0x%x) returned success but incorrect outsize %d\n", operation, bytes);
        return -1;
    }

    return hr;
}

/**
 * Closes file descriptor to access context
 *
 * @param id File descriptor.
 *
 * @return File descriptor.
 */
void decoder_close(IDeviceIoControl *handle)
{
    handle->Release();
}

/**
 * Allocates context and initilaize instance specific setting
 *
 * param in id : File descriptor
 * param in dec: Refer vdec_init_t
 * param out mem : Refer vdec_mem_desc_t
 *
 * return 0 for success
 */
HRESULT decoder_init(IDeviceIoControl *handle, vdec_init_t *dec, vdec_mem_desc_t *mem)
{
    DWORD inSize = sizeof(vdec_init_t);
    DWORD outSize = sizeof(vdec_mem_desc_t);

    return ioctl(handle, VPU_IOC_INIT, (UCHAR *)dec, inSize, (UCHAR *)mem, outSize);
}



/**
 * Destroys context
 *
 * param in id : File descriptor
 *
 * return 0 for success
 */
HRESULT decoder_deinit(IDeviceIoControl *handle)
{
    return ioctl(handle, VPU_IOC_DEINIT, 0, 0, 0, 0);
}

/**
 * Updates  VPU bitstream buffer wptr and gets decoded buffer.
 *
 * param in id : file descriptor
 * param in dec : Refer vdec_decode_t for more details
 *
 * return 0 for success
 */
HRESULT decoder_decode(IDeviceIoControl *handle, vdec_decode_t *dec)
{
    DWORD inSize = sizeof(vdec_decode_t);
    DWORD outSize = sizeof(vdec_decode_t);
    return ioctl(handle, VPU_IOC_DECODE, (UCHAR *)dec, inSize, (UCHAR *)dec, outSize);
}

/**
 * Provides status of context.
 *
 * param out status : Refer vdec_status_t
 *
 * return 0 for success
 */
HRESULT decoder_status(IDeviceIoControl *handle, vdec_status_t *status)
{
    DWORD outSize = sizeof(vdec_status_t);
    return ioctl(handle, VPU_IOC_STATUS, NULL, 0, (UCHAR *)status, outSize);
}

/**
 * Release displayed buffer back
 * param in id : file descriptor
 * param in index : output buffer index
 *
 * return 0 for success
 */
HRESULT decoder_clear_output(IDeviceIoControl *handle, int *index)
{
    DWORD inSize = sizeof(int);
    return ioctl(handle, VPU_IOC_CLEAR, (UCHAR *)index, inSize, NULL, 0);
}

/**
 * Gets decoded output
 *
 * param out status : Refer decoder_get_output
 *
 * return 0 for success
 */
HRESULT decoder_get_output(IDeviceIoControl *handle, fbo_t *fbo)
{
    DWORD inSize = sizeof(fbo_t);
    DWORD outSize = sizeof(fbo_t);
    return ioctl(handle, VPU_IOC_GETOUTPUT, (UCHAR *)fbo, inSize, (UCHAR *)fbo, outSize);

}

/**
 * Flush
 *
 * param out status : Refer vdec_status_t
 *
 * return 0 for success
 */
HRESULT decoder_flush(IDeviceIoControl *handle, vdec_flush_t *flush)
{
    DWORD inSize = sizeof(vdec_flush_t);
    return ioctl(handle, VPU_IOC_FLUSH, (UCHAR *)flush, inSize, NULL, 0);
}