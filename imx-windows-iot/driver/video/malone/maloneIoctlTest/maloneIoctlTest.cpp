/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "pch.h"

#include <stdio.h>
#include <string>
#include <cfgmgr32.h>

#include <initguid.h>



#include "Public.h"
#include <stdint.h>
#include "vpu_dec\vdec_cmn.h"


using namespace Concurrency;
using namespace Microsoft::WRL;
using namespace Microsoft::WRL::Wrappers;




// attempt to read a memory address
// returns true on success
bool testMemoryAccess(void *ptr, long *value)
{
    bool success = true;
    
    __try {
        *value = *(long *)ptr;
    }
    __except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
        EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
    {
        success = false;
    }
    return success;
}

std::wstring GetInterfacePath(const GUID& InterfaceGuid)
{
    ULONG length;
    CONFIGRET cr = CM_Get_Device_Interface_List_SizeW(
        &length,
        const_cast<GUID*>(&InterfaceGuid),
        nullptr,        // pDeviceID
        CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if (cr != CR_SUCCESS) {
        throw wexception::make(
            HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
            L"Failed to get size of device interface list. (cr = 0x%x)",
            cr);
    }

    if (length < 2) {
        throw wexception::make(
            HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
            L"The IMXVPUKM device was not found on this system. (cr = 0x%x)",
            cr);
    }

    std::unique_ptr<WCHAR[]> buf(new WCHAR[length]);
    cr = CM_Get_Device_Interface_ListW(
        const_cast<GUID*>(&InterfaceGuid),
        nullptr,        // pDeviceID
        buf.get(),
        length,
        CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if (cr != CR_SUCCESS) {
        throw wexception::make(
            HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
            L"Failed to get device interface list. (cr = 0x%x)",
            cr);
    }

    // Return the first string in the multistring
    return std::wstring(buf.get());
}


FileHandle OpenVpuHandle()
{
    auto interfacePath = GetInterfacePath(GUID_DEVINTERFACE_malonekm);

    printf("Opening device %S\n", interfacePath.c_str());

    FileHandle fileHandle(CreateFile(
        interfacePath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,          // dwShareMode
        nullptr,    // lpSecurityAttributes
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr));  // hTemplateFile
    
    if (!fileHandle.IsValid()) {
        if (GetLastError() == ERROR_ACCESS_DENIED) {
            // Try opening read-only
            fileHandle.Attach(CreateFile(
                interfacePath.c_str(),
                GENERIC_READ,
                0,          // dwShareMode
                nullptr,    // lpSecurityAttributes
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr));  // hTemplateFile

            if (fileHandle.IsValid()) {
                return fileHandle;
            }
        }

        throw wexception::make(
            HRESULT_FROM_WIN32(GetLastError()),
            L"Failed to open a handle to the VPU device. "
            L"(hr = 0x%x, interfacePath = %s)",
            HRESULT_FROM_WIN32(GetLastError()),
            interfacePath.c_str());
    }

    return fileHandle;
}

#define LOW_LATENCY_EN 1
void test()
{
    auto handle = OpenVpuHandle();
    void* vpuMem = NULL;
    DWORD bytes;

    printf("Vpu Handle Opened\n");

    vdec_init_t init = { 0 };
    init.fourcc = (uint32_t)AOIFOURCC("H264");
    init.dis_reorder = LOW_LATENCY_EN;
    init.frame_mode = 1;
    init.force_fbufs_num = 0;

    vdec_mem_desc_t sb_mem = { 0 };

    if (!DeviceIoControl(handle.Get(), VPU_IOC_INIT, &init, sizeof(init), &sb_mem, sizeof(sb_mem), &bytes,  nullptr)
        || (bytes != sizeof(sb_mem))) {
        throw wexception::make(
            HRESULT_FROM_WIN32(GetLastError()),
            L"VPU_IOC_INIT type %d failed. "
            L"(hr = 0x%x, bytes = %d)",
            sb_mem.flags,
            HRESULT_FROM_WIN32(GetLastError()),
            bytes);
    }
    else
    {
        long value = 0;

        if (sb_mem.physAddress > ULONG_MAX)
        {
            throw wexception::make(
                E_FAIL,
                L"VPU_IOC_INIT type %d succeeded but sb memory outside 32 bit address range. "
                L"VirtAddres = %p, physAddress = %llx", sb_mem.flags, sb_mem.virtAddress, sb_mem.physAddress);
        }

        if (testMemoryAccess(sb_mem.virtAddress, &value))
        {
            printf("VPU_IOC_INIT(%d, %d) VirtAddres = %p, physAddress = %llx, value=%d\n", sb_mem.size, sb_mem.flags, sb_mem.virtAddress, sb_mem.physAddress, value);
        }
        else
        {
            throw wexception::make(
                E_FAIL,
                L"VPU_IOC_INIT type %d succeeded but sb memory access failed. "
                L"VirtAddres = %p, physAddress = %llx", sb_mem.flags, sb_mem.virtAddress, sb_mem.physAddress);
        }
    }

    handle.Close();
    printf("Vpu Handle Closed\n");

    return;
}

int main()
{
    printf("imx8q-IoctlTest\n");

    try
    {
        // basic tests
        test();

//        TestReserveRelease();

//        TestReserveFileClose();
    }
    catch (wexception &err)
    {
        printf("Test execution failed HR=%08x\n"
            "%S\n",
            err.HResult(),
            err.wwhat());
        return -1;
    }

    return 0;
}
