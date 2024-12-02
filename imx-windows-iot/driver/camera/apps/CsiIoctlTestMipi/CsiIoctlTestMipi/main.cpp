/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

   // #include "pch.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include <Windows.h>
#include <cfgmgr32.h>

#include <Windows.System.Threading.h>
#include <ppl.h>
#include <wrl.h>
#include "util.h"

#include <initguid.h>

#include <winioctl.h>
#include "imxmipi\Public.h"

using namespace Concurrency;
using namespace Microsoft::WRL;
using namespace Microsoft::WRL::Wrappers;

static unsigned tId = 0; // Test counter


UINT8 csiLanes = 1;
PCWSTR appName;
bool cfg_only_get_frame = false;
uint8_t GtHsSettle_EscClk = 0xE;
wchar_t driver_id_str = '0'; // User can specify instance


#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const uint32_t csi2rxHsSettle[][3] = {
	{
		kVIDEO_Resolution720P,
		30,
		0x12,
	},
	{
		kVIDEO_Resolution720P,
		15,
		0x17,
	},
	{
		kVIDEO_ResolutionVGA,
		30,
		0x1F,
	},
	{
		kVIDEO_ResolutionVGA,
		15,
		0x24,
	},
	{
		kVIDEO_ResolutionQVGA,
		30,
		0x1F,
	},
	{
		kVIDEO_ResolutionQVGA,
		15,
		0x24,
	},
};

uint32_t GetSettle(uint32_t resolution, uint32_t framerate)
{
	uint32_t tHsSettle_EscClk = 0x12;
	for (uint8_t i = 0; i < ARRAY_SIZE(csi2rxHsSettle); i++) {
		if ((resolution == csi2rxHsSettle[i][0]) && (csi2rxHsSettle[i][1] == framerate)) {
			tHsSettle_EscClk = csi2rxHsSettle[i][2];
			break;
		}
	}
	return tHsSettle_EscClk;
}

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

FileHandle OpenCsiHandle(DWORD additionalFlags = 0, const wchar_t id = 0)
{
	auto interfacePath = GetInterfacePath(GUID_DEVINTERFACE_IMXMIPI);
	if (id != 0) {
		interfacePath.back() = id;
	}
	printf("  Opening device %S\n", interfacePath.c_str());

	FileHandle fileHandle(CreateFile(// CreateFile2
		interfacePath.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,          // dwShareMode
		nullptr,    // lpSecurityAttributes
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | additionalFlags,
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
				FILE_ATTRIBUTE_NORMAL | additionalFlags,
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
#if 0
void init_cam()
{
	printf("Tinit_cam\n");
	const size_t imageSizeB = 1280*720*2;
	PVOID image = malloc(imageSizeB);
	if (image == NULL) {
		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"malloc(%d) failed. "
			L"(hr = 0x%x)",
			imageSizeB,
			HRESULT_FROM_WIN32(GetLastError()));
	}
	else {
		DWORD bytes;

		auto handle = OpenCsiHandle(0, driver_id_str);
		{
			// camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, csiLanes, 0xE /* 0x12 */ };
			// camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, csiLanes, 0x10 };
			camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, csiLanes, GtHsSettle_EscClk };
			// camera_config_t cfg{ resolution, cameraPixelFormat , resultPixelFormat, framePerSec, mipiChannel, csiLanes, tHsSettle_EscClk, InterfaceMIPI,  };

			// cfg.tHsSettle_EscClk = (UINT8)GetSettle(cfg.resolution, cfg.framePerSec);
			ULONG output;
			if (!DeviceIoControl(handle.Get(), IOCTL_CSI_DRIVER_INIT,
				(LPVOID)&cfg, sizeof(cfg),
				&output, sizeof(output),
				&bytes, nullptr
			)) {
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"IOCTL_CSI_DRIVER_INIT failed. "
					L"(hr = 0x%x, bytes = %d of %d)",
					HRESULT_FROM_WIN32(GetLastError()),
					bytes, sizeof(output));
			}
			else
			{
				printf("    IOCTL_CSI_DRIVER_INIT output = %d\n", output);
			}

			printf("Test %d: IOCTL_CSI_DRIVER_GET_FRAME with invalid address\n", ++tId);
		}

		printf("    IOCTL_CSI_DRIVER_GET_FRAME Test Default image: 0x%x\n", *(PUINT32)image);
		{
			LARGE_INTEGER cnt, StartingTime, EndingTime, ElapsedMicroseconds;
			LARGE_INTEGER Frequency;
			FrameInfo_t fInfo{ 0 };

			fInfo.Virtual = (PUCHAR)1;
			QueryPerformanceFrequency(&Frequency);
			QueryPerformanceCounter(&StartingTime);
			QueryPerformanceCounter(&EndingTime);
			cnt.QuadPart = 0;
			int seconds = 0;
			while (1)
			{

				if (!DeviceIoControl(handle.Get(), IOCTL_CSI_DRIVER_GET_FRAME,
					&fInfo, sizeof(fInfo),
					image, imageSizeB,
					&bytes, nullptr
				)) { // || (bytes != sizeof(output))) {
					throw wexception::make(
						HRESULT_FROM_WIN32(GetLastError()),
						L"IOCTL_CSI_DRIVER_GET_FRAME failed. "
						L"(hr = 0x%x, bytes = %d of %d)",
						HRESULT_FROM_WIN32(GetLastError()),
						bytes, sizeof(imageSizeB));
				}
				else
				{
					StartingTime = EndingTime;
					QueryPerformanceCounter(&EndingTime);
					ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
					ElapsedMicroseconds.QuadPart *= 1000;
					ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
					cnt.QuadPart += ElapsedMicroseconds.QuadPart;
					if (cnt.QuadPart > 1000) {
						cnt.QuadPart = 0;
						++seconds;
					}
					printf("   %d s: IOCTL_CSI_DRIVER_GET_FRAME (%lld ms) passed 0x%x\n", seconds, ElapsedMicroseconds.QuadPart, *(PUINT32)image);

				}
			}
		}
		while (1);
	}
}
// 1843200
#endif
void test()
{
	printf("Test %d: simple ioctl tests\n", ++tId);
	auto handle = OpenCsiHandle();
	// int val;
	// ULONG input;
	ULONG output = 0;
	// void *vpuMem = NULL;
	DWORD bytes;

	camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, csiLanes, GtHsSettle_EscClk };
	// camera_config_t cfg{ resolution, cameraPixelFormat , resultPixelFormat, framePerSec, mipiChannel, csiLanes, tHsSettle_EscClk, InterfaceMIPI,  };

	// cfg.tHsSettle_EscClk = (UINT8)GetSettle(cfg.resolution, cfg.framePerSec);
	printf("Test %d: IOCTL_CSI_DRIVER_TEST_SUCCESS_REPLY\n", ++tId);
	// VPU_MC_CORES test call
	if (!DeviceIoControl(
		handle.Get(),
		IOCTL_CONFIGURE,
		(LPVOID)&cfg, sizeof(cfg),
		&output,
		sizeof(output),
		&bytes,
		nullptr
	)) { // || (bytes != sizeof(output))) {
		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"IOCTL_CSI_DRIVER_TEST_SUCCESS_REPLY failed. "
			L"(hr = 0x%x, bytes = %d of %d)",
			HRESULT_FROM_WIN32(GetLastError()),
			bytes, sizeof(output));
	}
	else
	{
		printf("    IOCTL_CSI_DRIVER_TEST_SUCCESS_REPLY output = %d\n", output);
	}
#if 0
	// IOCTL_CSI_DRIVER_GET_FRAME test call
	printf("Test %d: IOCTL_CSI_DRIVER_GET_FRAME at Stop\n", ++tId);
	if (DeviceIoControl(
		handle.Get(),
		IOCTL_STOP,
		nullptr,
		0,
		&output,
		sizeof(output),
		&bytes,
		nullptr
	) || (GetLastError() != ERROR_NOT_READY)) { // || (bytes != sizeof(output))) {
		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"IOCTL_CSI_DRIVER_GET_FRAME did not report NOT_READY. "
			L"(hr = 0x%x, bytes = %d of %d)",
			/*HRESULT_FROM_WIN32(*/GetLastError(),//),
			bytes, sizeof(output));
	}
	else
	{
		printf("    IOCTL_CSI_DRIVER_GET_FRAME output = %d\n", output);
	}
#endif
}
#if 0
void TestMultiClient()
{
	printf("Test %d: OpenCsiHandle double 1 and single 2\n", ++tId);
	auto handle1 = OpenCsiHandle();
	auto handle2 = OpenCsiHandle(0, '2');
	try {
		auto handle3 = OpenCsiHandle();
		throw wexception::make(
			E_FAIL,
			L"    Successfully opened channel 1 twice.");
	}
	catch (wexception &err)
	{
		if (err.HResult() != E_ACCESSDENIED) {
			throw;
		}
		else {
			printf("	E_ACCESSDENIED - Couldn't open device twice.\n");
		}
	}
}

void TestOverlappedIo()
{
	printf("Test %d: OverlappedIo CancelIo\n", ++tId);
	auto handle = OpenCsiHandle(FILE_FLAG_OVERLAPPED);
		// InitIrpParam param;
		// param.req = InitIrpParam::STOP_TEST;
		// param.timeout = 1000;

		// LPOVERLAPPED overlappedContext = {0};
		Event overlappedEvent(CreateEventW(NULL, TRUE, FALSE, NULL));
		if (!overlappedEvent.IsValid()) {
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"Failed to create event for overlapped IO. " L"(hr = 0x%x)", HRESULT_FROM_WIN32(GetLastError()));
		}

	{ // IOCTL_CSI_DRIVER_TEST_NO_REPLY - STOP_TEST
			auto overlappedContext = OVERLAPPED();
			overlappedContext.hEvent = overlappedEvent.Get();
			printf("    DeviceIoControl(IOCTL_CSI_DRIVER_TEST_NO_REPLY)\n");
		if (!DeviceIoControl(
			handle.Get(),
			IOCTL_CSI_DRIVER_TEST_NO_REPLY,
			NULL, //(LPVOID)&param,
			0, //sizeof(param),
			NULL,
			0,
			NULL,
			&overlappedContext) && (GetLastError() != ERROR_IO_PENDING)) {

			throw wexception::make(
				HRESULT_FROM_WIN32(GetLastError()),
				L"Stop test(IOCTL_CSI_DRIVER_TEST_NO_REPLY). "
				L"(hr = 0x%x)",
				HRESULT_FROM_WIN32(GetLastError()));
		}
		else {
			printf("		CancelIoEx()\n");
			BOOL r = CancelIoEx(handle.Get(), &overlappedContext);
			DWORD Status = GetLastError();
			printf("		CancelIoEx() %s with %x\n", r?"succeeded":"failed", Status);

			// if ((Status != ERROR_OPERATION_ABORTED) && (Status != ERROR_SUCCESS)) {
			DWORD NumberOfBytesTransferred;
			printf("		GetOverlappedResult()\n");
			r = GetOverlappedResult(handle.Get(), &overlappedContext, &NumberOfBytesTransferred, NULL);
			printf("		GetOverlappedResult() %s with %x\n", r ? "succeeded" : "failed", GetLastError());
			if (!r && (GetLastError() != ERROR_OPERATION_ABORTED)) { // Success or non-aborted Status means cancel test has failed.
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"Stop test(IOCTL_CSI_DRIVER_TEST_NO_REPLY). "
					L"(hr = 0x%x)",
					HRESULT_FROM_WIN32(GetLastError()));
			}
		}
		printf("		IOCTL_CSI_DRIVER_INIT(STOP_TEST) passed.\n");
	}

	{ // IOCTL_CSI_DRIVER_INIT - STOP_TEST
		auto overlappedContext = OVERLAPPED();
		overlappedContext.hEvent = overlappedEvent.Get();
		printf("    DeviceIoControl(IOCTL_CSI_DRIVER_INIT)\n");
		if (!DeviceIoControl(handle.Get(),IOCTL_CSI_DRIVER_INIT,
			NULL, //(LPVOID)&param,
			0, //sizeof(param),
			NULL,
			0,
			NULL,
			&overlappedContext) && (GetLastError() != ERROR_IO_PENDING)) {
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"Stop test(IOCTL_CSI_DRIVER_INIT). " L"(hr = 0x%x)", HRESULT_FROM_WIN32(GetLastError()));
		}
		else {
			printf("		CancelIoEx()\n");
			BOOL r = CancelIoEx(handle.Get(), &overlappedContext);
			DWORD Status = GetLastError();
			printf("		CancelIoEx() %s with %x\n", r ? "succeeded" : "failed", Status);
			// if ((Status != ERROR_OPERATION_ABORTED) && (Status != ERROR_SUCCESS)) {
			DWORD NumberOfBytesTransferred; 
			printf("		GetOverlappedResult()\n");
			r = GetOverlappedResult(handle.Get(), &overlappedContext, &NumberOfBytesTransferred, NULL);
			printf("		GetOverlappedResult() %s with %x\n", r ? "succeeded" : "failed", GetLastError());
			if (!r && (Status != ERROR_OPERATION_ABORTED) || (Status != ERROR_NOT_FOUND) && r) { // Success or non-aborted Status means cancel test has failed.
				throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"Stop test(IOCTL_CSI_DRIVER_INIT). " L"(hr = 0x%x)", HRESULT_FROM_WIN32(GetLastError()));
			}
		}
		printf("		IOCTL_CSI_DRIVER_INIT(STOP_TEST) not fully implemented.\n");
	}

	{ // IOCTL_CSI_DRIVER_GET_FRAME - STOP_TEST
		auto overlappedContext = OVERLAPPED();
		overlappedContext.hEvent = overlappedEvent.Get();
		printf("    DeviceIoControl(IOCTL_CSI_DRIVER_GET_FRAME)\n");
		if (!DeviceIoControl(handle.Get(), IOCTL_CSI_DRIVER_GET_FRAME,
			NULL, //(LPVOID)&param,
			0, //sizeof(param),
			NULL,
			0,
			NULL,
			&overlappedContext) && (GetLastError() != ERROR_IO_PENDING)) {

			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"Stop test(IOCTL_CSI_DRIVER_GET_FRAME). " L"(hr = 0x%x)", HRESULT_FROM_WIN32(GetLastError()));
		}
		else {
			printf("		CancelIoEx()\n");
			BOOL r = CancelIoEx(handle.Get(), &overlappedContext);
			DWORD Status = GetLastError();
			printf("		CancelIoEx() %s with %x\n", r ? "succeeded" : "failed", Status); // 3e5 == ERROR_IO_PENDING
			// if ((Status != ERROR_OPERATION_ABORTED) && (Status != ERROR_SUCCESS)) {
			DWORD NumberOfBytesTransferred;
			printf("		GetOverlappedResult()\n");
			r = GetOverlappedResult(handle.Get(), &overlappedContext, &NumberOfBytesTransferred, NULL);
			printf("		GetOverlappedResult() %s with %x\n", r ? "succeeded" : "failed", GetLastError());
			if (!r && (GetLastError() != ERROR_OPERATION_ABORTED)) { // Success or non-aborted Status means cancel test has failed.
				throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"Stop test(IOCTL_CSI_DRIVER_GET_FRAME). " L"(hr = 0x%x)", HRESULT_FROM_WIN32(GetLastError()));
			}
		}
		printf("		IOCTL_CSI_DRIVER_GET_FRAME(STOP_TEST) passed.\n");
	}
}
#endif
#if 0
void TestReserveRelease()
{
	auto handle1 = OpenCsiHandle();
	auto handle2 = OpenCsiHandle();

	auto vpu1 = handle1.Get();
	auto vpu2 = handle2.Get();
	event doneEvent;

	ULONG input;
	ULONG output;
	ULONG bytes;
	task_group tasks;
	BOOL failed = FALSE;

	printf("Testing Reserve/Release\n");

	printf("Reserving H264 decoder\n");

	// Reserve the H264 decoder
	input = DWL_CLIENT_TYPE_H264_DEC;
	if (!DeviceIoControl(
		vpu1,
		IOCTL_VPU_DEC_RESERVE,
		&input,
		sizeof(input),
		&output,
		sizeof(output),
		&bytes,
		nullptr) || (bytes != sizeof(output))) {

		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"IOCTL_VPU_DEC_RESERVE(%d) failed. "
			L"(hr = 0x%x, bytes = %d)",
			input,
			HRESULT_FROM_WIN32(GetLastError()),
			bytes);
	}

	printf("Running task to also reserve H264 decoder\n");

	// schedule a task to reserve the VPU
	tasks.run([&] {
		try
		{
			auto vpu2 = handle2.Get();

			ULONG in, out, b;
			in = DWL_CLIENT_TYPE_H264_DEC;

			printf("subtask: Reserving H264 decoder\n");

			if (!DeviceIoControl(
				vpu2,
				IOCTL_VPU_DEC_RESERVE,
				&in,
				sizeof(in),
				&out,
				sizeof(out),
				&b,
				nullptr) || (b != sizeof(out))) {

				failed = TRUE;
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"IOCTL_VPU_DEC_RESERVE(%d) failed. "
					L"(hr = 0x%x, bytes = %d)",
					input,
					HRESULT_FROM_WIN32(GetLastError()),
					bytes);
			}

			printf("subtask: H264 decoder reserved\n");

			doneEvent.set();

			printf("subtask: releasing H264 decoder\n");

			if (!DeviceIoControl(
				vpu2,
				IOCTL_VPU_DEC_RELEASE,
				&out,
				sizeof(out),
				nullptr,
				0,
				&b,
				nullptr) || (b != 0)) {

				failed = TRUE;
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"IOCTL_VPU_DEC_RELEASE(%d) failed. "
					L"(hr = 0x%x, bytes = %d)",
					input,
					HRESULT_FROM_WIN32(GetLastError()),
					bytes);
			}

			printf("subtask: H264 decoder released\n");
		}
		catch (wexception &err)
		{
			printf("subtask: execution failed HR=%08x\n"
				"%S\n",
				err.HResult(),
				err.wwhat());
		}
	});

	// wait 20 ms
	if (doneEvent.wait(20) == 0)
	{
		failed = TRUE;

		throw wexception::make(
			E_FAIL,
			L"done event is signaled before VPU released");
	}
	else
	{
		printf("Subtask is correctly blocked on IOCT_VPU_DEC_RESERVE\n");
	}

	printf("Releasing core %d\n", output);

	// release the VPU
	if (!DeviceIoControl(
		vpu1,
		IOCTL_VPU_DEC_RELEASE,
		&output,
		sizeof(output),
		nullptr,
		0,
		&bytes,
		nullptr) || (bytes != 0)) {

		failed = TRUE;
		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"IOCTL_VPU_DEC_RELEASE(%d) failed. "
			L"(hr = 0x%x, bytes = %d)",
			input,
			HRESULT_FROM_WIN32(GetLastError()),
			bytes);
	}

	// wait upto 20 ms, but should return immediately
	if (doneEvent.wait(20) != 0)
	{
		failed = TRUE;

		throw wexception::make(
			E_FAIL,
			L"done event was not signaled after VPU released");
	}
	else
	{
		printf("Subtask correctly signaled done event\n");
	}

	tasks.wait();
	printf("Finished TestReserveRelease\n");
}
#endif

#if 0
void TestReserveFileClose()
{
	auto handle1 = OpenCsiHandle();
	auto handle2 = OpenCsiHandle();

	auto vpu1 = handle1.Get();
	auto vpu2 = handle2.Get();
	event doneEvent;

	ULONG input;
	ULONG output;
	ULONG bytes;
	task_group tasks;
	BOOL failed = FALSE;

	printf("Testing Reserve/FileClose\n");

	printf("Reserving H265 decoder\n");

	// Reserve the H264 decoder
	input = DWL_CLIENT_TYPE_HEVC_DEC;
	if (!DeviceIoControl(
		vpu1,
		IOCTL_VPU_DEC_RESERVE,
		&input,
		sizeof(input),
		&output,
		sizeof(output),
		&bytes,
		nullptr) || (bytes != sizeof(output))) {

		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"IOCTL_VPU_DEC_RESERVE(%d) failed. "
			L"(hr = 0x%x, bytes = %d)",
			input,
			HRESULT_FROM_WIN32(GetLastError()),
			bytes);
	}

	printf("Running task to also reserve H264 decoder\n");

	// schedule a task to reserve the VPU
	tasks.run([&] {
		try
		{
			auto vpu2 = handle2.Get();

			ULONG in, out, b;
			in = DWL_CLIENT_TYPE_HEVC_DEC;

			printf("subtask: Reserving H265 decoder\n");

			if (!DeviceIoControl(
				vpu2,
				IOCTL_VPU_DEC_RESERVE,
				&in,
				sizeof(in),
				&out,
				sizeof(out),
				&b,
				nullptr) || (b != sizeof(out))) {

				failed = TRUE;
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"IOCTL_VPU_DEC_RESERVE(%d) failed. "
					L"(hr = 0x%x, bytes = %d)",
					input,
					HRESULT_FROM_WIN32(GetLastError()),
					bytes);
			}

			printf("subtask: H265 decoder reserved\n");

			doneEvent.set();

			printf("subtask: releasing H265 decoder\n");

			if (!DeviceIoControl(
				vpu2,
				IOCTL_VPU_DEC_RELEASE,
				&out,
				sizeof(out),
				nullptr,
				0,
				&b,
				nullptr) || (b != 0)) {

				failed = TRUE;
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"IOCTL_VPU_DEC_RELEASE(%d) failed. "
					L"(hr = 0x%x, bytes = %d)",
					input,
					HRESULT_FROM_WIN32(GetLastError()),
					bytes);
			}

			printf("subtask: H265 decoder released\n");
		}
		catch (wexception &err)
		{
			printf("subtask: execution failed HR=%08x\n"
				"%S\n",
				err.HResult(),
				err.wwhat());
		}
	});

	// wait 20 ms
	if (doneEvent.wait(20) == 0)
	{
		failed = TRUE;

		throw wexception::make(
			E_FAIL,
			L"done event is signaled before VPU released");
	}
	else
	{
		printf("Subtask is correctly blocked on IOCT_VPU_DEC_RESERVE\n");
	}

	printf("Closing file without releasing core %d\n", output);

	handle1.Close();

	// wait upto 20 ms, but should return immediately
	if (doneEvent.wait(20) != 0)
	{
		failed = TRUE;

		throw wexception::make(
			E_FAIL,
			L"done event was not signaled after file closed");
	}
	else
	{
		printf("Subtask correctly signaled done event\n");
	}

	tasks.wait();
	printf("Finished TestReserve/FileClose\n");
}
#endif
int __cdecl wmain(int argc, _In_reads_(argc) const wchar_t* argv[])
{
	printf("Csi Ioctl Test App\n\n");
	bool run_test = false;
	GetInterfacePath(GUID_DEVINTERFACE_IMXMIPI);
	if (argc < 2) {
		std::wcerr << L"Missing required command line parameter device_path.\n";
		run_test = true;
	}
	else {
		int argn = 0;
		appName = argv[argn++];

		while (argn < argc) {
			// unsigned int cmd = MyGetwch(); _putwch(WCHAR(cmd));

			// bool isDeffered = false;
			const wchar_t* arg = argv[argn++];
			wprintf(L"Command => %s\r\n", arg);

			if (!_wcsicmp(arg, L"-?") || !_wcsicmp(arg, L"/?") ||
				!_wcsicmp(arg, L"-h") || !_wcsicmp(arg, L"/h") ||
				!_wcsicmp(arg, L"-help") || !_wcsicmp(arg, L"/help")) {
				// PrintUsage();
				return 0;

			}
			// else if (!_wcsicmp(arg, L"-s") || !_wcsicmp(arg, L"/s")) {
			// 	cfg_only_get_frame = true;
			// }
			else if (!_wcsicmp(arg, L"-i") || !_wcsicmp(arg, L"/i")) {
				driver_id_str = *argv[argn++];
			}
			else if (!_wcsicmp(arg, L"-c") || !_wcsicmp(arg, L"/c")) {
				GtHsSettle_EscClk = (uint8_t)_wtoi(argv[argn++]);
			}
			else if (!_wcsicmp(arg, L"-l") || !_wcsicmp(arg, L"/l")) {
				csiLanes = (UINT8)(*argv[argn++] - '0');
			}
		}
	}
#if 0
	if (cfg_only_get_frame) {
		try {
			// UCHAR Value = 0;
			// do_write_irp(&Value, sizeof(Value), IOCTL_SNS0_DRIVER_RST_GPIO_SET);
			init_cam();
		}
		catch (const std::exception& ex) {
			std::wcerr << L"Error: " << ex.what() << L"\n";
			//forceQuit = true;
			return 1;
		}
		catch (wexception &err)
		{
			LPWSTR lpBuff;
			FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, NULL, err.HResult(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&lpBuff, 0, NULL);

			printf("Execution failed HR=%08x\n"
				"%S\n%S\n",
				err.HResult(),
				err.wwhat(), lpBuff);
			return -1;
		}
		catch (...) {
			std::wcerr << L"Error, unknown exception \n";
			//forceQuit = true;
			throw;
		}
	}
	else 
#endif
		if (run_test) {
		try
		{
			// basic tests
			test();
            // FIXME - Broken by design changes
			// TestMultiClient();
			// TestOverlappedIo();
			// TestReserveRelease();

			// TestReserveFileClose();
			putchar('\n');
			printf("All %d tests passed\n", tId);
		}
		catch (wexception &err)
		{
			LPWSTR lpBuff;
			FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, NULL, err.HResult(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&lpBuff, 0, NULL);

			printf("Test execution failed HR=%08x\n"
				"%S\n%S\n",
				err.HResult(),
				err.wwhat(), lpBuff);
			return -1;
		}
	}

	return 0;
}
